//
//  kern_start.cpp
//  RenesasXHCIFix
//
//  Lilu plugin that revives the Renesas uPD720202 ExpressCard USB3 controller on
//  macOS 26 / build 25G241 (x86_64, via OpenCore Legacy Patcher). The controller
//  runs in single-vector MSI mode and raises no interrupt for events on its
//  secondary interrupters, so AppleUSBXHCI only services completions on the
//  2.048 s MFINDEX-wrap interrupt -> bulk transfers crawl. This plugin drives the
//  driver's own interrupt processing periodically.
//
//  DELIVERY: a private IOTimerEventSource on the controller's own IOWorkLoop. Its
//  action runs under that work loop's gate; when a RAM event-ring peek shows a
//  pending event it calls the driver's own manuallyProcessInterrupts() (full
//  filter+action, services all rings). No signalInterrupt, no hardware writes.
//
//  CADENCE (self-limiting):
//    - active floor kActiveMinUS (2 ms) while there was I/O within kRecentNS (100 ms);
//    - idle heartbeat kIdleUS (32 ms) otherwise, so a new transfer is noticed
//      within one heartbeat;
//    - strict duty cap: next active delay = max(kActiveMinUS, ceil(manual * 9)),
//      holding time inside manuallyProcessInterrupts to <=10% of wall time.
//      This bounds OUR duty, not the whole work loop's CPU budget.
//    At most one manual call per tick (no drain loop).
//
//  AUTOMATIC: no boot-args, no control sysctls. Hooks install only on the exact
//  verified image (LC_UUID); arms from the controller's start, pauses around power
//  transitions and stops before the driver frees its interrupt resources.
//
//  CONCURRENCY:
//   - All state mutators (timer action, the three gated driver hooks, commit
//     runAction) run under the controller's work-loop gate. The gate is recursive,
//     so manuallyProcessInterrupts may re-enter a gated path; the epoch re-check
//     catches a transition that happened during the manual call.
//   - gCtxLock protects only the single-slot reservation (gOccupied), gCtx + its
//     refcount, identity lookup and the diagnostic snapshot (gDiag).
//     Lock order is always gate -> registry.
//   - The single slot is reserved at the start of attach and released only in
//     freeCtx, so two controller instances (e.g. a replug) cannot both publish.
//   - A manual refcount keeps retained objects alive past stop for an in-flight
//     action; epoch is bumped on every pause/stop/power entry to forbid re-arming.
//   - In-flight manual vs. driver teardown is NOT proven safe; the reentry
//     counters in kern.rxhci.stats record whether it is ever reached.
//
//  Build-specific layout (guarded by the LC_UUID gate):
//    controller + 0xe40 .. +0xe78 : 8 AppleUSBXHCIInterrupter* (slots 0x1c8..0x1cf)
//    controller + 0x2a8 : uint64 power/run state; working == 3
//    controller + 0x2b0 : uint32; running == 1
//    AppleUSBXHCIInterrupter + 0x30 : event-ring base (1024 TRBs)
//    AppleUSBXHCIInterrupter + 0x60 : dequeue index | (CCS << 31)
//    event TRB + 0xc : control dword; bit0 = cycle bit
//

#include <Headers/plugin_start.hpp>
#include <Headers/kern_api.hpp>
#include <Headers/kern_util.hpp>
#include <Headers/kern_patcher.hpp>

#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOLocks.h>
#include <libkern/c++/OSData.h>
#include <libkern/OSAtomic.h>
#include <sys/sysctl.h>
#include <mach-o/loader.h>
#include <mach/machine.h>
#include <kern/clock.h>

static const char *pathXHCI[] {
	"/System/Library/Extensions/IOUSBHostFamily.kext/Contents/PlugIns/AppleUSBXHCI.kext/Contents/MacOS/AppleUSBXHCI"
};

static KernelPatcher::KextInfo kextXHCI {
	"com.apple.driver.usb.AppleUSBXHCI", pathXHCI, arrsize(pathXHCI), {true}, {}, KernelPatcher::KextInfo::Unloaded
};

// Cadence (see header). Measured operating point, not an arbitrary A/B knob.
static constexpr uint32_t kActiveMinUS   = 2000;          // active-mode floor (2 ms; validated over 4.5 h sustained card load, no freeze)
static constexpr uint32_t kIdleUS        = 32000;         // idle heartbeat
static constexpr uint64_t kRecentNS      = 100000000ULL;  // 100 ms activity window
static constexpr uint32_t kDutyNum       = 9;             // (1-b)/b for b=0.1
// No active upper cap: the duty delay backs off proportionally to manual cost so
// the 10% contract holds strictly. setTimeoutUS takes UInt32 us (max ~4295 s),
// which covers any realistic duty delay, so no debt/deadline mechanism is needed;
// kUInt32MaxUS is only a final overflow guard that is never hit in practice.
static constexpr uint32_t kUInt32MaxUS   = 0xFFFFFFFFu;

static constexpr size_t  kSlotFirst      = 0x1c8;  // interrupter array (ctrl+0xe40) start index
static constexpr size_t  kSlotCount      = 8;
static constexpr size_t  kOffPowerState  = 0x2a8;  // uint64, working == 3
static constexpr size_t  kOffRunState    = 0x2b0;  // uint32, running == 1
static constexpr size_t  kOffRing        = 0x30;   // interrupter -> ring base
static constexpr size_t  kOffDeqIdx      = 0x60;   // interrupter -> idx | CCS<<31
static constexpr size_t  kOffTrbControl  = 0xc;    // TRB -> control dword
static constexpr uint32_t kRingMask      = 0x3ff;  // 1024 TRBs

// --- image gate --------------------------------------------------------------

static const uint8_t kAppleUSBXHCIUUID[16] {
	0x4F,0x5E,0x24,0xC5, 0x4F,0xEA, 0x32,0xDA, 0xA8,0xAD, 0xD2,0xFE,0xFE,0xF3,0xAB,0xF2
};
static bool imageVerified;
static bool hooksReady;

static bool verifyImageUUID(mach_vm_address_t base, size_t size) {
	if (!base || size < sizeof(mach_header_64))
		return false;
	auto mh = reinterpret_cast<const mach_header_64 *>(base);
	if (mh->magic != MH_MAGIC_64 || mh->cputype != CPU_TYPE_X86_64 ||
		mh->filetype != MH_KEXT_BUNDLE)
		return false;
	if (mh->sizeofcmds > size - sizeof(mach_header_64))
		return false;
	if (mh->ncmds > mh->sizeofcmds / sizeof(load_command))
		return false;
	const uint8_t *end = reinterpret_cast<const uint8_t *>(base) + sizeof(mach_header_64) + mh->sizeofcmds;
	const uint8_t *p = reinterpret_cast<const uint8_t *>(base) + sizeof(mach_header_64);
	bool found = false, match = false;
	for (uint32_t i = 0; i < mh->ncmds; i++) {
		if (static_cast<size_t>(end - p) < sizeof(load_command))
			return false;
		auto lc = reinterpret_cast<const load_command *>(p);
		if (lc->cmdsize < sizeof(load_command) || (lc->cmdsize & 7) != 0 ||
			static_cast<size_t>(end - p) < lc->cmdsize)
			return false;
		if (lc->cmd == LC_UUID) {
			if (found)
				return false;
			found = true;
			if (lc->cmdsize < sizeof(uuid_command))
				return false;
			auto uc = reinterpret_cast<const uuid_command *>(p);
			match = true;
			for (int j = 0; j < 16; j++)
				if (uc->uuid[j] != kAppleUSBXHCIUUID[j]) {
					match = false;
					break;
				}
		}
		p += lc->cmdsize;
	}
	return found && match && p == end;
}

// --- context + state ---------------------------------------------------------

enum Lifecycle { Empty, Ready, Paused, Stopping, Terminal };

struct Ctx {
	IOService          *ctrl;
	IOWorkLoop         *wl;
	IOTimerEventSource *timer;
	OSObject           *interrupters[kSlotCount];
	size_t              num;
	volatile SInt32     refs;
};

static Ctx      gStorage;              // single controller
static Ctx     *gCtx;                  // published pointer, under gCtxLock
static bool     gOccupied;             // single-slot reservation, under gCtxLock
static IOLock  *gCtxLock;

// gate-owned state (mutated only from gated contexts)
static volatile int      gLifecycle = Empty;
static volatile int      gPowerReady;
static volatile int      gArmed;
static volatile uint32_t gEpoch;
static volatile int      gTransitionDepth;
static uint64_t          gLastActivity; // ns uptime of last processed event (gated)

// monotonic diagnostic counters (independent atomics, read lock-free)
static volatile SInt32 gTicks, gBusy, gArmFail;
// Teardown-overlap counters (events are independent; slight read skew is fine).
// The reentry checks are scoped to OUR poll's in-flight manual only: a global
// in-flight flag would false-positive on another XHCI's native manual on a
// different controller, which is not a reentry of our card.
static volatile SInt32 gPollManual;      // manuallyProcessInterrupts via our poll
static volatile SInt32 gNativeManual;    // native driver calls (ALL controllers)
static volatile SInt32 gActivePoll;      // our poll's manual in flight (this card)
static volatile SInt32 gStopDuringPoll;  // stop hook seen while our poll's manual ran
static volatile SInt32 gPowerDuringPoll; // power hook seen while our poll's manual ran
static volatile SInt32 gNotInGate;       // a gated point reached without holding the gate

// consistent read-only snapshot: gated writers refresh it under gCtxLock,
// the sysctl reads it under the same lock. manualCount is the denominator
// for the average, kept together with the totals so the sysctl reads a matched
// pair under one lock.
struct Diag {
	int      life, powerReady, armed, refs;
	uint32_t epoch, num, lastIntervalUS;
	uint64_t manualCount;
	uint64_t manualMaxNS, manualTotalNS;
};
static Diag gDiag;

static mach_vm_address_t orgStart, orgStop, orgPower, orgManualProc;
static void wrapManualProc(void *that); // defined below; poll's call target

static inline uint64_t nowNS() {
	uint64_t abs = 0, ns = 0;
	clock_get_uptime(&abs);
	absolutetime_to_nanoseconds(abs, &ns);
	return ns;
}

// Refresh the diagnostic snapshot from gate-owned state under the registry lock.
// Called from gated contexts; lastIntervalUS/manualNS are 0 when not applicable.
static void publishDiag(uint32_t lastIntervalUS, uint64_t manualNS) {
	IOLockLock(gCtxLock);
	gDiag.life        = gLifecycle;
	gDiag.powerReady  = gPowerReady;
	gDiag.armed       = gArmed;
	gDiag.epoch       = gEpoch;
	gDiag.refs        = gCtx ? gCtx->refs : 0;
	gDiag.num         = gCtx ? static_cast<uint32_t>(gCtx->num) : 0;
	// lastIntervalUS is passed non-zero ONLY by a successful arm, so interval_us
	// always reflects the last timeout actually set, never a failed one.
	if (lastIntervalUS)
		gDiag.lastIntervalUS = lastIntervalUS;
	if (manualNS) {
		gDiag.manualCount++;
		gDiag.manualTotalNS += manualNS;
		if (manualNS > gDiag.manualMaxNS)
			gDiag.manualMaxNS = manualNS;
	}
	IOLockUnlock(gCtxLock);
}

// registry: acquire a ref to the published context iff it belongs to `ctrl`.
static Ctx *acquireFor(void *ctrl) {
	IOLockLock(gCtxLock);
	Ctx *c = gCtx;
	if (c && c->ctrl == ctrl)
		OSIncrementAtomic(&c->refs);
	else
		c = nullptr;
	IOLockUnlock(gCtxLock);
	return c;
}

static void freeCtx(Ctx *c) {
	// Last reference gone (always from a gated context). The timer was already
	// cancelled + removed from the work loop (at stop / attach rollback).
	if (c->timer) { c->timer->release(); c->timer = nullptr; }
	for (size_t i = 0; i < c->num; i++)
		if (c->interrupters[i]) { c->interrupters[i]->release(); c->interrupters[i] = nullptr; }
	c->num = 0;
	if (c->wl)   { c->wl->release();   c->wl = nullptr; }
	if (c->ctrl) { c->ctrl->release(); c->ctrl = nullptr; }
	gLifecycle = Empty;        // gate-owned; allow a fresh instance to re-attach
	// Publish the Empty snapshot while the reservation is STILL held, so no
	// new instance on another work loop can be mutating gate-owned globals as we
	// read them. gCtx was already unpublished by stop, so this reads Empty/0/0.
	publishDiag(0, 0);
	IOLockLock(gCtxLock);
	if (gCtx == c)
		gCtx = nullptr;
	gOccupied = false;         // release the single-slot reservation
	IOLockUnlock(gCtxLock);
}

static void putCtx(Ctx *c) {
	if (OSDecrementAtomic(&c->refs) == 1) // old value 1 -> now 0
		freeCtx(c);
}

static bool readPowerReady(IOService *ctrl) {
	if (ctrl->isInactive())
		return false;
	uint64_t ps = *reinterpret_cast<volatile uint64_t *>(reinterpret_cast<uint8_t *>(ctrl) + kOffPowerState);
	uint32_t rs = *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uint8_t *>(ctrl) + kOffRunState);
	return ps == 3 && rs == 1;
}

static bool anyEventPending(Ctx *c) {
	for (size_t i = 0; i < c->num; i++) {
		uint8_t *intr = reinterpret_cast<uint8_t *>(c->interrupters[i]);
		void *ring = *reinterpret_cast<void *volatile *>(intr + kOffRing);
		if (!ring)
			continue;
		uint32_t idx = *reinterpret_cast<volatile uint32_t *>(intr + kOffDeqIdx);
		uint32_t ccs = idx >> 31;
		uint32_t off = (idx & kRingMask) << 4;
		uint32_t ctl = *reinterpret_cast<volatile uint32_t *>(static_cast<uint8_t *>(ring) + off + kOffTrbControl);
		if ((ctl & 1) == ccs)
			return true;
	}
	return false;
}

// Choose the next delay. `busy` = an event was processed this tick; `manualNS` =
// wall time spent inside manuallyProcessInterrupts (0 if not called). Active
// (recent I/O) => floor 2 ms, stretched STRICTLY by the duty cap with no upper
// clamp so 10% holds even for an expensive manual; otherwise idle heartbeat.
// Returns 0 to mean "delay not representable -> fail closed, do NOT re-arm"
// (unreachable in practice: the multiplication guard trips only at a ~63-year
// manual, and the UInt32-us bound only at a ~8-minute manual).
// floorUS is the active-mode minimum spacing (always kActiveMinUS in this final
// build; kept as a parameter so the function stays the host-tested unit).
static uint32_t nextIntervalUS(bool busy, uint64_t manualNS, uint64_t now, uint32_t floorUS) {
	bool recent = busy || (now - gLastActivity) < kRecentNS;
	if (!recent)
		return kIdleUS;
	// ceil(manualNS * (1-b)/b) in microseconds, b = 0.1  ->  manualNS*9/1000.
	if (manualNS > (0xFFFFFFFFFFFFFFFFULL - 999ULL) / kDutyNum)
		return 0;                                 // manualNS*9 would overflow uint64
	uint64_t dutyUS = (manualNS * kDutyNum + 999ULL) / 1000ULL;
	if (dutyUS <= floorUS)
		return floorUS;
	if (dutyUS >= kUInt32MaxUS)
		return 0;                                 // not representable in setTimeoutUS
	return static_cast<uint32_t>(dutyUS);
}

// --- timer action (gated) ----------------------------------------------------

static void pollAction(OSObject *owner, IOTimerEventSource *sender) {
	// Identity: only the published context for this exact timer+controller.
	IOLockLock(gCtxLock);
	Ctx *c = gCtx;
	if (c && c->ctrl == owner && c->timer == sender)
		OSIncrementAtomic(&c->refs);
	else
		c = nullptr;
	IOLockUnlock(gCtxLock);
	if (!c)
		return;

	gArmed = 0;  // honest: not armed while the action runs
	if (gLifecycle != Ready || !gPowerReady) {
		putCtx(c);                       // paused/stopping -> no reschedule
		return;
	}
	publishDiag(0, 0);                   // show armed=0 during a long manual

	uint32_t ep = gEpoch;
	OSIncrementAtomic(&gTicks);
	if (!c->wl->inGate())                // expected to be gated; count anomalies
		OSIncrementAtomic(&gNotInGate);

	bool busy = anyEventPending(c);
	uint64_t manualNS = 0;
	uint64_t t_now;
	if (busy) {
		OSIncrementAtomic(&gBusy);
		OSIncrementAtomic(&gPollManual);
		uint64_t m0 = nowNS();
		OSIncrementAtomic(&gActivePoll);   // scoped to this card's poll
		FunctionCast(wrapManualProc, orgManualProc)(c->ctrl);
		OSDecrementAtomic(&gActivePoll);
		t_now = nowNS();
		manualNS = t_now - m0;
		gLastActivity = t_now;           // gate-owned; reset the activity window
	} else {
		t_now = nowNS();
	}

	// Re-arm only if nothing (stop/pause/power/reset) changed under us. gCtx is
	// re-read under the registry lock because a re-entrant stop (if manual opened
	// the gate) unpublishes it under that lock; the epoch catches the transition.
	IOLockLock(gCtxLock);
	bool still = (gCtx == c);
	IOLockUnlock(gCtxLock);

	uint32_t next = 0, applied = 0;
	if (still && gLifecycle == Ready && gPowerReady && gEpoch == ep) {
		next = nextIntervalUS(busy, manualNS, t_now, kActiveMinUS);
		// next == 0 means "not representable": fail closed, do not re-arm.
		if (next != 0 && sender->setTimeoutUS(next) == kIOReturnSuccess)
			{ gArmed = 1; applied = next; }
		else { gArmed = 0; OSIncrementAtomic(&gArmFail); }
	}
	publishDiag(applied, manualNS);      // interval published ONLY if armed
	putCtx(c);
}

// --- attach (gated commit) ---------------------------------------------------

// Runs under the controller's work-loop gate (via runAction). target == ctrl,
// arg0 == the already-added IOTimerEventSource, arg1 == the work loop. Publishes
// atomically or fails; the reservation is already held by attach().
static IOReturn commitAction(OSObject *target, void *arg0, void *arg1, void *, void *) {
	IOService *ctrl = OSDynamicCast(IOService, target);
	IOTimerEventSource *timer = static_cast<IOTimerEventSource *>(arg0);
	IOWorkLoop *wl = static_cast<IOWorkLoop *>(arg1);
	if (!ctrl || ctrl->isInactive() || gLifecycle != Empty || gCtx)
		return kIOReturnNotReady;

	// Capture the interrupter objects under the gate.
	OSObject *in[kSlotCount];
	size_t num = 0;
	OSObject **slots = reinterpret_cast<OSObject **>(ctrl);
	for (size_t k = 0; k < kSlotCount; k++) {
		OSObject *o = slots[kSlotFirst + k];
		if (!o)
			continue;
		if (!o->metaCast("AppleUSBXHCIInterrupter"))
			return kIOReturnUnsupported;
		in[num++] = o;
	}
	if (num == 0)
		return kIOReturnNoDevice;

	ctrl->retain();
	wl->retain();
	for (size_t i = 0; i < num; i++)
		in[i]->retain();
	gStorage.ctrl = ctrl;
	gStorage.wl = wl;
	gStorage.timer = timer;          // Ctx owns the factory reference
	gStorage.num = num;
	for (size_t i = 0; i < num; i++)
		gStorage.interrupters[i] = in[i];
	gStorage.refs = 1;               // publish reference

	gPowerReady = readPowerReady(ctrl) ? 1 : 0;
	gEpoch++;
	gLifecycle = gPowerReady ? Ready : Paused;
	gArmed = 0;
	gLastActivity = nowNS();         // start the activity window "active"
	if (gLifecycle == Ready) {
		if (timer->setTimeoutUS(kActiveMinUS) == kIOReturnSuccess)
			gArmed = 1;
		else
			OSIncrementAtomic(&gArmFail);
	}

	IOLockLock(gCtxLock);
	gCtx = &gStorage;
	IOLockUnlock(gCtxLock);
	publishDiag(gArmed ? kActiveMinUS : 0, 0);
	return kIOReturnSuccess;
}

static void releaseReservation() {
	IOLockLock(gCtxLock);
	gOccupied = false;
	IOLockUnlock(gCtxLock);
}

static void attach(IOService *ctrl) {
	if (!hooksReady || !ctrl || ctrl->isInactive())
		return;

	IOService *provider = ctrl->getProvider();
	OSData *vid = provider ? OSDynamicCast(OSData, provider->getProperty("vendor-id")) : nullptr;
	OSData *did = provider ? OSDynamicCast(OSData, provider->getProperty("device-id")) : nullptr;
	if (!vid || vid->getLength() < 2 ||
		*static_cast<const uint16_t *>(vid->getBytesNoCopy()) != 0x1912 ||
		!did || did->getLength() < 2 ||
		*static_cast<const uint16_t *>(did->getBytesNoCopy()) != 0x0015)
		return;

	// Reserve the single slot atomically. A second candidate (e.g. a replug
	// whose old instance has not finished reclamation) is rejected here, before
	// touching any shared state.
	IOLockLock(gCtxLock);
	if (gOccupied || gCtx) {
		IOLockUnlock(gCtxLock);
		return;
	}
	gOccupied = true;
	IOLockUnlock(gCtxLock);

	// Hold the work loop across addEventSource + runAction.
	IOWorkLoop *wl = ctrl->getWorkLoop();
	if (!wl) {
		releaseReservation();
		return;
	}
	wl->retain();

	IOTimerEventSource *timer = IOTimerEventSource::timerEventSource(ctrl, pollAction);
	if (!timer) {
		wl->release();
		releaseReservation();
		return;
	}
	if (wl->addEventSource(timer) != kIOReturnSuccess) {
		timer->release();
		wl->release();
		releaseReservation();
		return;
	}

	// Capture / publish / arm all happen under the gate in commitAction.
	IOReturn r = wl->runAction(commitAction, ctrl, timer, wl);
	if (r != kIOReturnSuccess) {
		wl->removeEventSource(timer);
		timer->release();
		wl->release();
		releaseReservation();         // nothing published; free the slot now
		SYSLOG("rxhci", "attach: commit failed 0x%x", r);
		return;
	}
	wl->release();                    // commit took its own retain into gStorage
	SYSLOG("rxhci", "attached (powerReady=%d)", gPowerReady);
}

// --- AppleUSBXHCI lifecycle hooks --------------------------------------------

static bool wrapStart(void *that, void *provider) {
	bool ok = FunctionCast(wrapStart, orgStart)(that, provider);
	if (ok)
		attach(static_cast<IOService *>(that));
	return ok;
}

static IOReturn wrapStopThreadCallGated(void *that) {
	// Take ownership of the publish reference and unpublish, iff it is ours.
	IOLockLock(gCtxLock);
	Ctx *c = nullptr;
	if (gCtx && gCtx->ctrl == that) {
		c = gCtx;
		gCtx = nullptr;             // new readers can no longer find it
	}
	IOLockUnlock(gCtxLock);
	if (!c)
		return FunctionCast(wrapStopThreadCallGated, orgStop)(that);

	// Did our poll's manual overlap this stop, and are we gated?
	if (gActivePoll > 0)
		OSIncrementAtomic(&gStopDuringPoll);
	if (c->wl && !c->wl->inGate())
		OSIncrementAtomic(&gNotInGate);

	gLifecycle = Stopping;          // forbid any arm/rearm
	gEpoch++;
	gArmed = 0;
	c->timer->cancelTimeout();
	c->wl->removeEventSource(c->timer);

	IOReturn r = FunctionCast(wrapStopThreadCallGated, orgStop)(that);

	gLifecycle = Terminal;
	publishDiag(0, 0);
	putCtx(c);                      // drop publish ref AFTER orig; frees once readers done
	return r;
}

static IOReturn wrapSetPowerStateGated(void *that, unsigned long powerState, void *whatDevice) {
	Ctx *c = acquireFor(that);
	if (c) {
		if (gActivePoll > 0)                   // overlap counter (scoped to our poll)
			OSIncrementAtomic(&gPowerDuringPoll);
		if (c->wl && !c->wl->inGate())
			OSIncrementAtomic(&gNotInGate);
		gTransitionDepth++;
		gPowerReady = 0;
		gEpoch++;
		gArmed = 0;
		c->timer->cancelTimeout();
	}
	IOReturn r = FunctionCast(wrapSetPowerStateGated, orgPower)(that, powerState, whatDevice);
	if (c) {
		int depth = --gTransitionDepth;
		// Resume only at the outermost transition to the working power state 3,
		// and only if this instance is still the published, non-stopped one.
		IOLockLock(gCtxLock);
		bool still = (gCtx == c);
		IOLockUnlock(gCtxLock);
		if (depth == 0 && r == kIOReturnSuccess && powerState == 3 && still &&
			gLifecycle != Stopping && gLifecycle != Terminal &&
			readPowerReady(c->ctrl)) {
			gLifecycle = Ready;
			gPowerReady = 1;
			gEpoch++;
			gLastActivity = nowNS();       // treat resume as recent activity
			if (c->timer->setTimeoutUS(kActiveMinUS) == kIOReturnSuccess)
				gArmed = 1;
			else { gArmed = 0; OSIncrementAtomic(&gArmFail); }
		}
		publishDiag(gArmed ? kActiveMinUS : 0, 0);
		putCtx(c);
	}
	return r;
}

// The driver's own calls to manuallyProcessInterrupts come through this routed
// trampoline (our poll calls orgManualProc directly, bypassing it), so this path
// counts NATIVE invocations distinctly from our poll's.
static void wrapManualProc(void *that) {
	// Native total across ALL controllers. We intentionally do NOT track in-flight
	// depth here: a scoped reentry check lives around our own poll call only, so a
	// concurrent native manual on another XHCI cannot false-positive it.
	OSIncrementAtomic(&gNativeManual);
	FunctionCast(wrapManualProc, orgManualProc)(that);
}

static void onKext(KernelPatcher &patcher, size_t id, mach_vm_address_t slide, size_t size) {
	if (id != kextXHCI.loadIndex)
		return;
	if (!verifyImageUUID(slide, size)) {
		SYSLOG("rxhci", "image gate: not the verified 25G241 AppleUSBXHCI - staying inert");
		return;
	}
	imageVerified = true;

	KernelPatcher::RouteRequest requests[] {
		{"__ZN12AppleUSBXHCI5startEP9IOService", wrapStart, orgStart},
		{"__ZN12AppleUSBXHCI19stopThreadCallGatedEv", wrapStopThreadCallGated, orgStop},
		{"__ZN12AppleUSBXHCI18setPowerStateGatedEmP9IOService", wrapSetPowerStateGated, orgPower},
		{"__ZN12AppleUSBXHCI25manuallyProcessInterruptsEv", wrapManualProc, orgManualProc},
	};

	if (patcher.routeMultiple(id, requests, slide, size, true, false)) {
		hooksReady = true;
		SYSLOG("rxhci", "hooks installed");
	} else {
		SYSLOG("rxhci", "failed to install hooks: %d - staying inert", patcher.getError());
		patcher.clearError();
	}
}

// --- read-only diagnostics ---------------------------------------------------

static int sysctlStats SYSCTL_HANDLER_ARGS {
	// Consistent snapshot: copy the published Diag under the registry lock. The
	// average uses manualCount + manualTotalNS from the SAME copy, so the
	// denominator is read once and cannot be zero-with-nonzero-total.
	IOLockLock(gCtxLock);
	Diag d = gDiag;
	IOLockUnlock(gCtxLock);

	int ticks = static_cast<int>(gTicks), busy = static_cast<int>(gBusy);
	uint64_t avgNS = d.manualCount ? d.manualTotalNS / d.manualCount : 0;

	char buf[512];
	int n = snprintf(buf, sizeof(buf),
		"image=%d hooks=%d state=%d powerReady=%d armed=%d epoch=%u refs=%d interrupters=%u\n"
		"interval_us=%u ticks=%d busy=%d idle=%d armFail=%d\n"
		"manual poll=%d native_all=%d count=%llu activePoll=%d max_us=%llu avg_us=%llu\n"
		"reentry stopDuringPoll=%d powerDuringPoll=%d notInGate=%d\n",
		imageVerified, hooksReady, d.life, d.powerReady, d.armed, d.epoch, d.refs, d.num,
		d.lastIntervalUS,
		ticks, busy, ticks - busy, static_cast<int>(gArmFail),
		static_cast<int>(gPollManual), static_cast<int>(gNativeManual),
		static_cast<unsigned long long>(d.manualCount),
		static_cast<int>(gActivePoll),
		static_cast<unsigned long long>(d.manualMaxNS / 1000),
		static_cast<unsigned long long>(avgNS / 1000),
		static_cast<int>(gStopDuringPoll), static_cast<int>(gPowerDuringPoll),
		static_cast<int>(gNotInGate));
	// snprintf returns the length it WOULD have written; clamp to the buffer so
	// SYSCTL_OUT never reads past it if the format ever grows.
	size_t len = (n < 0) ? 0 :
		(static_cast<size_t>(n) >= sizeof(buf) ? sizeof(buf) - 1 : static_cast<size_t>(n));
	return SYSCTL_OUT(req, buf, len + 1);
}

SYSCTL_NODE(_kern, OID_AUTO, rxhci, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "Renesas uPD720202 ExpressCard USB3");
SYSCTL_PROC(_kern_rxhci, OID_AUTO, stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, nullptr, 0,
	sysctlStats, "A", "status (read-only)");

static void pluginStart() {
	gCtxLock = IOLockAlloc();
	if (!gCtxLock) {
		SYSLOG("rxhci", "IOLockAlloc failed - inert");
		return;
	}
	sysctl_register_oid(&sysctl__kern_rxhci);
	sysctl_register_oid(&sysctl__kern_rxhci_stats);

	auto error = lilu.onKextLoad(&kextXHCI, 1,
		[](void *, KernelPatcher &patcher, size_t id, mach_vm_address_t slide, size_t size) {
			onKext(patcher, id, slide, size);
		});
	if (error != LiluAPI::Error::NoError)
		SYSLOG("rxhci", "onKextLoad failed: %d", error);
}

static const char *bootargOff[]   { "-rxhcioff" };
static const char *bootargDebug[] { "-rxhcidbg" };
static const char *bootargBeta[]  { "-rxhcibeta" };

PluginConfiguration ADDPR(config) {
	xStringify(PRODUCT_NAME),
	parseModuleVersion(xStringify(MODULE_VERSION)),
	LiluAPI::AllowNormal | LiluAPI::AllowInstallerRecovery | LiluAPI::AllowSafeMode,
	bootargOff, arrsize(bootargOff),
	bootargDebug, arrsize(bootargDebug),
	bootargBeta, arrsize(bootargBeta),
	KernelVersion::Tahoe,
	KernelVersion::Tahoe,
	[]() { pluginStart(); }
};
