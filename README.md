# RenesasXHCIFix — Renesas uPD720202 ExpressCard USB 3.0 fix for macOS Tahoe (OCLP)

A [Lilu](https://github.com/acidanthera/Lilu) kernel plugin that makes a no-name
**Renesas uPD720202** ExpressCard / USB 3.0 adapter actually work on
**macOS 26 Tahoe** running through **OpenCore Legacy Patcher (OCLP)** on a
**MacBook Pro 2011 (MacBookPro8,3)**.

If you found this because your USB 3.0 ExpressCard is detected but unusably slow on
macOS, read on — this is the same problem.

## Symptoms (what you probably searched for)

- Renesas **uPD720202** (PCI **1912:0015**) ExpressCard USB 3.0 on macOS: devices
  are detected but transfers are glacially slow (e.g. ~1 KB in 16 seconds), disks
  never mount or take minutes.
- The same card and devices work fine on another machine / in Linux.
- Happens under **OpenCore Legacy Patcher** on **macOS Ventura/Sonoma/Sequoia/Tahoe**
  on old Macs (tested on Tahoe 25G241, MacBookPro8,3, 2011).
- In the IORegistry the controller binds to the generic `AppleUSBXHCIPCI`
  (`EXCD@…`), not to a vendor subclass.

## Why it happens

The uPD720202 runs in **single-vector MSI** mode and raises no interrupt for events
on its secondary interrupters. `AppleUSBXHCI` therefore only notices command
completions on the ~**2.048 s** MFINDEX-wrap timer, so bulk transfers crawl. The
card and its firmware (2.0.2.6) are fine — the same card reads at 33 MB/s in Linux.
Apple ships vendor subclasses of `AppleUSBXHCI` for Intel/ASMedia/AMD/Fresco but
**none for Renesas**, so there is no interrupt handling tuned for this controller.

This was confirmed by reverse-engineering the exact `AppleUSBXHCI` x86_64 image
(LC_UUID `4F5E24C5-4FEA-32DA-A8AD-D2FEFEF3ABF2`):

- The generic single-MSI source is registered with `filterOccurred` + `interruptOccurred`;
  `interruptOccurred` dispatches the isochronous + io + primary handlers, i.e. it
  services *all* interrupter rings.
- Controller layout: interrupter array at `+0xe40` (8 slots), per-vector interrupt
  sources at `+0xe80`, power/run state at `+0x2a8` (working == 3) and `+0x2b0`
  (running == 1); each interrupter's event ring is at `+0x30` with dequeue index
  `| (CCS<<31)` at `+0x60`, and rings hold 1024 TRBs.
- `manuallyProcessInterrupts()` (controller vtable slot `0xbf0`) is the driver's own
  full service entry, normally invoked under the work-loop gate from the power path
  (`setPowerStateGated → lowerOnePowerStateTo`).

These offsets are specific to build 25G241 and are the reason for the LC_UUID gate.

## What this does

It periodically drives the driver's own interrupt processing so completions are
serviced promptly instead of once every ~2 seconds.

A private `IOTimerEventSource` is added to the controller's own `IOWorkLoop`. Its
action runs serialized under that work-loop's gate, peeks the event rings, and —
when something is pending — calls the driver's own `manuallyProcessInterrupts()`
(the full filter+action path that services all rings). No `signalInterrupt` and no
write to the hardware producer count. Lifecycle is handled by start/stop/power
hooks: a refcount keeps the held objects alive and an epoch counter blocks
re-arming across a power/stop transition. (This is the observed design, not a proof
of safety for every teardown — reentrant stop during an in-flight manual call, and
hot-removal, still need more testing; see the caveats below.)

The cadence is **self-limiting**: a 2 ms floor while there is recent I/O, a 32 ms
idle heartbeat, and a strict duty cap — after each manual call the *next* delay is
scaled from how long that call took, keeping our manual processing to ≤10 % of wall
time (it backs off *after* a slow call; it does not interrupt one in progress, nor
does it bound the driver's own CPU use), failing closed on any timing overflow.

It is **fully automatic**: just add the kext to your OpenCore config — no boot-args,
no switches. It installs its hooks **only** on the exact verified `AppleUSBXHCI`
image (an LC_UUID gate); on any other macOS build it installs nothing. The polling
timer is attached **only** to the Renesas card (PCI `1912:0015`) — no other USB
controller is polled.

> ⚠️ **Build-specific.** The reverse-engineered offsets are pinned to the macOS
> `AppleUSBXHCI` build **25G241** (x86_64) by the LC_UUID gate. On any other macOS
> build the plugin deliberately does nothing — it will not work until the offsets
> are re-verified for that build.

## Build

No Xcode needed — Apple Command Line Tools only (`xcode-select --install`). Two
dependencies go in a `vendor/` folder beside `RenesasXHCIFix/` (kept out of git).
It was built against these exact versions:

```sh
# from the repo root
mkdir -p vendor && cd vendor

# MacKernelSDK — kernel headers + libkmod, pinned to the commit used here
git clone https://github.com/acidanthera/MacKernelSDK.git
git -C MacKernelSDK checkout 3f750085caa17ec3a7880f11c11bf4f48cd6a164

# Lilu DEBUG SDK — extract Lilu.kext from the 1.7.1 DEBUG release
curl -LO https://github.com/acidanthera/Lilu/releases/download/1.7.1/Lilu-1.7.1-DEBUG.zip
mkdir -p lilu-debug && (cd lilu-debug && unzip -o ../Lilu-1.7.1-DEBUG.zip)
cd ..
```

Expected layout (`build.sh` reads these paths):

```
vendor/MacKernelSDK/Headers/
vendor/MacKernelSDK/Library/x86_64/libkmod.a
vendor/lilu-debug/Lilu.kext/Contents/Resources/Headers/
vendor/lilu-debug/Lilu.kext/Contents/Resources/Library/plugin_start.cpp
```

Then:

```sh
cd RenesasXHCIFix && ./build.sh      # -> build/RenesasXHCIFix.kext
```

## Install (OpenCore)

1. Copy `RenesasXHCIFix.kext` into `EFI/OC/Kexts/`.
2. Add it to `config.plist` under `Kernel → Add` (it depends on **Lilu**, so it must
   be listed **after** `Lilu.kext`), `MinKernel` `25.0.0`.
3. Reboot. The card should enumerate and read at the measured ~13 MB/s automatically
   (see [Status](#status) for what that number means).

> ⚠️ The built-in keyboard/trackpad freeze described below was seen once in testing.
> For your first try, boot from a **separate OpenCore entry / EFI**, not your daily one.

**Check it's active:**

```sh
sysctl -n kern.rxhci.stats
```

Under card load you should see the hooks installed and `interval_us=2000`. If it
reports it stayed inert, your `AppleUSBXHCI` does not match the LC_UUID gate — a
current Darwin 25 kernel is not enough on its own, it must be exactly build 25G241.

**To disable it:** set its `Kernel → Add` entry `Enabled=false` (or remove the entry)
and reboot, or add the `-rxhcioff` boot-arg. Don't `kextunload` it live — it is a
resident Lilu plugin.

## Status

The card works automatically on a normal boot: no boot stick and no manual
switches, ~13 MB/s. Confirmed on hardware — `interval_us = 2000` under load, 768 MB
read at 12.81 MB/s, keyboard/trackpad live throughout.

**Honest caveats.** An earlier 2 ms build once froze the built-in
keyboard/trackpad under sustained load after ~40 min; its exact source was not
kept, so there is no proven causal diff, and the concurrency hardening predates
that incident. On this build the freeze did **not** recur over 4.5 h of
provocation, but that is not a proof of a fix and not a long soak of this exact
binary. Sleep/wake cycles, card hot-removal under load, and every reset/error
teardown path are not yet exercised over a long run — test before relying on it.

## How this was made

This project was researched, reverse-engineered, implemented and reviewed
**with AI coding agents**, driven by the project owner:

- **Implementation — [Claude Code](https://claude.com/claude-code) (Anthropic),
  model: Claude Opus 4.8.** Diagnosis, binary reverse-engineering of `AppleUSBXHCI`,
  the kext implementation, and build/test tooling.
- **Independent code review — [OpenAI Codex](https://openai.com/codex/) (Codex CLI),
  model: GPT-6 Astra (`gpt-6-astra`).** Adversarial review of the design and code,
  independent disassembly to confirm offsets and lifecycle paths, and the review
  documents.

The two agents collaborated under the project owner's direction, who set the goals
and ran all on-hardware testing.

## License

The code in this repository is [MIT](LICENSE) © 2026 Max Dernovyi.

Built on [Lilu](https://github.com/acidanthera/Lilu) and
[MacKernelSDK](https://github.com/acidanthera/MacKernelSDK) by Acidanthera. A built
`.kext` links Lilu's `plugin_start.cpp` and `libkmod`, so any **binary**
redistribution must also carry their licenses (Lilu is BSD-3-Clause and requires the
copyright/conditions/disclaimer be reproduced); MIT covers only the code here.

Experimental software — use at your own risk; test before relying on it.
