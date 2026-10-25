// Equivalent of the file Xcode generates for kext targets.
#include <mach/mach_types.h>

extern kern_return_t _start(kmod_info_t *, void *);
extern kern_return_t _stop(kmod_info_t *, void *);
extern kern_return_t RenesasXHCIFix_kern_start(kmod_info_t *, void *);
extern kern_return_t RenesasXHCIFix_kern_stop(kmod_info_t *, void *);

__attribute__((visibility("default"))) KMOD_EXPLICIT_DECL(com.dmlab.RenesasXHCIFix, "1.0.0", _start, _stop)
__private_extern__ kmod_start_func_t *_realmain = RenesasXHCIFix_kern_start;
__private_extern__ kmod_stop_func_t *_antimain = RenesasXHCIFix_kern_stop;
__private_extern__ int _kext_apple_cc = __APPLE_CC__;
