# sst-effects

`sst-effects` is a header-only set of templates which implement the 
effects used in surge, surge-fx surge for rack and short circuit. 
As of April 24 2023, they are a work in progress with only the Reverb
and Flanger effect being factored here. As we adapt more effects to
ShortCircuit and other properties, more effects will move quickly,
with the ultimate goal being all the current Surge effects other than
Airwindows and Nimbus being in this repository.

The effects use C++ templates to allow various configurations to interface
with the DSP code. To understand this strategy currently, your clearest
document is

1: The extensive header in include/sst/effects/EffectCore.h
2: The tests in tests/concrete-runs and tests/create-effect

This module depends on the surge modules 'sst-basic-blocks' and
'sst-filters'. If you are using this software in your project,
you must make those available at cmake time. If you build the tests
in this project in a standalone basis without those modules available
they will be retrieved via CPM (which is how our CI works).

Finally, these contain hand coded SSE2. If you want to compile for
non-X86 architectures, you must include `simde` or equivalent, and on
X86 architectures must include `<pmmintrin.h>` and so on. These headers
are purposefully fragile under SSE allowing you to make that choice externally.
You can see how the regtests accomplish this with `tests/simd-test-include.h`