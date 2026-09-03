# Building the library and tests outside MSVC

The application target needs delta-studio, which is Windows and DirectX only.
The simulation library, the control layer and the test suite do not, so they
can be built and run headless:

```
cmake -B build -DENGINE_SIM_APP=OFF -DPIRANHA_ENABLED=ON -DDISCORD_ENABLED=OFF
cmake --build build --target engine-sim-test
./build/engine-sim-test
```

`cmake/msvc_compat.h` is force-included for non-MSVC compilers and supplies
`__forceinline`, `__int64` and the string and math headers MSVC pulls in
transitively.

## Requirements beyond the submodules

- flex and bison, for piranha's parser generator
- Boost filesystem, for piranha's path handling
- GoogleTest is fetched by CMake; behind a TLS-terminating proxy pass a local
  checkout with `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<path>`

## Two patches piranha needs

The piranha submodule does not compile with GCC or Clang. Both problems are
MSVC extensions that other compilers reject. They live in the submodule, so
this repository cannot carry them as source; it carries them as a patch:

```
git -C dependencies/submodules/piranha apply ../../../cmake/piranha-gcc.patch
```

Revert with `git -C dependencies/submodules/piranha checkout .` to leave the
submodule clean again.

`include/language_rules.h` declares four explicit specializations of
`getLiteralBuiltinName` inside the class body; the patch moves them to
namespace scope and marks them `inline`. `include/ir_value_constant.h`
specializes the member template `validateData` inside a class template, which
the language does not allow; the patch replaces the specialization with a
plain overload taking `const piranha::native_string &`, which overload
resolution picks for the string case.

With the patch applied `engine-sim-script-interpreter` builds and the script
tests run. Windows and MSVC builds need none of this.

## Pre-existing test failures

Four tests fail on the unmodified upstream code and are unrelated to the
powertrain work. They fail identically at the commit before it, verified by
building that commit in a separate worktree:

- `GasSystemTests.PressureEquilibriumMaxFlow`
- `GasSystemTests.PressureEquilibriumMaxFlowInfinite`
- `GasSystemTests.PressureEquilibriumMaxFlowInfiniteOverpressure`
- `FunctionTests.FunctionGaussianTest`

`SynthesizerTests.SynthesizerSystemTestSingleThread` does not terminate; run
the suite with `--gtest_filter=-SynthesizerTests.SynthesizerSystemTest*`.

## Which rev limit is the real one

There used to be three numbers that could disagree:

- `ignition_module(rev_limit: ...)` in the engine script, which drove the actual
  spark cut in `IgnitionModule`
- `ecu.limiter.rev_limit`, which drove only the ECU's soft ramp and fuel cut
- `engine(redline: ...)`, a third value

The engine control unit now commands the limiter. It writes `revLimit` and
`limiterDuration` into `ActuatorCommands`, and `PowertrainSystem::applyCommands`
pushes them into the ignition module every control tick. `ecu.limiter.rev_limit`
is therefore the only number that decides where the engine stops pulling, and a
drive mode or a browser slider that changes it moves the hard cut with it.

The ignition module's own limit is set to `rev_limit + hard_offset`, so the ECU's
soft ignition cut and hard fuel cut always act before the module's backstop.

`engine(redline: ...)` remains what it always was: the dyno sweep range and the
gauge marking. It does not limit anything.
