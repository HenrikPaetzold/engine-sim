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
MSVC extensions that other compilers reject, and both are in the submodule,
not in this repository, so they are not carried in this tree.

`include/language_rules.h` declares four explicit specializations of
`getLiteralBuiltinName` inside the class body. Move them to namespace scope
after the class and mark them `inline`:

```cpp
template<> inline std::string LanguageRules::getLiteralBuiltinName<piranha::native_bool>() const {
    return *m_literalRules.lookup(LiteralType::Boolean);
}
```

`include/ir_value_constant.h` specializes the member template `validateData`
inside a class template, which the language does not allow. Replace the
specialization with a plain overload taking `const piranha::native_string &`;
overload resolution then picks it for the string case.

With those two changes `engine-sim-script-interpreter` builds and the script
tests run.

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
