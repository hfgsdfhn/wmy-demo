# C++ PID Controller

This library is a C++ rewrite of `代码库/PID.c` and `代码库/PID.h`.

`pid::PIDController` provides the original PID behavior:

- proportional, integral, and derivative terms
- integral and output limiting
- no derivative term on the first calculation after construction or reset
- resettable internal state

The library has no STM32 or HAL dependency.

## Build and run

```powershell
cmake -G "MinGW Makefiles" -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
.\build\pid_example.exe
```

## Basic use

```cpp
#include "pid/PIDController.hpp"

pid::PIDController controller(1.0F, 0.1F, 0.05F, 10.0F, 100.0F, 50.0F);
float output = controller.calculate(target, feedback);
```
