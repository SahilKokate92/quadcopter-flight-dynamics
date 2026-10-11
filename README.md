# \# Quadcopter Flight Dynamics

# 

# A nonlinear quadcopter flight-dynamics simulation project featuring a C++ 6-DOF rigid-body model, four-rotor dynamics, motor response, quaternion-based attitude representation, and numerical integration using fourth-order Runge–Kutta (RK4).

# 

# The project focuses on developing, testing, and analyzing the mathematical model of a quadcopter.

# 

# \## Features

# 

# \- Nonlinear 6-DOF rigid-body dynamics.

# \- Quaternion-based attitude representation.

# \- Four-rotor thrust and reaction-torque model.

# \- First-order motor-speed dynamics.

# \- RK4 numerical integration.

# \- C++ unit tests and numerical validation cases.

# \- Open-loop asymmetric rotor simulation.

# \- CSV state logging and Python-based visualization.

# 

# \## Project Structure

# 

# ```text

# quadcopter-flight-dynamics/

# ├── controller/

# │   └── cpp/

# │       ├── include/

# │       ├── src/

# │       ├── tests/

# │       ├── examples/

# │       └── CMakeLists.txt

# ├── docs/

# │   ├── README.md

# │   ├── CPP\_TESTS.md

# │   ├── NUMERICAL\_MODEL.md

# │   ├── OPEN\_LOOP\_SIMULATION.md

# │   └── VALIDATION\_AND\_ASSUMPTIONS.md

# ├── README.md

# └── ...

# ```

# 

# The Python reference simulator and other existing project files are maintained separately from the C++ implementation.

# 

# \## Requirements

# 

# \- C++17-compatible compiler

# \- CMake

# \- Python 3

# \- Python packages: NumPy, pandas, and Matplotlib for the visualization script

# 

# \## Build the C++ Project

# 

# Run the following commands from the repository root.

# 

# \### 1. Configure

# 

# ```bash

# cmake -S controller/cpp -B controller/cpp/build

# ```

# 

# \### 2. Build

# 

# ```bash

# cmake --build controller/cpp/build

# ```

# 

# To build only the open-loop example:

# 

# ```bash

# cmake --build controller/cpp/build --target open\_loop\_asymmetric

# ```

# 

# \## Run the Open-Loop Example

# 

# Run the executable from the repository root.

# 

# \*\*Windows:\*\*

# 

# ```powershell

# .\\controller\\cpp\\build\\open\_loop\_asymmetric.exe

# ```

# 

# \*\*Linux:\*\*

# 

# ```bash

# ./controller/cpp/build/open\_loop\_asymmetric

# ```

# 

# The example runs a 15-second open-loop simulation with asymmetric rotor-speed commands. It records the vehicle state history to:

# 

# ```text

# controller/cpp/examples/open\_loop\_asymmetric\_state.csv

# ```

# 

# The exact executable location may differ depending on the CMake generator and build configuration.

# 

# \## Visualize the Results

# 

# Install the Python dependencies if necessary:

# 

# ```bash

# python -m pip install numpy pandas matplotlib

# ```

# 

# Generate the plots:

# 

# ```bash

# python controller/cpp/examples/visualize\_open\_loop.py

# ```

# 

# The script generates PNG figures in:

# 

# ```text

# controller/cpp/examples/results/

# ```

# 

# The plots show the 3D trajectory, full-state time histories, body angular rates, and rotor-speed responses.

# 

# \## Run the Tests

# 

# Build the project, then run the registered C++ tests:

# 

# ```bash

# ctest --test-dir controller/cpp/build --output-on-failure

# ```

# 

# The test suite covers individual model components, selected analytical comparisons, and numerical-stability checks.

# 

# \## Documentation

# 

# Detailed technical documentation is available in the \[`docs/`](docs/README.md) directory.

# 

# \- \[C++ Test Suite](docs/CPP\_TESTS.md)

# \- \[Numerical Model](docs/NUMERICAL\_MODEL.md)

# \- \[Open-Loop Simulation](docs/OPEN\_LOOP\_SIMULATION.md)

# \- \[Validation and Assumptions](docs/VALIDATION\_AND\_ASSUMPTIONS.md)

# 

# \## Scope

# 

# This is an engineering simulation and development project. Numerical tests and simulation results do not, by themselves, establish agreement with experimental flight data.

# 

# \## License

# 

# See the repository's license file for terms of use.



