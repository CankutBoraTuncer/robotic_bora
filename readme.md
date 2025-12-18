# Custom Rai Lib by Bora 

This repository originates from **Marc Toussaint** (MIT License).

## Adding C++ Changes
- Declare new classes/functions in headers (`*.h`), implement in sources (`*.cpp`).
- Put default arguments **only** in headers.
- Keep symbols in the correct namespace (e.g., `rai`).
- Share utilities in a common header/source to avoid redefinitions.
- Rebuild the project after changes and fix compiler errors.

## Adding Python Bindings (pybind11)
- Include the relevant C++ headers in the binding file (e.g., `ry/py-PathAlgos.cpp`).
- Add bindings with `pybind11::class_` / `m.def` using fully qualified names (e.g., `rai::MCR_PathFinder`).
- Ensure the module init function (e.g., `init_PathAlgos`) is called from `PYBIND11_MODULE` (e.g., `py-main.cpp`).
- Rebuild/reinstall the Python extension by `make -C build _robotic install -j$(nproc)` exposes the new bindings.
