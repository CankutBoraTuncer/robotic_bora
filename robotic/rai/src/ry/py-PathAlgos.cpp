/* ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../PathAlgos/RRT_PathFinder.h"
#include "../PathAlgos/MCR_PathFinder.h"

void init_PathAlgos(pybind11::module& m) {
  
  // 1. Existing PathFinder Class Binding
  pybind11::class_<rai::PathFinder, std::shared_ptr<rai::PathFinder>>(m, "PathFinder", "todo doc")
      .def(pybind11::init<>())
      .def("setProblem", &rai::PathFinder::setProblem, "", pybind11::arg("Configuration"), pybind11::arg("starts"), pybind11::arg("goals"), pybind11::arg("collisionTolerance")=1e-4, pybind11::arg("isIndependent")=false)
      .def("setExplicitCollisionPairs", &rai::PathFinder::setExplicitCollisionPairs, "only after setProblem", pybind11::arg("collisionPairs"))
      .def("solve", &rai::PathFinder::solve, "")
      .def("get_resampledPath", &rai::PathFinder::get_resampledPath, "")
      ;

  // 2. New MCR_PathFinder Class Binding
  pybind11::class_<rai::MCR_PathFinder, std::shared_ptr<rai::MCR_PathFinder>>(m, "MCR_PathFinder", "MCR Path Planner")
    .def(pybind11::init<rai::Configuration&, const rai::String, const rai::String, const StringA&, bool, bool,double, int>(),
            pybind11::arg("C"),
            pybind11::arg("agent"),
            pybind11::arg("goalFrame"),
            pybind11::arg("obsList"),
            pybind11::arg("persist")=false,
            pybind11::arg("earlyExit")=false,
            pybind11::arg("penalty")=500.0,
            pybind11::arg("verbose")=0)
    .def("solve", &rai::MCR_PathFinder::solve, "Solve the MCR problem", 
          pybind11::arg("maxIters")=2000, 
          pybind11::arg("stepSize")=0.5, 
          pybind11::arg("connRadius")=1.5)
    ;
}

#endif