/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "PathResult.h"
#include "RRT_PathFinder.h"
#include "D_RRT_PathFinder.h"
#include "ConfigurationProblem.h"
#include "../Optim/NLP.h"
#include "../Algo/ann.h"

//===========================================================================

namespace rai {

struct MR_PathFinder : NonCopyable {
  std::shared_ptr<ConfigurationProblem> problem;
  std::shared_ptr<SolverReturn> ret;
  std::shared_ptr<RRT_PathFinder> rrtSolver;
  rai::Configuration C;
  
  std::map<rai::String, arr> robots;
  arr starts;
  arr goals;
   
  void setProblem(const rai::Configuration& C, const arr& starts, const arr& goals, const std::map<rai::String, arr>& robots, double collisionTolerance=-1.);
  shared_ptr<SolverReturn> solve();
  shared_ptr<SolverReturn> shortcutPath(const arr& path);

};

} //namespace
