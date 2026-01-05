/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MR_RRT_PathFinder.h"

#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../KOMO/pathTools.h"

#ifdef RAI_GL
#  include <GL/glew.h>
#  include <GL/gl.h>
#endif

#include <cmath>
#include <limits>


namespace rai {

void MR_PathFinder::setProblem(const Configuration& C, const arr& starts, const arr& goals, const std::map<rai::String, arr>& robots, double collisionTolerance) 
 {
  this->C = C;
  this->robots = robots;
  this->starts = starts;
  this->goals = goals;

  // Create a PathFinder instance and call its setProblem method
  problem = make_shared<ConfigurationProblem>(C, true, collisionTolerance, 1);
  problem->verbose=0;
  rrtSolver = make_shared<RRT_PathFinder>(*problem, starts, goals);
  cout <<"MR_RRT PathFinder: stepsize=" <<rrtSolver->stepsize
       <<", subsampleChecks=" <<rrtSolver->subsampleChecks
       <<", maxIters=" <<rrtSolver->maxIters
       <<", collisionTolerance=" <<collisionTolerance
       <<endl;
}

//===========================================================================

shared_ptr<SolverReturn> MR_PathFinder::solve() {
  // return the solve method from the RRT_PathFinder instance
  if(!ret) ret = make_shared<SolverReturn>();
  ret->time -= rai::cpuTime();
  rrtSolver->run();
  ret->time += rai::cpuTime();
  
  // Apply shortcutting to optimize the path
  if(rrtSolver->path.N > 0) {
    ret = shortcutPath(rrtSolver->path);
  } else {
    ret->x = rrtSolver->path;
  }
  
  ret->done = true;
  ret->feasible = (ret->x.N > 0);
  ret->evals = rrtSolver->iters;
  
  return ret;
}

//===========================================================================

shared_ptr<SolverReturn> MR_PathFinder::shortcutPath(const arr& path) {

  cout << "MR_RRT_PathFinder::shortcutPath called with path of length " << path.d0 << endl;
  // For each robot, find the average velocity along the path
  std::map<rai::String, double> avgVelocities;
  for (std::map<rai::String, arr>::const_iterator it = robots.begin(); it != robots.end(); ++it) {
    const rai::String& name = it->first;
    const arr& jointMask = it->second;
    double totalDist = 0.0;

    for (uint i = 1; i < path.d0; ++i) {
      const arr& cur = path[i];
      const arr& pre = path[i - 1];

      double stepSq = 0.0;
      // assume jointMask has same length as cur/pre and is 0/1
      for (uint j = 0; j < cur.N; ++j) {
        if (jointMask(j) != 0.0) {              // or > 0.5 if it's double-ish
          const double d = cur(j) - pre(j);
          stepSq += d * d;
        }
      }

      totalDist += std::sqrt(stepSq);
    }

    avgVelocities[name] = (path.d0 > 1) ? totalDist / double(path.d0 - 1) : 0.0;
  }

  // ---- find robot with maximum avg velocity ----
  rai::String maxRobot;
  double maxVel = -std::numeric_limits<double>::infinity();

  for (std::map<rai::String, double>::const_iterator it = avgVelocities.begin(); it != avgVelocities.end(); ++it) {
    if (it->second > maxVel) {
      maxVel = it->second;
      maxRobot = it->first;
    }
  }

  cout << "Max robot: " << maxRobot << ", max velocity: " << maxVel << ", path length: " << path.d0 << endl;


  // Slice the fastest robots joints from the path
  std::map<rai::String, arr> p_fast;
  for (uint i = 0; i < path.d0; ++i) {
    arr q = path[i];
    arr q_fast;
    const arr& jointMask = robots.at(maxRobot);
    for (uint j = 0; j < q.N; ++j) {
      if (jointMask(j) != 0.0) {              // or > 0.5 if it's double-ish
        q_fast.append(q(j));
      }
    }
    // Replace the original path with the sliced one
    p_fast[maxRobot].append(q_fast);
  }

  arr slow_start, slow_goal;
  const arr& fast_mask = robots.at(maxRobot);
  for (uint j = 0; j < starts.N; ++j) {
    if (fast_mask(j) == 0.0) {  // slow robot = inverse of fast robot mask
      slow_start.append(starts(j));
      slow_goal.append(goals(j));
    }
  }

  cout << "Slow start size: " << slow_start.N << ", slow goal size: " << slow_goal.N << endl;
  
  // Disable the joints of the fastest robot's frame in the configuratrion
  C.getFrame(maxRobot)->joint->makeRigid();
  
  C.view(true);

  // Run D_RRT for the slowest robot using the other robots frames as the input
  std::shared_ptr<rai::D_PathFinder> rrt = make_shared<rai::D_PathFinder>();
  rrt->setProblem(C, slow_start, slow_goal, p_fast, problem->collisionTolerance);
  auto ret = rrt->solve();
  cout << "D_RRT_PathFinder returned feasible: " << ret->feasible << ", path length: " << ret->x.d0 << endl;

  if (ret->feasible) {
    // Reconstruct the full path
    arr full_path;
    for (uint t = 0; t < ret->x.d0; ++t) {
      arr q_full = path[t];  // start with the original path configuration
      arr q_slow = ret->x[t];
      uint slow_idx = 0;
      for (uint j = 0; j < q_full.N; ++j) {
        if (robots[maxRobot](j) != 0.0) {
          q_full(j) = q_slow(slow_idx++);
        }
      }
      full_path.append(q_full);
    }
    ret->x = full_path;

    return ret;

  } else {
    return ret; 
  } 
} //namespace

}