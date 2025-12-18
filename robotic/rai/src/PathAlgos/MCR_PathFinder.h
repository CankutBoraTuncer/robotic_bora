/* ------------------------------------------------------------------
    MCR (Minimum Constraint Removal) Path Planner Header
    --------------------------------------------------------------  */

#pragma once

#include "ConfigurationProblem.h"
#include "RRT_PathFinder.h" 
#include <vector>
#include <map>
#include "../Kin/kin.h"
#include "../Kin/frame.h"
#include "../Kin/proxy.h"

namespace rai {


// Graph Node for PRM
struct MCR_Node {
  arr pos;
  struct Edge {
    MCR_Node* target;
    double cost;
    uintA constraints; // Frame IDs of obstacles on this edge
  };
  std::vector<Edge> neighbors;
  
  MCR_Node(const arr& _pos) : pos(_pos) {}
};

// Result structure
struct MCRResult {
  arr path;
  double totalCost = 0.0;
  StringA removedConstraints; // Names of objects
  bool success = false;
};

// The Main Planner Class
struct MCR_PathFinder {
  rai::Configuration& C;
  rai::String agent;
  arr startPos;
  arr goalPos;
  uintA obsIndices;        // Frame IDs of the soft obstacles
  StringA obsList;
  uintA softObsIndices; 
  double penalty;
  int verbose;
  arr bounds_lo = {-2, -2};
  arr bounds_up = {2, 2};

  // Graph Storage
  std::vector<MCR_Node*> nodes;
  MCR_Node* startNode = nullptr;
  MCR_Node* goalNode = nullptr;

  MCR_PathFinder(rai::Configuration& _C, const rai::String _agent, const rai::String _goalFrame, const StringA& _obsList, double _penalty=500.0, int _verbose=0);

  // Helpers
  void checkCollisions(const arr& q, bool& isHard, uintA& softHits);
  double getEdgeData(const arr& from, const arr& to, uintA& constraints, double stepSize=0.1);
  bool hasPathToGoal();

  // Core Methods
  StringA solve(int maxIters=2000, double stepSize=0.5, double connRadius=1.5);
  MCRResult runAStar();
};

} // namespace rai

