/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include <ANN/ANNx.h>	
#include "MR_ConfigurationProblem.h"

//===========================================================================
//
// Approximate Nearest Neighbor Search (kd-tree)
// Nodes are arrays of robots, distances computed from concatenated joint states
//

struct MR_ANN {
  unique_ptr<struct sANN> self;

  // X stores concatenated joint states of robot arrays (each row is one node)
  arr X;                     //the data set for which a ANN tree is built (rows=nodes, cols=concatenated joint state dim)
  uint bufferSize; //a tree is only rebuilt if there are more than 'buffer' new points appended [default: 100]

  MR_ANN();
  ~MR_ANN();

  void clear();              //clears the tree and X
  void append(const rai::Array<Robot>& robotArrayNode); //append a node (array of robots) to X
  void calculate();          //compute a tree for all of X

  uint getNN(const rai::Array<Robot>& queryNode, double eps=.0, bool verbose=false); //find nearest neighbor from query robot array
  void getkNN(arr& sqrDists, uintA& idx, const rai::Array<Robot>& queryNode, uint k, double eps=.0, bool verbose=false); //find k nearest neighbors
};
