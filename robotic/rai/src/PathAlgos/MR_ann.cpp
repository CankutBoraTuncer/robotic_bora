/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MR_ann.h"


struct sANN {
  ANNkd_tree* tree=0;
  //PartialLeastSquares pls;
  rai::Array<double*> cpointers;
  uint treeSize;   //for how many entries in nodes have we build the tree?
  void clear() { if(tree) delete tree;   tree=nullptr;  cpointers.clear();  treeSize=0; }
};

MR_ANN::MR_ANN() {
  bufferSize = 100;
  self = make_unique<sANN>();
  self->tree = 0;
  self->treeSize = 0;
}

MR_ANN::~MR_ANN() {
  self->clear();
  //annClose(); //mt09-07-29 this would close down the MR_ANN lib completely
}

void MR_ANN::clear() {
  self->clear();
  X.clear();
}

uint MR_ANN::getNN(const rai::Array<Robot>& queryNode, double eps, bool verbose) {
  uintA idx;
  arr sqrDists;
  getkNN(sqrDists, idx, queryNode, 1, eps, verbose);
  return idx(0);
}

void MR_ANN::append(const rai::Array<Robot>& robotArrayNode) {
  // Append a node (array of robots) to the dataset
  // Concatenate joint states of all robots in the array
  arr concatenatedStates;
  for(const auto& robot : robotArrayNode) {
    concatenatedStates.append(robot.x);
  }
  
  if(!X.N) {
    self->clear();
    X = concatenatedStates;
    X.reshape(1, concatenatedStates.N);
  } else {
    double* p=X.p;
    X.append(concatenatedStates);
    if(X.N==concatenatedStates.N) X.reshape(X.N/(uint)concatenatedStates.N, (uint)concatenatedStates.N);
    if(X.p!=p) self->clear(); //when the memory location changed clear the tree! (implies recomputation)
  }
}

void MR_ANN::calculate() {
  // Build KD-tree from concatenated joint states of all robot arrays
  if(self->treeSize == X.d0) return;
  self->clear();
  self->cpointers = getCarray(X);
  self->tree = new ANNkd_tree(self->cpointers.p, X.d0, X.d1);
  self->treeSize = X.d0;
}

void MR_ANN::getkNN(arr& sqrDists, uintA& idx, const rai::Array<Robot>& queryNode, uint k, double eps, bool verbose) {
  // Find k nearest neighbors using concatenated joint state distances
  arr queryStates;
  for(const auto& robot : queryNode) {
    queryStates.append(robot.x);
  }
  
  CHECK_GE(X.d0, k, "data has less (" <<X.d0 <<") than k=" <<k <<" nodes");
  CHECK_EQ(queryStates.N, X.d1, "query node has wrong dimension. queryStates.N=" << queryStates.N << ", X.d1=" << X.d1);

  if(X.d0-self->treeSize>bufferSize) {
    if(verbose) std::cout <<"MR_ANN recomputing: X.d0=" <<X.d0 <<" treeSize=" <<self->treeSize <<std::endl;
    calculate();
  }
  uint restStartsAt;
  if(self->treeSize>=k) {
    sqrDists.resize(k);
    idx.resize(k);
    self->tree->annkSearch(queryStates.p, k, (int*)idx.p, sqrDists.p, eps);
    restStartsAt=self->treeSize;
  } else {
    sqrDists.clear();
    idx.clear();
    restStartsAt=0;
  }

  //now check if in the rest of X there are even nearer nodes
  arr nodeJointStates;
  for(uint i=restStartsAt; i<X.d0; i++) {
    for(uint j=0; j<=idx.N && j<k; j++) {
      nodeJointStates.referToDim(X, i);
      double d = sqrDistance(nodeJointStates, queryStates);
      if(j==idx.N || d < sqrDists(j)) {
        idx.insert(j, i);
        sqrDists.insert(j, d);
        break;
      }
    }
  }
  if(idx.N>k) {
    idx.resizeCopy(k);
    sqrDists.resizeCopy(k);
  }

  if(verbose) {
    std::cout
        <<"MR_ANN query:"
        <<"\n nodes count = " <<X.d0 <<"  concatenated state dim = " <<X.d1 <<"  treeSize = " <<self->treeSize
        <<"\n query node with concatenated joint states = " <<queryStates
        <<"\n found neighbors:\n";
    for(uint i=0; i<idx.N; i++) {
      std::cout <<' '
                <<i <<' '
                <<idx(i) <<'\t'
                <<sqrt(sqrDists(i)) <<'\t'
                <<std::endl;
    }
  }
}


