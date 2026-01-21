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

#include <cstring> // std::strcmp

//===========================================================================

namespace {
template<class Map>
typename Map::iterator findLex(Map& m, const rai::String& key) {
  const char* kp = (const char*)key; if(!kp) kp = "";
  for(auto it = m.begin(); it != m.end(); ++it) {
    const char* ip = (const char*)it->first; if(!ip) ip = "";
    if(std::strcmp(ip, kp) == 0) return it;
  }
  return m.end();
}
}

MR_RRT_SingleTree::MR_RRT_SingleTree(const arr& q0, const shared_ptr<QueryResult>& q0_qr) {
//  if(!q0_qr->isFeasible) LOG(0) <<"rooting RRT with infeasible start configuration -- that's likely to fail: query is:\n" <<*q0_qr;
  add(q0, 0, q0_qr);
}

// -----------------------------------------------------------------

arr MR_RRT_SingleTree::getNodeByDepth(uint depth) {
  drawMutex.lock(RAI_HERE);
  for(uint i=0; i<queries.N; i++) {
    if(queries(i)->depth == depth) {
      arr q = ann.X[i].copy();
      drawMutex.unlock();
      return q;
    }
  }
  drawMutex.unlock();
  return NoArr;
}

// -----------------------------------------------------------------

uint MR_RRT_SingleTree::add(const arr& q, uint parentID, const shared_ptr<QueryResult>& _qr) {
  drawMutex.lock(RAI_HERE);
  ann.append(q);
  parent.append(parentID);
  
  // Set depth based on parent
  if(parent.N == 1) {
    // Root node
    _qr->depth = 0;
  } else {
    _qr->depth = queries(parentID)->depth + 1;
  }
  
  queries.append(_qr);
  disp3d.append(_qr->disp3d);
  disp3d.reshape(-1, 3);

  CHECK_EQ(parent.N, ann.X.d0, "");
  CHECK_EQ(queries.N, ann.X.d0, "");
  //CHECK_EQ(disp3d.d0, ann.X.d0, "");
  drawMutex.unlock();
  return parent.N-1;
}

// -----------------------------------------------------------------

double MR_RRT_SingleTree::getNearest(const arr& target) {
  //find NN
  nearestID = ann.getNN(target);
  return length(target - ann.X[nearestID]);
}

// -----------------------------------------------------------------

arr MR_RRT_SingleTree::getProposalTowards(const arr& target, double stepsize) {
  //find NN
  nearestID = ann.getNN(target);

  //compute default step
  arr delta = target - ann.X[nearestID]; //difference vector between q and nearest neighbor
  double dist = length(delta);
  if(dist>stepsize)  delta *= stepsize/dist;

  return getNode(nearestID) + delta;
}

arr MR_RRT_SingleTree::getNewSample(const arr& target, double stepsize, double p_sideStep, bool& isSideStep, const uint recursionDepth) {
  //find NN
  nearestID = ann.getNN(target);
  std::shared_ptr<QueryResult> qr = queries(nearestID);

  //compute default step
  arr delta = target - getNode(nearestID);
  double dist = length(delta);
  if(dist>stepsize) delta *= stepsize/dist;

  //without side stepping, we're done
  isSideStep = false;
  if(p_sideStep<=0. || recursionDepth >= 3) return getNode(nearestID) + delta;

  //check whether this is a predicted collision
  bool predictedCollision=false;
  if(qr->coll_y.N) {
    arr y = qr->coll_y + qr->coll_J * delta;
    if(min(y)<0.) predictedCollision = true;
  }

  if(predictedCollision && p_sideStep>0. && rnd.uni()<p_sideStep) {
    isSideStep=true;

    //compute new target
    arr d = qr->getSideStep();
    d *= rnd.uni(stepsize, 2.) / length(d);
    arr targ = getNode(nearestID) + d;
    bool tmp;
    return getNewSample(targ, stepsize, p_sideStep, tmp, recursionDepth + 1);
  } else {
    return getNode(nearestID) + delta;
  }

  HALT("shouldn't be here");
  return NoArr;
}

// -----------------------------------------------------------------


arr MR_RRT_SingleTree::getNewSample(const arr& target, const std::map<int, arr>& robots, double stepsize, double p_sideStep, bool& isSideStep, const uint recursionDepth) {
  //find NN
  nearestID = ann.getNN(target);
  std::shared_ptr<QueryResult> qr = queries(nearestID);


  const double vmax = stepsize;
  arr js = getNode(nearestID);
  arr delta = target - js;

  for (const auto& [robot, jointMask] : robots) {

    // compute norm over this robot's joints only
    double dist2 = 0.0;
    for (uint i = 0; i < jointMask.N; ++i) {
      if (jointMask(i) == 1) dist2 += delta(i) * delta(i);
    }

    double dist = sqrt(dist2);
    if (dist < 1e-12) continue;            // nothing to do

    if (dist > vmax) {
      double rat = vmax / dist;            // cap to vmax
      for (uint i = 0; i < jointMask.N; ++i) {
        if (jointMask(i) == 1) delta(i) *= rat;
      }
    }
  }

  arr JS = (js + delta).copy();

  //without side stepping, we're done
  isSideStep = false;
  cout << "p_sideStep: " << p_sideStep << " recursionDepth: " << recursionDepth << endl;
  if(p_sideStep<=0. || recursionDepth >= 3) return JS;

  
  
  cout << "Checking side stepping for robots: " << qr->coll_y_robots.size() << endl;
  for (const auto& [robot_name, jointMask] : robots) {
    //check whether this is a predicted collision for this robot
    bool predictedCollision=false;
    if(qr->coll_y_robots.find(robot_name) != qr->coll_y_robots.end()) {
      arr y = qr->coll_y_robots[robot_name] + qr->coll_J_robots[robot_name] * delta;
      cout << "Robot: " << robot_name << " Predicted collision values: " << y << endl;
      if(min(y)<0.) predictedCollision = true;
    }
    if(predictedCollision && p_sideStep>0. && rnd.uni()<p_sideStep) {
      isSideStep=true;
      cout << "SIDE STEPPING FOR ROBOT: " << robot_name << endl;

      //compute new target
      arr d = qr->getSideStep();
      d *= rnd.uni(stepsize, 2.) / length(d);
      arr targ = getNode(nearestID) + d;
      bool tmp;
      arr ns = getNewSample(targ, stepsize, p_sideStep, tmp, recursionDepth + 1);
      for (uint i = 0; i < jointMask.N; ++i) {
        if (jointMask(i) == 1) {
          JS(i) = ns(i);
        } 
      } 
    } 
  }

  return JS;

  //HALT("shouldn't be here");
  //return NoArr;
}

// -----------------------------------------------------------------
arr MR_RRT_SingleTree::getPathFromNode(uint fromID) {
  arr path;
  uint node = fromID;
  for(;;) {
    path.append(ann.X[node]);
    if(!node) break;
    node = getParent(node);
  }
  path.reshape(-1, ann.X.d1);
  return path;
}

//===========================================================================

bool MR_RRT_PathFinder::growTreeTowardsRandom(MR_RRT_SingleTree& rrt) {
  const arr start = rrt.ann.X[0];
  arr t(rrt.getNode(0).N);
  rndUniform(t, -RAI_2PI, false);
  HALT("DON'T USE 2PI")

  arr q = rrt.getProposalTowards(t, stepsize);

  auto qr = P.query(q);
  if(qr->isFeasible) {
    if(subsampleChecks>0 && !checkConnection(P, start, q, subsampleChecks, true)) {
      return false;
    }

    rrt.add(q, rrt.nearestID, qr);
    return true;
  }
  return false;
}

//===========================================================================
//===========================================================================

bool MR_RRT_PathFinder::growTreeToTree(MR_RRT_SingleTree& rrt_A, MR_RRT_SingleTree& rrt_B, bool forward) {
  bool isSideStep, isForwardStep;
  

  arr qG;
  double prob = forward ? 0.5 : 0.5;
  //decide on a target: forward or random
  arr t;
  if(rnd.uni()<prob) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
    #if 1
        t.resize(rrt_A.getNode(0).N);
        for(uint i=0; i<t.N; i++) {
          double lo=P.limits(0, i), up=P.limits(1, i);
          CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
          t.elem(i) = lo + rnd.uni()*(up-lo);
        }
    #else
        t.resize(rrt_A.getNode(0).N);
        rndUniform(t, -RAI_2PI, RAI_2PI, false);
    #endif
        isForwardStep = false;
  }

  int nearestID1 = rrt_A.ann.getNN(t);
  arr js = rrt_A.getNode(nearestID1);

  P.C.setJointState(js);
  //P.C.view(true, "RRT_GrowTreeToTree");

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, robots, stepsize, p_sideStep, isSideStep, 0);
  
  int nearestID2 = rrt_B.ann.getNN(q);
  qG = rrt_B.getNode(nearestID2);

  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P.query(q, robots, isFinished);

  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }


  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, robots,stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q, robots, isFinished);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  //checking subsamples
  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  if(qr->isFeasible){
    rrt_A.add(q, parentID, qr);
    int nearestID = rrt_B.ann.getNN(q);
    arr nearestNode = rrt_B.ann.X[nearestID];
    
    bool isConnected = false;
    // Check the distance for each robot's joints
    for (const auto& [robot, jointMask] : robots) {

      arr np;
      arr rp;
      for (uint i=0; i<jointMask.N; i++) {
        if (jointMask(i) == 1){
          rp.append(q(i));
          np.append(nearestNode(i));
  
        }

      }
      arr diff = rp - np;
      double dist = length(diff);
      cout << "Robot: " << robot << " Distance to nearest node: " << dist << " " <<  stepsize << " " << subsampleChecks << endl;
      if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) {isConnected = false; return true;} }
      else { if(dist<stepsize) {isConnected = false; return true;}}
    }
    //if (isConnected){return true;}

  }

  return false;
}

//===========================================================================
//===========================================================================

bool MR_RRT_PathFinder::growTreeToTree(MR_RRT_SingleTree& rrt_A, MR_RRT_SingleTree& rrt_B, std::map<int, shared_ptr<MR_RRT_SingleTree>>& rrtRobots_A, std::map<int, shared_ptr<MR_RRT_SingleTree>>& rrtRobots_B, bool forward) {
  bool isSideStep, isForwardStep;
  
  arr qG;
  double prob = forward ? 0.7 : 0.7;
  arr t;

  if(rnd.uni()<prob) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
    #if 1
        t.resize(rrt_A.getNode(0).N);
        for(uint i=0; i<t.N; i++) {
          double lo=P.limits(0, i), up=P.limits(1, i);
          CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
          t.elem(i) = lo + rnd.uni()*(up-lo);
        }
    #else
        t.resize(rrt_A.getNode(0).N);
        rndUniform(t, -RAI_2PI, RAI_2PI, false);
    #endif
        isForwardStep = false;
  }

  arr js = rrt_A.getNode(rrt_A.ann.getNN(t));


  
  if (forward) {
    for (const auto& [robot, path_tree] : rrtPathTrees){
      if (!isFinished[robot]) continue;
      cout << "Getting goal for robot: " << robot << endl;
      arr r_j;
      arr jointMask = robots[robot];

      for (uint i = 0; i < jointMask.N; i++) {
        if (jointMask(i) == 1) {
          r_j.append(js(i));
        }
      }
      uint depth = path_tree->ann.getNN(r_j) + 1;
      if (depth >= path_tree->parent.N) depth = path_tree->parent.N - 1;
      arr rq = path_tree->getNode(depth);
      
      uint k = 0;
      for (uint i = 0; i < jointMask.N; i++) {
        if (jointMask(i) == 1) {
          t(i) = rq(k);
          k++;
        }
      }
      cout << "Robot goal joints: " << rq << endl;
    }
  } else {
    for (const auto& [robot, path_tree] : rrtPathTrees){
      if (!isFinished[robot]) continue;
      cout << "Getting goal for robot: " << robot << endl;
      arr r_j;
      arr jointMask = robots[robot];

      for (uint i = 0; i < jointMask.N; i++) {
        if (jointMask(i) == 1) {
          r_j.append(js(i));
        }
      }
      uint depth = path_tree->ann.getNN(r_j) - 1;
      if (depth < 0) depth = 0;
      arr rq = path_tree->getNode(depth);
      
      uint k = 0;
      for (uint i = 0; i < jointMask.N; i++) {
        if (jointMask(i) == 1) {
          t(i) = rq(k);
          k++;
        }
      }
      cout << "Robot goal joints: " << rq << endl;
    }
  }
  
  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, robots, stepsize, p_sideStep, isSideStep, 0);

  P.C.setJointState(js);

  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P.query(q);

  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  
  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, robots,stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  //checking subsamples
  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  if(qr->isFeasible){
    rrt_A.add(q, parentID, qr);
    bool isConnected = true;
    

    for (const auto& [robot, robotMask] : robots) {

      if (isFinished[robot]) continue;


      arr q_robot;
      arr q_t;
      arr tt;
      for (uint i = 0; i < robotMask.N; i++) {
        if (robotMask(i) == 1) {
          q_robot.append(q(i));
          q_t.append(qT(i));
          tt.append(t(i));
        }
      } 

      auto itA = rrtRobots_A.find(robot);
      auto itB = rrtRobots_B.find(robot);
      shared_ptr<MR_RRT_SingleTree> r_A = itA->second;
      shared_ptr<MR_RRT_SingleTree> r_B = itB->second;

      int parentIDr = r_A->ann.getNN(q_robot);
      r_A->nearestID = parentID;
      auto qr_1 = P.query(q_robot, robot);
      r_A->add(q_robot, parentID, qr_1);

      uint nearestIDr = r_B->ann.getNN(q_robot);
      r_B->nearestID = nearestIDr;
      rrt_B.nearestID = nearestIDr;
      arr nearestNode = r_B->ann.X[r_B->ann.getNN(q_robot)];

      // nearestNode and q_robot are in the SAME (robot) space
      CHECK_EQ(nearestNode.N, q_robot.N, "robot-space dim mismatch for '" << robot << "'");
      double dist = length(q_robot - nearestNode);

      cout << "Robot: " << robot << " Distance to nearest node: " << dist << " " << stepsize << " " << subsampleChecks << endl;

      if(subsampleChecks>0) { if(dist > stepsize/subsampleChecks) { isConnected = false;} 
                              else {isFinished[robot] = true;} }

      else { if(dist > stepsize) {isConnected = false;} 
            else {isFinished[robot] = true; } }

      
     if(isFinished[robot]){
      cout << "Robot: " << robot << " is connected!" << endl;
       // Store the path for this robot as a tree  
      arr path_r;
      arr path_t;
      if (forward) {
        path_r = r_A->getPathFromNode(r_A->nearestID);
        path_t = r_B->getPathFromNode(r_B->nearestID);
      } else {
        path_r = r_B->getPathFromNode(r_B->nearestID);
        path_t = r_A->getPathFromNode(r_A->nearestID);
      }
       revertPath(path_r);
       path_r.append(path_t);
       rrtPaths[robot] = path_r;
       //Generate a tree using this path
       auto qr_path = P.query(path_r[0], robot);  
        shared_ptr<MR_RRT_SingleTree> rrt_path = make_shared<MR_RRT_SingleTree>(path_r[0], qr_path);
        uint nn =0;
        for (uint i = 1; i < path_r.d0; i++) {
          qr_path = P.query(path_r[i], robot);
          rrt_path->add(path_r[i], nn, qr_path);
          nn +=1;
        }
        rrtPathTrees[robot] = rrt_path;
      }
      cout << "isFinished status: " << isFinished[robot] << endl;
    }
    if (forward) {
      arr diff = q - qT;
      double dist = length(diff);
      
      if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) {return true;} }
      else { if(dist<stepsize) {return true;}}
    }
  }

  return false;
}

//===========================================================================

MR_RRT_PathFinder::MR_RRT_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, const std::map<int, arr>& _robots, const  double _stepsize, int _subsampleChecks, int _maxIters, int _verbose)
  : P(_P),
    stepsize(_stepsize),
    maxIters(_maxIters),
    verbose(_verbose),
    subsampleChecks(_subsampleChecks),
    robots(_robots) {

  if(stepsize<0.) stepsize = rai::getParameter<double>("rrt/stepsize", .1);
  if(subsampleChecks<0) subsampleChecks = rai::getParameter<int>("rrt/subsamples", 4);
  if(maxIters<0) maxIters = rai::getParameter<int>("rrt/maxIters", 5000);
  if(verbose<0) verbose = rai::getParameter<int>("rrt/verbose", 0);

  q0 = _starts;
  qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<MR_RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<MR_RRT_SingleTree>(qT, qTret);

  for (const auto& [robot, jointMask] : robots) {
    arr q0_robot, qT_robot;
    for (uint i = 0; i < jointMask.N; ++i) {
      if (jointMask(i) == 1) {
        q0_robot.append(q0(i));
        qT_robot.append(qT(i));
      }
    }
    auto q0ret_robot = P.query(q0_robot, robot);
    auto qTret_robot = P.query(qT_robot, robot);

    if(!q0ret_robot->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret_robot->writeDetails(std::cout, P); }
    if(!qTret_robot->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret_robot->writeDetails(std::cout, P); }
    
    cout << "Initializing RRT for robot: " << robot << endl;
    rrtRobots0[robot] = make_shared<MR_RRT_SingleTree>(q0_robot, q0ret_robot);
    rrtRobotsT[robot] = make_shared<MR_RRT_SingleTree>(qT_robot, qTret_robot);

    isFinished[robot] = false;
  }

  P.C.setJointState(q0);

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void MR_RRT_PathFinder::planForward(const arr& q0, const arr& qT) {
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    if(success) break;

    //some output
    if(verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int MR_RRT_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)maxIters) return -1;

  //bool success = growTreeToTree(*rrt0, *rrtT, true);
  //if(!success) success = growTreeToTree(*rrtT, *rrt0, false);

  bool success = growTreeToTree(*rrt0, *rrtT, rrtRobots0, rrtRobotsT, true);
  if(!success) success = growTreeToTree(*rrtT, *rrt0, rrtRobotsT, rrtRobots0, false);

  //animation display
  if(verbose>2) {
    if(!(iters%100)) {
      DISP.setJointState(rrt0->getLast());
      DISP.view(verbose>4, STRING("planConnect evals " <<P.evals));
    }
  }
  if(verbose>1) {
    if(!(iters%100)) {
      std::cout <<"RRT queries=" <<P.evals <<" tree sizes = " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
    }
  }

  //-- the rest is only on success -> extract the path, display, etc

  if(success) {
    if(verbose>0) {
      std::cout <<"  -- rrt success:";
      std::cout <<" queries:" <<P.evals <<" tree sizes: " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
//      std::cout <<"  forwardSteps: " <<(100.*n_forwardStepGood/n_forwardStep) <<"%/" <<n_forwardStep;
//      std::cout <<"  backSteps: " <<(100.*n_backStepGood/n_backStep) <<"%/" <<n_backStep;
//      std::cout <<"  rndSteps: " <<(100.*n_rndStepGood/n_rndStep) <<"%/" <<n_rndStep;
//      std::cout <<"  sideSteps: " <<(100.*n_sideStepGood/n_sideStep) <<"%/" <<n_sideStep;
//      std::cout <<std::endl;
    }

    
    cout << "Constructing final path..." << endl;
    // Get all robots paths and combine
    int maxPathLength = 0;
    
    for (const auto& [robot, path_r] : rrtPaths) {
      if (path_r.d0 > maxPathLength) {
        maxPathLength = path_r.d0;
      }
    }
    
    /**/
    cout << "Max path length across robots: " << maxPathLength << endl;
    
    for (uint j = 0; j < (uint)maxPathLength; ++j){
      arr qj;
      qj.resize(q0.N);
      qj.setZero();

      // fill global joint vector from each robot's robot-space path
      for (const auto& [robot_id, jointMask] : robots) {
        auto itP = rrtPaths.find(robot_id);
        CHECK(itP != rrtPaths.end(), "Missing path for robot_id " << robot_id);

        const arr& pr = itP->second;              // shape: Tr x dim_robot
        uint Tr = pr.d0;
        uint dim_robot = pr.d1;

        uint row = (j < Tr ? j : Tr - 1);         // repeat last if shorter

        // local column counter in robot-space
        uint k = 0;
        for (uint i = 0; i < jointMask.N; ++i) {
          if (jointMask(i) == 1) {
            CHECK(k < dim_robot, "robot-space dim mismatch for robot_id " << robot_id);
            qj(i) = pr(row, k);
            ++k;
          }
        }
        CHECK(k == dim_robot, "robot-space path dim doesn't match jointMask for robot_id " << robot_id);
      }

      path.append(qj);
    }

    

    path.reshape(-1, q0.N);
    //revertPath(path);
    
    path = rrt0->getPathFromNode(rrt0->nearestID);
    //cout << "ADSKJHASDKJHJKASD" << endl;
    //arr pathT = rrtT->getPathFromNode(rrtT->nearestID);
    revertPath(path);
    path.append(qT);
    cout << "Path constructed with length: " << path.d0 << endl;

    //display
    if(verbose>1) {
      std::cout <<"  path-length=" <<path.d0 <<std::endl;
      if(verbose>2) {
        DISP.proxies.clear();
        for(uint t=0; t<path.d0; t++) {
          DISP.setJointState(path[t]);
          DISP.view(false, STRING("rrt result "<<t));
          rai::wait(.1);
        }
        DISP.view(verbose>3);
        DISP.clear();
      }
    }

    return 1;
  }

  return 0;
}

arr MR_RRT_PathFinder::planConnect() {
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==-1) path.clear();
  return path;
}

arr MR_RRT_PathFinder::run(double timeBudget) {
  planConnect();
  return path;
}

namespace rai {

void MR_PathFinder::setProblem(const Configuration& C, const arr& starts, const arr& goals, const std::map<int, arr>& robots, double collisionTolerance, bool isIndependent) {
  if(collisionTolerance<0.) collisionTolerance = rai::getParameter<double>("rrt/collisionTolerance", 1e-4);
  problem = make_shared<ConfigurationProblem>(C, true, collisionTolerance, 1);
  problem->verbose=0;
  rrtSolver = make_shared<MR_RRT_PathFinder>(*problem, starts, goals, robots);


  cout <<"RRT PathFinder: stepsize=" <<rrtSolver->stepsize
       <<", subsampleChecks=" <<rrtSolver->subsampleChecks
       <<", maxIters=" <<rrtSolver->maxIters
       <<", collisionTolerance=" <<collisionTolerance
       <<endl;
}

void MR_PathFinder::setExplicitCollisionPairs(const StringA& collisionPairs) {
  CHECK(problem, "need to set problem first");
  problem->setExplicitCollisionPairs(collisionPairs);
}

shared_ptr<SolverReturn> MR_PathFinder::solve() {
  if(!ret) ret = make_shared<SolverReturn>();

  ret->time -= rai::cpuTime();
  rrtSolver->run();
  ret->time += rai::cpuTime();

  ret->done = true; //(r!=0);
  ret->feasible = rrtSolver->path.N; //(r==1);
  ret->x = rrtSolver->path;
  ret->evals = rrtSolver->iters;

  return ret;
}

arr MR_PathFinder::get_resampledPath(uint T){ return path_resampleLinear(ret->x, T); }

} //namespace