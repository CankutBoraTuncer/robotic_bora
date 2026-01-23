/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Kin/proxy.h"
#include "../Kin/feature.h"
#include "../Optim/constrained.h"
#include "../Geo/fclInterface.h"

#include "ConfigurationProblem.h"

ConfigurationProblem::ConfigurationProblem(const rai::Configuration& _C, bool _computeCollisions, double _collisionTolerance, int _verbose)
  : C(_C),
    computeAllCollisions(_computeCollisions),
    collisionTolerance(_collisionTolerance),
    verbose(_verbose) {

  q0 = C.getJointState();
  limits = C.getJointLimits();
  max_step = zeros(limits.d1);

  for(rai::Dof* dof: C.activeDofs) {
    uint i=dof->qIndex;
    uint d=dof->dim;
    if(d) {
      for(uint k=0; k<d; k++) max_step(i+k) = 1.;
    }
  }

  computeCollisionFeatures = false;
  if(!computeCollisionFeatures) {
    C.fcl(verbose-1)->mode = rai::FclInterface::_binaryCollisionAll;
  }
}

shared_ptr<GroundedObjective> ConfigurationProblem::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target) {
  shared_ptr<Feature> f = symbols2feature(feat, frames, C, scale, target, 0);

  shared_ptr<GroundedObjective> ob = make_shared<GroundedObjective>(f, type, intA{});
  ob->frames = C.getFrames(f->frameIDs);

  objectives.append(ob);
  return ob;
}

void ConfigurationProblem::setExplicitCollisionPairs(const StringA& _collisionPairs) {
  computeAllCollisions = false;
  collisionPairs = C.getFrameIDs(_collisionPairs);
  collisionPairs.reshape(-1, 2);
}

shared_ptr<QueryResult> ConfigurationProblem::query(const arr& x) {
  if(limits.N) {
    for(uint i=0; i<x.N; i++) {
      if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
      }
    }
  }

  C.setJointState(x);
  
  if(computeAllCollisions) {
    //C.stepSwift();
    C.stepFcl();
    //for(rai::Proxy& p:C.proxies) p.ensure_coll();
  } else if(collisionPairs.N) {
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();

  if(!computeCollisionFeatures) {
#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif
  } else {
    //collision features
    uint N = C.proxies.N;
    qr->collisions.resize(N, 2).setZero();
    qr->coll_y.resize(N, 1).setZero();
    qr->coll_J.resize(N, 1, x.N).setZero();
    qr->normal_y.resize(N, 3).setZero();
    qr->normal_J.resize(N, 3, x.N).setZero();
    qr->side_J.resize(N, 3, x.N).setZero();

    uint i=0;
    for(const rai::Proxy& p:C.proxies) {
      qr->collisions[i] =  uintA{p.a->ID, p.b->ID};
      arr Jp1, Jp2, Jx1, Jx2;
      {
        C.jacobian_pos(Jp1, C(p.a->ID), p.collision->p1);
        C.jacobian_pos(Jp2, C(p.b->ID), p.collision->p2);
        C.jacobian_angular(Jx1, C(p.a->ID));
        C.jacobian_angular(Jx2, C(p.b->ID));
      }
      p.collision->kinDistance(qr->coll_y[i].noconst(), qr->coll_J[i].noconst(), Jp1, Jp2);
      p.collision->kinNormal(qr->normal_y[i].noconst(), qr->normal_J[i].noconst(), Jp1, Jp2, Jx1, Jx2);

      arr a, b, Ja, Jb;
      {
        C.kinematicsPos(a, Ja, C(p.a->ID));
        C.kinematicsPos(b, Jb, C(p.b->ID));
      }
#if 0
      arr z = a-b;
      z /= length(z);
#else
      arr z = qr->normal_y[i];
#endif
      qr->side_J[i] = (eye(3) - (z^z))* (Ja - Jb);

      i++;
    }
    CHECK_EQ(i, N, "");
    qr->coll_J.reshape(qr->coll_y.N, x.N);

    //is feasible?
    qr->isFeasible = (!qr->coll_y.N || min(qr->coll_y)>=-collisionTolerance);

    //goal features
    N=0;
    for(shared_ptr<GroundedObjective>& ob : objectives) N += ob->feat->dim(ob->frames);
    qr->goal_y.resize(N);
    qr->goal_J.resize(N, x.N);

    i=0;
    //  arr z, Jz;
    for(shared_ptr<GroundedObjective>& ob : objectives) {
      arr z = ob->feat->eval(ob->frames);
      for(uint j=0; j<z.N; j++) {
        qr->goal_y(i+j) = z(j);
        qr->goal_J[i+j] = z.J()[j];
      }
      i += z.N;
    }
    CHECK_EQ(i, N, "");

    //is goal?
    qr->isGoal= (absMax(qr->goal_y)<1e-2);
  }

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}


// ---------------------------------------------------------------------------

shared_ptr<QueryResult> ConfigurationProblem::query(const arr& x, const int robot_id) {
  if(limits.N) {
    for(uint i=0; i<x.N; i++) {
      if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
      }
    }
  }

  C.frames.elem(robot_id)->setJointState(x);

  if(computeAllCollisions) {
    //C.stepSwift();
    C.stepFcl();
    //for(rai::Proxy& p:C.proxies) p.ensure_coll();
  } else if(collisionPairs.N) {
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();

  if(!computeCollisionFeatures) {
#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif
  } else {
    //collision features
    uint N = C.proxies.N;
    qr->collisions.resize(N, 2).setZero();
    qr->coll_y.resize(N, 1).setZero();
    qr->coll_J.resize(N, 1, x.N).setZero();
    qr->normal_y.resize(N, 3).setZero();
    qr->normal_J.resize(N, 3, x.N).setZero();
    qr->side_J.resize(N, 3, x.N).setZero();

    uint i=0;
    for(const rai::Proxy& p:C.proxies) {
      qr->collisions[i] =  uintA{p.a->ID, p.b->ID};
      arr Jp1, Jp2, Jx1, Jx2;
      {
        C.jacobian_pos(Jp1, C(p.a->ID), p.collision->p1);
        C.jacobian_pos(Jp2, C(p.b->ID), p.collision->p2);
        C.jacobian_angular(Jx1, C(p.a->ID));
        C.jacobian_angular(Jx2, C(p.b->ID));
      }
      p.collision->kinDistance(qr->coll_y[i].noconst(), qr->coll_J[i].noconst(), Jp1, Jp2);
      p.collision->kinNormal(qr->normal_y[i].noconst(), qr->normal_J[i].noconst(), Jp1, Jp2, Jx1, Jx2);

      arr a, b, Ja, Jb;
      {
        C.kinematicsPos(a, Ja, C(p.a->ID));
        C.kinematicsPos(b, Jb, C(p.b->ID));
      }
#if 0
      arr z = a-b;
      z /= length(z);
#else
      arr z = qr->normal_y[i];
#endif
      qr->side_J[i] = (eye(3) - (z^z))* (Ja - Jb);

      i++;
    }
    CHECK_EQ(i, N, "");
    qr->coll_J.reshape(qr->coll_y.N, x.N);

    //is feasible?
    qr->isFeasible = (!qr->coll_y.N || min(qr->coll_y)>=-collisionTolerance);

    //goal features
    N=0;
    for(shared_ptr<GroundedObjective>& ob : objectives) N += ob->feat->dim(ob->frames);
    qr->goal_y.resize(N);
    qr->goal_J.resize(N, x.N);

    i=0;
    //  arr z, Jz;
    for(shared_ptr<GroundedObjective>& ob : objectives) {
      arr z = ob->feat->eval(ob->frames);
      for(uint j=0; j<z.N; j++) {
        qr->goal_y(i+j) = z(j);
        qr->goal_J[i+j] = z.J()[j];
      }
      i += z.N;
    }
    CHECK_EQ(i, N, "");

    //is goal?
    qr->isGoal= (absMax(qr->goal_y)<1e-2);
  }

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}

// ---------------------------------------------------------------------------

shared_ptr<QueryResult> ConfigurationProblem::query(const arr& x, const std::map<rai::String, arr>& frame, uint depth) {
  if(limits.N) {
    for(uint i=0; i<x.N; i++) {
      if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
      }
    }
  }


  arr lastValue;
  for(auto& [key, value] : frame) {
    //cout << key << ", " << value[depth] << " depth: " << depth <<  endl;
    if (depth < value.N) {

      arr pos = {value[depth](0), value[depth](1), 0.2};
      pos(2) = 0.2;
      C.getFrame(key)->setPosition(pos);
      lastValue = value;
    }
  }
  
  C.setJointState(x);
  //C.view(true, STRING("Set Pos: " <<x << " Other robot pos: " << lastValue[depth]).p);

  if(computeAllCollisions) {
    //C.stepSwift();
    C.stepFcl();
    //for(rai::Proxy& p:C.proxies) p.ensure_coll();
  } else if(collisionPairs.N) {
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();

  if(!computeCollisionFeatures) {
#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif
  } else {
    //collision features
    uint N = C.proxies.N;
    qr->collisions.resize(N, 2).setZero();
    qr->coll_y.resize(N, 1).setZero();
    qr->coll_J.resize(N, 1, x.N).setZero();
    qr->normal_y.resize(N, 3).setZero();
    qr->normal_J.resize(N, 3, x.N).setZero();
    qr->side_J.resize(N, 3, x.N).setZero();

    uint i=0;
    for(const rai::Proxy& p:C.proxies) {
      qr->collisions[i] =  uintA{p.a->ID, p.b->ID};
      arr Jp1, Jp2, Jx1, Jx2;
      {
        C.jacobian_pos(Jp1, C(p.a->ID), p.collision->p1);
        C.jacobian_pos(Jp2, C(p.b->ID), p.collision->p2);
        C.jacobian_angular(Jx1, C(p.a->ID));
        C.jacobian_angular(Jx2, C(p.b->ID));
      }
      p.collision->kinDistance(qr->coll_y[i].noconst(), qr->coll_J[i].noconst(), Jp1, Jp2);
      p.collision->kinNormal(qr->normal_y[i].noconst(), qr->normal_J[i].noconst(), Jp1, Jp2, Jx1, Jx2);

      arr a, b, Ja, Jb;
      {
        C.kinematicsPos(a, Ja, C(p.a->ID));
        C.kinematicsPos(b, Jb, C(p.b->ID));
      }
#if 0
      arr z = a-b;
      z /= length(z);
#else
      arr z = qr->normal_y[i];
#endif
      qr->side_J[i] = (eye(3) - (z^z))* (Ja - Jb);

      i++;
    }
    CHECK_EQ(i, N, "");
    qr->coll_J.reshape(qr->coll_y.N, x.N);

    //is feasible?
    qr->isFeasible = (!qr->coll_y.N || min(qr->coll_y)>=-collisionTolerance);

    //goal features
    N=0;
    for(shared_ptr<GroundedObjective>& ob : objectives) N += ob->feat->dim(ob->frames);
    qr->goal_y.resize(N);
    qr->goal_J.resize(N, x.N);

    i=0;
    //  arr z, Jz;
    for(shared_ptr<GroundedObjective>& ob : objectives) {
      arr z = ob->feat->eval(ob->frames);
      for(uint j=0; j<z.N; j++) {
        qr->goal_y(i+j) = z(j);
        qr->goal_J[i+j] = z.J()[j];
      }
      i += z.N;
    }
    CHECK_EQ(i, N, "");

    //is goal?
    qr->isGoal= (absMax(qr->goal_y)<1e-2);
  }

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}

// ---------------------------------------------------------------------------

shared_ptr<QueryResult> ConfigurationProblem::query(arr& x, const std::map<int, arr>& robots, const arr& qT, const double stepSize, const int subsampleChecks, bool isForwardStep) {
  if(limits.N) {
    for(uint i=0; i<x.N; i++) {
      if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
      }
    }
  }

  //cout << "Query with stepSize: " << stepSize << ", subsampleChecks: " << subsampleChecks << ", isForwardStep: " << isForwardStep <<  endl;
  
    arr curState = C.getJointState();
    //cout << "Set joint states: " << x <<  endl;
    //cout << "Target joint states: " << qT <<  endl;
    

    //C.view(true, "before setJointState");

    arr nx;
    // For each robot check if their joint state matches the target at the end
    for (const auto& [robot, jointMask] : robots) {

      arr js;
      arr rx;
      arr cx;
      for (uint i=0; i<jointMask.N; i++) {
        if (jointMask(i) == 1){
          js.append(qT(i));
          rx.append(x(i));
          cx.append(curState(i));
        }
      }

      // cout << "JS: " << js << ", RX: " << rx << ", CX: " << cx <<  endl;

      arr delta = js - rx;
      arr curDelta = js - cx;
      
      double curDist = length(curDelta);
      double dist = length(delta);
      
      arr t_j;
      // cout << "Robot: " << robot << " Current vs Target: " << curDist << ", Set vs Target: " << dist << " Limit " << stepSize << endl;
      
      double threshold =  subsampleChecks > 0 ? (stepSize / subsampleChecks): stepSize;
      if (curDist < threshold) {nx.append(cx); t_j = cx;}
      else {nx.append(rx); t_j = rx;}     
   
      // cout << "Current State for robot: " << robot << " " << C.frames.elem(robot)->getJointState() << " Set State for Robot" << t_j <<  endl;
      // cout << "-------------------------" <<  endl;
      C.frames.elem(robot)->setJointState(t_j);
      int idx = 0;
      for (uint i=0; i<jointMask.N; i++) {
        if (jointMask(i) == 1){
          x(i) = t_j(idx);
          idx++;
        }
      }


    }
 
    // cout << "New joint states: " << x <<  endl;

  
  //C.view(true, "before setJointState");
  //C.setJointState(x);
  //C.view(true, "after setJointState");
  //C.view(true, STRING("Set Pos: " <<x << " Other robot pos: " << lastValue[depth]).p);


  if(computeAllCollisions) {
    //C.stepSwift();
    C.stepFcl();
    //for(rai::Proxy& p:C.proxies) p.ensure_coll();
  } else if(collisionPairs.N) {
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();
  
  if(!computeCollisionFeatures) {
#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif
  } else {
    //collision features
    uint N = C.proxies.N;
    qr->collisions.resize(N, 2).setZero();
    qr->coll_y.resize(N, 1).setZero();
    qr->coll_J.resize(N, 1, x.N).setZero();
    qr->normal_y.resize(N, 3).setZero();
    qr->normal_J.resize(N, 3, x.N).setZero();
    qr->side_J.resize(N, 3, x.N).setZero();

    uint i=0;
    for(const rai::Proxy& p:C.proxies) {
      qr->collisions[i] =  uintA{p.a->ID, p.b->ID};
      arr Jp1, Jp2, Jx1, Jx2;
      {
        C.jacobian_pos(Jp1, C(p.a->ID), p.collision->p1);
        C.jacobian_pos(Jp2, C(p.b->ID), p.collision->p2);
        C.jacobian_angular(Jx1, C(p.a->ID));
        C.jacobian_angular(Jx2, C(p.b->ID));
      }
      p.collision->kinDistance(qr->coll_y[i].noconst(), qr->coll_J[i].noconst(), Jp1, Jp2);
      p.collision->kinNormal(qr->normal_y[i].noconst(), qr->normal_J[i].noconst(), Jp1, Jp2, Jx1, Jx2);

      arr a, b, Ja, Jb;
      {
        C.kinematicsPos(a, Ja, C(p.a->ID));
        C.kinematicsPos(b, Jb, C(p.b->ID));
      }
#if 0
      arr z = a-b;
      z /= length(z);
#else
      arr z = qr->normal_y[i];
#endif
      qr->side_J[i] = (eye(3) - (z^z))* (Ja - Jb);
      
      // cout << "Collision between: " << p.a->name << " and " << p.b->name << " Distance: " << qr->coll_y[i](0) << endl;
      // Track per-robot collisions: identify which robots are involved in this collision
      for(const auto& [robot_id, jointMask] : robots) {
        rai::Frame* robot_frame = C.frames.elem(robot_id);
        if(!robot_frame) continue;
        
        // Check if either frame in collision involves this robot or its parts
        bool involves_robot = false;
        if(p.a->ID == robot_frame->ID || p.b->ID == robot_frame->ID) involves_robot = true;
        // cout << "INVOLVES ROBOT: " << robot_id << " " << involves_robot << endl;
        if(involves_robot) {
          // Add collision distance and Jacobian for this robot
          // If there are multiple collisions for the same robot, the minimum distance is kept
          if(qr->coll_y_robots.find(robot_id) == qr->coll_y_robots.end()) {
            qr->coll_y_robots[robot_id] = qr->coll_y[i].copy();
            qr->coll_J_robots[robot_id] = qr->coll_J[i].copy();
          } else {
            // Keep the collision with minimum distance
            if(qr->coll_y[i](0) < qr->coll_y_robots[robot_id](0)) {
              qr->coll_y_robots[robot_id] = qr->coll_y[i].copy();
              qr->coll_J_robots[robot_id] = qr->coll_J[i].copy();
            }
          }
        }
      }

      i++;
    }
    CHECK_EQ(i, N, "");
    qr->coll_J.reshape(qr->coll_y.N, x.N);

    //is feasible?
    qr->isFeasible = (!qr->coll_y.N || min(qr->coll_y)>=-collisionTolerance);

    //goal features
    N=0;
    for(shared_ptr<GroundedObjective>& ob : objectives) N += ob->feat->dim(ob->frames);
    qr->goal_y.resize(N);
    qr->goal_J.resize(N, x.N);

    i=0;
    //  arr z, Jz;
    for(shared_ptr<GroundedObjective>& ob : objectives) {
      arr z = ob->feat->eval(ob->frames);
      for(uint j=0; j<z.N; j++) {
        qr->goal_y(i+j) = z(j);
        qr->goal_J[i+j] = z.J()[j];
      }
      i += z.N;
    }
    CHECK_EQ(i, N, "");

    //is goal?
    qr->isGoal= (absMax(qr->goal_y)<1e-2);
  }

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}

// ---------------------------------------------------------------------------
shared_ptr<QueryResult> ConfigurationProblem::query(arr& x, const std::map<int, arr>& robots, const std::map<int, bool>& isFinished) {
  if(limits.N) {
    for(uint i=0; i<x.N; i++) {
      if(limits(1, i)>limits(0, i) && (x.elem(i)<limits(0, i) || x.elem(i)>limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<x.elem(i) <<' ' <<limits[i];
      }
    }
  }
    arr js = C.getJointState();
    // For each robot check if their joint state matches the target at the end
    for (const auto& [robot, jointMask] : robots) {

      if(isFinished.at(robot)) {
        // cout << "Robot: " << robot << " is finished, keeping current joint state." << endl;
        continue;
      }
      arr rx;
      for (uint i=0; i<jointMask.N; i++) {
        if (jointMask(i) == 1){
          rx.append(x(i));
          js(i) = x(i);
        }
      }

      C.frames.elem(robot)->setJointState(rx);
    }
    x = js;

    

    // cout << "New joint states: " << x <<  endl;

  if(computeAllCollisions) {
    //C.stepSwift();
    C.stepFcl();
    //for(rai::Proxy& p:C.proxies) p.ensure_coll();
  } else if(collisionPairs.N) {
    C.proxies.resize(collisionPairs.d0);
    for(uint i=0; i<collisionPairs.d0; i++) {
      C.proxies(i).a = C.frames(collisionPairs(i, 0));
      C.proxies(i).b = C.frames(collisionPairs(i, 1));
      C.proxies(i).d = -0.;
    }
    for(rai::Proxy& p:C.proxies) p.calc_coll();
    C._state_proxies_isGood = true;
  }
  evals++;

  //C.view();

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();
  
  if(!computeCollisionFeatures) {
#if 1
    double D=0.;
    for(rai::Proxy& p:C.proxies){
      p.calc_coll();
      if(p.d<0.) D -= p.d;
    }
    qr->totalCollision = D;
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#else
    qr->totalCollision = C.getTotalPenetration();
    qr->isFeasible = (qr->totalCollision<collisionTolerance);
#endif
  } else {
    //collision features
    uint N = C.proxies.N;
    qr->collisions.resize(N, 2).setZero();
    qr->coll_y.resize(N, 1).setZero();
    qr->coll_J.resize(N, 1, x.N).setZero();
    qr->normal_y.resize(N, 3).setZero();
    qr->normal_J.resize(N, 3, x.N).setZero();
    qr->side_J.resize(N, 3, x.N).setZero();

    uint i=0;
    for(const rai::Proxy& p:C.proxies) {
      qr->collisions[i] =  uintA{p.a->ID, p.b->ID};
      arr Jp1, Jp2, Jx1, Jx2;
      {
        C.jacobian_pos(Jp1, C(p.a->ID), p.collision->p1);
        C.jacobian_pos(Jp2, C(p.b->ID), p.collision->p2);
        C.jacobian_angular(Jx1, C(p.a->ID));
        C.jacobian_angular(Jx2, C(p.b->ID));
      }
      p.collision->kinDistance(qr->coll_y[i].noconst(), qr->coll_J[i].noconst(), Jp1, Jp2);
      p.collision->kinNormal(qr->normal_y[i].noconst(), qr->normal_J[i].noconst(), Jp1, Jp2, Jx1, Jx2);

      arr a, b, Ja, Jb;
      {
        C.kinematicsPos(a, Ja, C(p.a->ID));
        C.kinematicsPos(b, Jb, C(p.b->ID));
      }
#if 0
      arr z = a-b;
      z /= length(z);
#else
      arr z = qr->normal_y[i];
#endif
      qr->side_J[i] = (eye(3) - (z^z))* (Ja - Jb);
      
      // cout << "Collision between: " << p.a->name << " and " << p.b->name << " Distance: " << qr->coll_y[i](0) << endl;
      // Track per-robot collisions: identify which robots are involved in this collision
      for(const auto& [robot_id, jointMask] : robots) {
        rai::Frame* robot_frame = C.frames.elem(robot_id);
        if(!robot_frame) continue;
        
        // Check if either frame in collision involves this robot or its parts
        bool involves_robot = false;
        if(p.a->ID == robot_frame->ID || p.b->ID == robot_frame->ID) involves_robot = true;
        // cout << "INVOLVES ROBOT: " << robot_id << " " << involves_robot << endl;
        if(involves_robot) {
          // Add collision distance and Jacobian for this robot
          // If there are multiple collisions for the same robot, the minimum distance is kept
          if(qr->coll_y_robots.find(robot_id) == qr->coll_y_robots.end()) {
            qr->coll_y_robots[robot_id] = qr->coll_y[i].copy();
            qr->coll_J_robots[robot_id] = qr->coll_J[i].copy();
          } else {
            // Keep the collision with minimum distance
            if(qr->coll_y[i](0) < qr->coll_y_robots[robot_id](0)) {
              qr->coll_y_robots[robot_id] = qr->coll_y[i].copy();
              qr->coll_J_robots[robot_id] = qr->coll_J[i].copy();
            }
          }
        }
      }

      i++;
    }
    CHECK_EQ(i, N, "");
    qr->coll_J.reshape(qr->coll_y.N, x.N);

    //is feasible?
    qr->isFeasible = (!qr->coll_y.N || min(qr->coll_y)>=-collisionTolerance);

    //goal features
    N=0;
    for(shared_ptr<GroundedObjective>& ob : objectives) N += ob->feat->dim(ob->frames);
    qr->goal_y.resize(N);
    qr->goal_J.resize(N, x.N);

    i=0;
    //  arr z, Jz;
    for(shared_ptr<GroundedObjective>& ob : objectives) {
      arr z = ob->feat->eval(ob->frames);
      for(uint j=0; j<z.N; j++) {
        qr->goal_y(i+j) = z(j);
        qr->goal_J[i+j] = z.J()[j];
      }
      i += z.N;
    }
    CHECK_EQ(i, N, "");

    //is goal?
    qr->isGoal= (absMax(qr->goal_y)<1e-2);
  }

  //display (link of last joint)
  qr->disp3d = C.activeDofs.elem(-1)->frame->getPosition();
  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }

  return qr;
}

// ---------------------------------------------------------------------------
void QueryResult::getViolatedContacts(arr& y, arr& J, double margin) {
  uintA violated;
  for(uint i=0; i<coll_y.N; i++) if(coll_y.elem(i)<margin) violated.append(i);

  if(!violated.N) {
    y.resize(0);
    J.resize(0, coll_J.d1);
  } else {
    y = coll_y.sub(violated);
    J = coll_J.sub(violated);
  }

}

arr QueryResult::getSideStep() {
  arr s = randn(3);
  s /=length(s);

  arr S(side_J.d0, 3);
  for(uint i=0; i<S.d0; i++) S[i] = s;

  arr J = side_J;

  S.reshape(-1);
  J.reshape(S.N, -1);

#if 0
  arr U, sig, V;
  svd(U, sig, V, J);
  arr d = ~V * sig % V * randn(V.d1); //random step in input space of J!
#else
  arr JI = ~J; //pseudoInverse(J);
  arr d = JI * S;
#endif

  if(length(d)<1e-10) HALT("???");

  return d;
}

arr QueryResult::getForwardStep() {
  arr goal_JI = pseudoInverse(goal_J);
  arr d = goal_JI * (-goal_y);
  return d;
}

arr QueryResult::getBackwardStep(double relativeStepLength, double margin, const arr& nullStep) {
//  CHECK(!isFeasible, "");
  CHECK(coll_y.N>0, "");

  arr y, J;
  getViolatedContacts(y, J, margin);
  y -= margin;

  arr Jinv = pseudoInverse(J, NoArr, 1e-4);
  arr d = Jinv * (-relativeStepLength * y);

  if(!!nullStep) d += (eye(J.d1) - Jinv * J) * nullStep;

  return d;
}

void QueryResult::write(std::ostream& os) const {
  os <<"query: h_goal: " <<sumOfAbs(goal_y)
     <<" g_coll: " <<sum(elemWiseHinge(-coll_y))
     <<" isGoal: " <<isGoal
     <<" isFeasible: " <<isFeasible;
}

void QueryResult::writeDetails(std::ostream& os, const ConfigurationProblem& P, double margin) const {
  write(os);
  if(!P.computeCollisionFeatures) {
    for(const rai::Proxy& p:P.C.proxies) if(p.d<=0.) {
        os <<"\nproxy: " <<p;
      }
  } else {
    for(uint i=0; i<coll_y.N; i++) {
      if(coll_y.elem(i)<margin) {
        os <<"\ncoll " <<i <<':' <<collisions[i]
           <<':' <<P.C.frames(collisions(i, 0))->name <<'-' <<P.C.frames(collisions(i, 1))->name
           <<" y:" <<coll_y.elem(i) <<" normal:" <<normal_y[i];
      }
    }
  }
  os <<std::endl;
}

bool makePoseFeasible(arr& x, ConfigurationProblem& P, double IKstepSize, double maxQStepSize, uint trials) {
  shared_ptr<QueryResult> qr = P.query(x);
  for(uint k=0; k<trials; k++) {
    if(qr->isFeasible) {
      break;
    } else {
    }
    arr delta = qr->getBackwardStep(IKstepSize);
    double l = length(delta);
    if(maxQStepSize>0. && l>maxQStepSize) delta *= maxQStepSize/l;
    x += delta;
    qr = P.query(x);
  }
  return qr->isFeasible;
}
