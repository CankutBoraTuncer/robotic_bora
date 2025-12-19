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

#include "MR_ConfigurationProblem.h"

ConfigurationProblem::ConfigurationProblem(const rai::Configuration& _C, const StringA& _robotNames, bool _computeCollisions, double _collisionTolerance, int _verbose)
  : C(_C),
    robotNames(_robotNames),
    computeAllCollisions(_computeCollisions),
    collisionTolerance(_collisionTolerance),
    verbose(_verbose)
    {

  rai::Array<Robot> robots;
  for (const rai::String& rname : robotNames) {
    rai::Frame* robotFrame = C.getFrame(rname);
    Robot r = Robot(rname, robotFrame->getJointState(), robotFrame->joint->limits);
    robots.append(r);
  }

  q0 = C.getJointState();
  
  computeCollisionFeatures = false;
  if(!computeCollisionFeatures) {
    C.fcl(verbose-1)->mode = rai::FclInterface::_binaryCollisionAll;
  }
}

// ============================================================================== //

shared_ptr<QueryResult> ConfigurationProblem::query(const Robot& r) {
  if(r.limits.N) {
    for(uint i=0; i<r.x.N; i++) {
      if(r.limits(1, i)>r.limits(0, i) && (r.x.elem(i)<r.limits(0, i) || r.x.elem(i)>r.limits(1, i))) {
        //LOG(-1) <<"QUERY OUT OF LIMIT: joint " <<i <<": " <<r.x.elem(i) <<' ' <<r.limits[i];
      }
    }
  }
  C.getFrame(r.n)->setJointState(r.x);
  C.stepFcl();

  evals++;

  shared_ptr<QueryResult> qr = make_shared<QueryResult>();

  double D=0.;
  for(rai::Proxy& p:C.proxies){
    p.calc_coll();
    if(p.d<0.) D -= p.d;
  }
  qr->totalCollision = D;
  qr->isFeasible = (qr->totalCollision<collisionTolerance);

  if(verbose) {
    C.view(verbose>1, STRING("ConfigurationProblem query:\n" <<*qr));
  }
  return qr;
}

// ============================================================================== //

rai::Array<shared_ptr<QueryResult>> ConfigurationProblem::query_multi(rai::Array<Robot>& x) {
  
  rai::Array<shared_ptr<QueryResult>> results;
  for(Robot& r : x) {
    shared_ptr<QueryResult> qr = query(r);
    results.append(qr);
  }
  return results;
}
  
// ============================================================================== //

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

// ============================================================================== //

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

// ============================================================================== //

arr QueryResult::getForwardStep() {
  arr goal_JI = pseudoInverse(goal_J);
  arr d = goal_JI * (-goal_y);
  return d;
}

// ============================================================================== //

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

// ============================================================================== //

void QueryResult::write(std::ostream& os) const {
  os <<"query: h_goal: " <<sumOfAbs(goal_y)
     <<" g_coll: " <<sum(elemWiseHinge(-coll_y))
     <<" isGoal: " <<isGoal
     <<" isFeasible: " <<isFeasible;
}

// ============================================================================== //

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

// ============================================================================== //


