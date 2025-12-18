/* ------------------------------------------------------------------
    MCR Path Planner Implementation
    --------------------------------------------------------------  */

#include "MCR_PathFinder.h"
#include <queue>
#include <set>
#include <algorithm>
#include <map>


namespace rai {

// ==============================================================================
//   MCR_PathFinder Methods
// ==============================================================================


MCR_PathFinder::MCR_PathFinder(rai::Configuration& _C, const rai::String _agent, const rai::String _goalFrame, const StringA& _obsList, bool _persist, bool _earlyExit, double _penalty, int _verbose)
  : agent(_agent), verbose(_verbose), obsList(_obsList), penalty(_penalty), C(_C), persist(_persist), earlyExit(_earlyExit) {

    rai::Configuration C(_C);
    
    startPos = C.getFrame(_agent)->getPosition().sub(0,1);
    goalPos = C.getFrame(_goalFrame)->getPosition().sub(0,1);
     
    // Cache Frame IDs for soft obstacles
    for(const rai::String& name : obsList){
        rai::Frame* f = C.getFrame(name);
        if(f) softObsIndices.append(f->ID);
        else if(verbose > 0) std::cout << "MCR Warning: Obstacle '" << name << "' not found." << std::endl;
    }

}

// ============================================================================== //

void MCR_PathFinder::checkCollisions(const arr& q, bool& isHard, uintA& softHits){
    isHard = false;
    softHits.clear();
    C.setJointState(q);
   
    C.stepFcl();
    // Compute collisions
    uint targetID = C.getFrame(agent)->ID;
    for(const rai::Proxy& p : C.proxies) {
      if(p.a->ID == targetID || p.b->ID == targetID) {
        double dist = p.d;
        uint obsId = (p.a->ID == targetID) ? p.b->ID : p.a->ID;
        bool isSoft = softObsIndices.contains(obsId);
        if(isSoft)softHits.setAppend(obsId);
        else {
          isHard = true;
          return;
        }
      }
    }

}

// ============================================================================== //

double MCR_PathFinder::getEdgeData(const arr& from, const arr& to, uintA& constraints, double stepSize){
    double d = length(from - to);
    int steps = (int)(d / stepSize) + 1;
    
    constraints.clear();
    
    // Use binary sampling like checkConnection for better coverage
    for(int i=1; i<steps; i++){
        double ind = corput(i, 2);  // Use Corput sequence for better sampling
        arr interp = from + ind * (to - from);
        
        bool isHard;
        uintA stepHits;
        checkCollisions(interp, isHard, stepHits);
        
        if(isHard) return -1.0; // Invalid edge - hard collision
        
        // Accumulate soft obstacle hits
        for(uint id : stepHits) constraints.setAppend(id);
    }
    
    // Also check the endpoint (similar to checkConnection's loop structure)
    bool isHard;
    uintA stepHits;
    checkCollisions(to, isHard, stepHits);
    if(isHard) return -1.0;
    for(uint id : stepHits) constraints.setAppend(id);

    // Cost Function: Distance + (Penalty * Count)
    return d + (double)constraints.N * penalty;
}

// ============================================================================== //

StringA MCR_PathFinder::solve(int maxIters, double stepSize, double connRadius){
    if (!persist) {
        // Clean up previous run
        for(auto* n : nodes) delete n;
        nodes.clear();
    }

    // Init Start/Goal
    startNode = new MCR_Node(startPos);
    goalNode = new MCR_Node(goalPos);
    nodes.push_back(startNode);
    nodes.push_back(goalNode);

    // Early exit flag
    bool goalConnected = false;

    // PRM Sampling
    for(int k=0; k<maxIters; k++){
        // Sample
        arr q_rand(startPos.N);
        if(rnd.uni() < 0.1){ // Goal Bias
            q_rand = goalPos;
        } else {
            for(uint i=0; i<q_rand.N; i++) q_rand(i) = rnd.uni(bounds_lo(i), bounds_up(i));
        }

        // Nearest Neighbor (Brute force for simplicity)
        MCR_Node* q_near = nodes[0];
        double min_dist = length(q_near->pos - q_rand);
        for(MCR_Node* n : nodes){
            double d = length(n->pos - q_rand);
            if(d < min_dist){ min_dist = d; q_near = n; }
        }

        // Steer
        arr vec = q_rand - q_near->pos;
        double dist = length(vec);
        if(dist > stepSize) vec *= (stepSize / dist);
        arr q_new = q_near->pos + vec;

        // Check Validity (Node check)
        bool isHard; uintA dummy;
        checkCollisions(q_new, isHard, dummy);
        //C.view(true, ("MCR sampling step: " + std::to_string(k)).c_str());
        if(isHard) continue;

        // Add Node
        MCR_Node* newNode = new MCR_Node(q_new);
        nodes.push_back(newNode);

        // Connect Neighbors
        for(MCR_Node* n : nodes){
            if(n == newNode) continue;
            double d = length(n->pos - newNode->pos);
            if(d <= connRadius){
                uintA edgeConstr;
                // Check edge
                double cost = getEdgeData(n->pos, newNode->pos, edgeConstr, 0.1); 
                
                if(cost >= 0){ // valid
                    n->neighbors.push_back({newNode, cost, edgeConstr});
                    newNode->neighbors.push_back({n, cost, edgeConstr});
                    
                    // Early exit check: if goal is now connected to start
                    if(earlyExit && (n == goalNode || newNode == goalNode) && !goalConnected){
                        // TODO: Add better early exit
                        if(hasPathToGoal()){
                            goalConnected = true; // this should be true to trigger the break
                            if(verbose > 0) std::cout << "MCR: Early exit - goal connected at iteration " << k << std::endl;
                            break;
                        }
                    }
                }
            }
        }
        
        if(goalConnected) break;
    }

    if(verbose > 0) std::cout << "MCR: Graph built (" << nodes.size() << " nodes)." << std::endl;
    MCRResult res = runAStar();

    if(!res.success){
        if(verbose) std::cout << "MCR Failed to find a path." << std::endl;
        return {};
    }
    
    if(verbose) std::cout << "MCR Found path. Blocking objects: " << res.removedConstraints << std::endl;
    
    return res.removedConstraints;
}

// ============================================================================== //

bool MCR_PathFinder::hasPathToGoal(){
    // Simple BFS to check connectivity
    std::set<MCR_Node*> visited;
    std::queue<MCR_Node*> q;
    
    q.push(startNode);
    visited.insert(startNode);
    
    while(!q.empty()){
        MCR_Node* curr = q.front();
        q.pop();
        
        if(curr == goalNode) return true;
        
        for(const auto& edge : curr->neighbors){
            if(!visited.count(edge.target)){
                visited.insert(edge.target);
                q.push(edge.target);
            }
        }
    }
    
    return false;
}

// ============================================================================== //

// 1. Change the struct to use std::vector for safety inside the PQ
struct AStarItem {
    double f;
    double g;
    MCR_Node* node;
    MCR_Node* parent;
    std::vector<uint> constraints; // Changed from uintA to std::vector<uint>
    
    bool operator>(const AStarItem& other) const { return f > other.f; }
};

MCRResult MCR_PathFinder::runAStar(){
    std::priority_queue<AStarItem, std::vector<AStarItem>, std::greater<AStarItem>> pq;
    std::map<MCR_Node*, double> visited;
    
    struct ParentInfo { MCR_Node* p; uintA c; };
    std::map<MCR_Node*, ParentInfo> cameFrom;

    double start_h = length(startNode->pos - goalNode->pos);
    pq.push({start_h, 0.0, startNode, nullptr, {}});

    while(!pq.empty()){
        AStarItem cur = pq.top();
        pq.pop();

        if(visited.count(cur.node) && visited[cur.node] <= cur.g) continue;
        visited[cur.node] = cur.g;
        
        uintA curConstraints;
        curConstraints.resize(cur.constraints.size());
        for(uint i=0; i<cur.constraints.size(); i++) curConstraints(i) = cur.constraints[i];

        if(cur.parent) cameFrom[cur.node] = {cur.parent, curConstraints};

        // Goal Check
        if(length(cur.node->pos - goalNode->pos) < 0.2){
            MCRResult res;
            res.success = true;
            res.totalCost = cur.g;
            
            // Reconstruct Path
            MCR_Node* itr = cur.node;
            std::set<uint> allConstr;
            
            std::vector<arr> pathNodes;
            while(itr){
                pathNodes.push_back(itr->pos);
                if(cameFrom.count(itr)){
                    for(uint id : cameFrom[itr].c) allConstr.insert(id);
                    itr = cameFrom[itr].p;
                } else {
                    itr = nullptr;
                }
            }
            
            // Build path array properly with correct 2D indexing
            if(!pathNodes.empty()) {
                uint dim = pathNodes[0].N;  // Should be 2 for 2D positions
                uint numNodes = pathNodes.size();
                
                res.path.resize(numNodes, dim);
                
                // Copy in reverse order (start->goal) using 2D indexing
                for(uint i=0; i<numNodes; i++) {
                    uint reverseIdx = numNodes - 1 - i;
                    for(uint j=0; j<dim; j++) {
                        res.path(reverseIdx, j) = pathNodes[i](j);
                    }
                }
            }
            

            for(uint id : allConstr){
                // Find frame name from ID
                for (rai::Frame* f : C.frames) {
                    if (f->ID == id) {
                        res.removedConstraints.append(f->name);
                        break;
                    }
                }
            }
            return res;
        }

        // Expand
        for(const auto& edge : cur.node->neighbors){
            double new_g = cur.g + edge.cost;
            
            if(!visited.count(edge.target) || new_g < visited[edge.target]){
                double h = length(edge.target->pos - goalNode->pos);
                
                std::vector<uint> nextConstraints = cur.constraints;
                for(uint id : edge.constraints) nextConstraints.push_back(id);
                
                pq.push({new_g + h, new_g, edge.target, cur.node, nextConstraints});
            }
        }
    }

    return MCRResult(); 
}

} // namespace rai

