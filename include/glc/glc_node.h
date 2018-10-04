/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Incorporation into open source software is not permitted.
 * Use in private (closed source) projects for academic research is permitted.
 */

#ifndef GLC_UTILS_H
#define GLC_UTILS_H

#include <memory>
#include <chrono>
#include <random>
#include <queue>
#include <deque>
#include <valarray>
#include <algorithm> 

namespace glc{
  
  struct Node{
    
    std::shared_ptr<Node> parent;
    std::vector< std::shared_ptr<Node> > children;//TODO maybe std::unordered_map?
    std::valarray<double> state;
    double time=0;
    double cost=0;
    double merit=0;//Cost plus heuristic
    int u_idx=0; // index of control input from parent
    int depth=0;//Depth in search tree
    bool in_goal = false;//set to true if the arc connecting parent to state intersects goal
    
    Node(int _card_omega, 
         int _control_index, 
         double _cost, 
         double _cost_to_go, 
         const std::valarray<double>& _state, 
         double _time,
         const std::shared_ptr<Node> _parent
    );
  };
  
  struct NodeMeritOrder{
  public:
    bool operator()(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2);
  };
  
}//namespace glc
#endif
