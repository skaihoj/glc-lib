// /* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
//  * Written by Brian Paden
//  * Incorporation into open source software is not permitted.
//  * Use in private (closed source) projects for academic research is permitted.
//  */
// 
// 
// #ifndef GLC_PLANNER_H
// #define GLC_PLANNER_H
// 
// #include <map>
// #include <set>
// #include <limits>
// #include <memory>
// #include <stack>
// #include <queue>
// #include <deque>
// #include <unistd.h>
// 
// #include <glc_interface.h>
// #include <glc_utils.h>
// 
// namespace glc{  
//   
//   class GLCPlanner
//   {
//   public:
//     //Interface
//     nodePtr best = node::inf_cost_node;
//     nodePtr root_ptr;
//     DynamicalSystem* dynamics;
//     GoalRegion* goal;
//     Obstacles* obs;
//     CostFunction* cf;
//     Heuristic* h;
//     //Primitive data structures for GLC method
//     std::priority_queue<nodePtr,std::vector<nodePtr>,NodeMeritOrder> queue;
//     std::set<Domain> domain_labels;
//     std::set<Domain>::iterator it;
//     //
//     int depth_limit;
//     double partition_scale;
//     double eta;
//     double expand_time;
//     //Run statistics
//     bool foundGoal=false; bool live=true;
//     Parameters params;
//     std::deque<vctr> controls;
//     int sim_count = 0;
//     int coll_check = 0;
//     int iter=0;
//     clock_t run_time, tstart;
//     double UPPER=DBL_MAX;
//     
//     //Planner tree handling functions
//     void add_child(nodePtr parent, nodePtr child);//TODO move to node class
//     std::vector<nodePtr> pathToRoot(bool forward=false);
//     splinePtr recoverTraj(const std::vector<nodePtr>& path);
//     
//     
//     //Planner methods
//     void expand();
//     void plan(PlannerOutput& out);
//     void plan();
//     bool getSolution(splinePtr& traj_out);
//     
//     
//     //Constructor
//     GLCPlanner(Obstacles* _obs, 
//                GoalRegion* _goal, 
//                DynamicalSystem* _dynamics, 
//                Heuristic* _h, 
//                CostFunction* _cf, 
//                const Parameters& _params, 
//                const std::deque<vctr>& _controls):
//                 params(_params), 
//                 controls(_controls), 
//                 dynamics(_dynamics), 
//                 obs(_obs), 
//                 goal(_goal), 
//                 cf(_cf), 
//                 h(_h){
// 
//       root_ptr = nodePtr(new node(_controls.size(),0, 0,_h->costToGo(params.x0),params.x0,0,nullptr));
//       Domain d0(root_ptr);
//       queue.push(root_ptr);
//       domain_labels.insert(d0);
//       ////////////*Scaling functions*//////////////
//       // 1/R
//       expand_time=params.time_scale/(double)params.res;
//       //h(R)
//       depth_limit=params.depth_scale*params.res*floor(log(params.res));
//       //eta(R) \in \little-omega (log(R)*R^L_f)
//       if(dynamics->getLipschitzConstant()==0.0){
//         eta = params.res*log(params.res)*log(params.res)/params.partition_scale;
//       }
//       else{
//         eta = pow(params.res,1+dynamics->getLipschitzConstant())/params.partition_scale;
//       }
//       
//       /////////*Print Parameters*/////////////
//       std::cout << "\n\n\n\nPre-search summary:\n" << std::endl;
//       std::cout << "      Expand time: " << expand_time << std::endl;
//       std::cout << "      Depth limit: " << depth_limit <<  std::endl;
//       std::cout << "      Domain size: " << 1.0/eta << std::endl;
//       std::cout << "   Max iterations: " << params.max_iter << std::endl;
//       
//       tstart = clock();
//     }
//   };
//   
//   void GLCPlanner::add_child(nodePtr parent, nodePtr child){
//     child->parent = parent;
//     child->depth = parent->depth+1;
//     child->time = (parent->time+expand_time);
//     parent->children[child->u_idx] = child;
//   }
//   
//   void GLCPlanner::expand(){
//     iter++;
//     if(queue.empty()){
//       std::cout << "---The queue is empty. Resolution too low or no solution.---" << std::endl;
//       live=false;//TODO return this instead
//       return;
//     }
//     nodePtr current_node = queue.top();
//     queue.pop();
// 
//     //Goal checking
//     if(current_node->in_goal and current_node->cost < best->cost){
//       run_time = clock() - tstart;  
//       best=current_node;
//       UPPER=current_node->cost;
//       foundGoal=true;
//       live=false;//only for best-first search
//       std::cout << "\n\nFound goal at iter: " << iter << std::endl;
//       std::cout << "     solution cost: " << UPPER << std::endl;
//       std::cout << "      running time: " << (float) run_time/ (float) CLOCKS_PER_SEC << std::endl;
//       std::cout << "  Simulation count: " << dynamics->sim_counter << std::endl;
//       std::cout << "  Collision checks: " << obs->collision_counter << std::endl;
//       std::cout << "       Size of set: " << domain_labels.size() << std::endl;
//       std::cout << "     Size of queue: " << queue.size() << std::endl;
//     }
//     
//     if(current_node->depth >=depth_limit or iter>params.max_iter)
//     {
//       std::cout << "---exceeded depth or iteration limit---" << std::endl;
//       live=false;
//       return;
//     }
//     
//     //A set of domains visited by new nodes made by expand
//     std::set<Domain*> domains_needing_update; 
//     std::map<nodePtr, splinePtr> traj_from_parent;
//     std::map<nodePtr, splinePtr> control_from_parent;
//     
//     //Expand top of queue and store arcs in set of domains
//     for(int i=0;i<controls.size();i++){
//       splinePtr new_traj;
//       std::valarray<double> c0;
//       //Create a control signal spline which is a first order hold.
//       //u(t)=c0+c1*t. If expanding root, just use u(t)=constant;
//       if(current_node->parent==nullptr){
//         c0 = controls[i];
//       }
//       else{
//         c0 = controls[current_node->u_idx];
//       }
//       std::valarray<double> c1 = (controls[i]-c0)/expand_time;
//       std::vector<std::valarray<double> > segment({c0,c1});
//       splineTensor linear_interp({segment});
//       //The above parameters are used to construct new_control
//       splinePtr new_control(new Spline(linear_interp,expand_time,current_node->time,controls[i].size(),2));
//       //Forward simulate with new_control to get a cubic spline between collocation points
//       dynamics->sim(new_traj, current_node->time, current_node->time+expand_time , current_node->state, new_control);
//       nodePtr new_arc(new node(controls.size(),
//                                i,
//                                cf->cost(new_traj, new_control,current_node->time,current_node->time+expand_time)+current_node->cost, 
//                                h->costToGo(new_traj->at(current_node->time+expand_time)), 
//                                new_traj->at(current_node->time+expand_time), 
//                                current_node->time+expand_time,
//                                current_node));
//       
//       traj_from_parent[new_arc] = new_traj;
//       control_from_parent[new_arc] = new_control;
// 
//       //Create a region for the new trajectory
//       vctr w = eta * new_arc->state;
//       Domain d_new;
//       d_new.coordinate = vecFloor( w );
//       //Get the domain for the coordinate or create it and insert into labels.
//       Domain& bucket = const_cast<Domain&>( *(domain_labels.insert(d_new).first) );
//       //Add to a queue of domains that need inspection
//       domains_needing_update.insert(&bucket);
// 
//       if(compareMerit(new_arc,bucket.label)){
//         bucket.candidates.push(new_arc);
//       }     
//     }
//     //Go through the new trajectories and see if there is a possibility for relabeling before collcheck
//     for(auto& open_domain : domains_needing_update){
//       Domain& current_domain = *open_domain;
//       //We go through the queue of candidates for relabeling/pushing in each set
//       bool found_best = false;
//       while( (not found_best) and (not current_domain.candidates.empty()))
//       {
//         //If the top of the candidate queue is cheaper than the label we should coll check it
//         if(compareMerit(current_domain.candidates.top(),current_domain.label)){
//           const nodePtr& best_relabel_candidate = current_domain.candidates.top(); 
//           splinePtr candidate_traj = traj_from_parent[best_relabel_candidate];
//           if(obs->collisionFree(candidate_traj)){
//             add_child(current_node, best_relabel_candidate);
//             //Flag vertex if it's in the goal
//             double time;
//             if( goal->inGoal(candidate_traj,time)){
//               best_relabel_candidate->in_goal=true;
//               best_relabel_candidate->cost = best_relabel_candidate->parent->cost + cf->cost(candidate_traj,
//                                                       control_from_parent[best_relabel_candidate],
//                                                       candidate_traj->initialTime(),
//                                                       time);
//             }
//             queue.push(best_relabel_candidate);//anything coll free at this point goes to queue
//             if(!found_best){
//               found_best = true;
//               current_domain.label = best_relabel_candidate;
//             }
//           }
//         }
//         current_domain.candidates.pop();
//       }
//       if(current_domain.empty()){
//         domain_labels.erase(current_domain);
//       }
//     }
//     return;
//   }
//   void GLCPlanner::plan(){
//     while(live){
//       expand();
//     }
//     return;
//   }
//   
//   void GLCPlanner::plan(PlannerOutput& out){
//     GLCPlanner::plan();
//     out.cost=UPPER;
//     out.time=(float) run_time/ (float) CLOCKS_PER_SEC; 
//     out.solution_found=foundGoal;//TODO change to found_goal
//     return;
//   }
//   
//   //get the nodePtr path to the root with the order specified by foward
//   std::vector<nodePtr> GLCPlanner::pathToRoot(bool forward){
//     if(foundGoal==false){//this function doesn't work if the planner doesn't have a solution
//       std::vector<nodePtr> empty_vector;
//       return empty_vector;
//     }
//     nodePtr currentNode = best;
//     std::vector<nodePtr> path;
//     while( not (currentNode->parent == nullptr) ){
//       path.push_back(currentNode);
//       currentNode=currentNode->parent;
//     }
//     path.push_back(currentNode);
//     
//     if(forward){std::reverse(path.begin(),path.end());}
//     
//     return path;
//   }
//   
//   //return the planned trajectory
//   splinePtr GLCPlanner::recoverTraj(const std::vector<nodePtr>& path)
//   {
//     splinePtr opt_sol=nullptr;
//     if(path.size()<2){return opt_sol;}
//     //recalculate arcs connecting nodes
//     for(int i=0; i<path.size()-1;i++){
//       //The interval for the next polynomial segment is [t0,tf]
//       double t0=path[i]->time;
//       double tf=t0+expand_time; 
//       //A piecewise linear segment of control based on collocation points stored in nodePtr path[i]
//       std::valarray<double> c0;
//       if(i==0){c0 = controls[path[i+1]->u_idx];}//special case for root vertex - uses control of child
//       else{c0 = controls[path[i]->u_idx];}
//       std::valarray<double> c1 = (controls[path[i+1]->u_idx]-c0)/expand_time;
//       std::vector<std::valarray<double> > segment({c0,c1});
//       splineTensor linear_interp({segment});
//       splinePtr control_segment(new Spline(linear_interp,expand_time,t0,controls[path[i]->u_idx].size(),2));
//       //Simulate dynamics with input control_segment
//       splinePtr traj_segment;
//       dynamics->sim(traj_segment, t0, tf, path[i]->state,control_segment);
//       if(i==0){
//         opt_sol=splinePtr(new Spline(*(traj_segment.get())));//
//       }
//       else{
//         opt_sol->concatenate(traj_segment);
//       }
//       splineTensor coef(traj_segment->coefficient_array);
//     }
//     
//     return opt_sol;
//   }
// }//close namespace
// 
// struct PlannerOutput{
//   double cost;
//   double time;
//   bool solution_found;
// };
// #endif