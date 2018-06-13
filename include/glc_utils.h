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
#include <algorithm> 
#include <math_utils.h>

namespace glc{
  
  class node; // TODO rename to Node
  typedef std::shared_ptr<node> nodePtr; 
  
  class node
  {
  public:
    static const nodePtr inf_cost_node;
    
    nodePtr parent;
    std::vector< nodePtr > children;//TODO maybe std::unordered_map?
    vctr state;
    double time=0;
    double cost=0;
    double merit=0;//Cost plus heuristic
    int u_idx=0; // index of control input from parent
    int depth=0;//Depth in search tree
    bool in_goal = false;//set to true if the arc connecting parent to state intersects goal
    
    node(int _card_omega, 
         int _control_index, 
         double _cost, 
         double _cost_to_go, 
         const vctr& _state, 
         double _time,
         const nodePtr _parent
    ) : 
    children(_card_omega), 
    cost(_cost), 
    merit(_cost_to_go+_cost), 
    time(_time), 
    parent(_parent), 
    state(_state), 
    u_idx(_control_index){}
  };
  
  bool compareMerit(const nodePtr& node1, const nodePtr& node2){
    return node1->merit<node2->merit;
  }
  class NodeMeritOrder{
  public:
    bool operator()(const nodePtr& node1, const nodePtr& node2){
      return not compareMerit(node1,node2);//negation so top of queue is min not max     
    }
  };
  const nodePtr node::inf_cost_node(new node(0, -1, DBL_MAX, DBL_MAX,vctr(0),0,nullptr));
  
  class Domain{
  public:
    std::vector<int> coordinate;//serves as index of partition domain
    nodePtr label;//pointer to node labeling region
    std::priority_queue<nodePtr, std::vector<nodePtr>, NodeMeritOrder> candidates;//candidates for relabeling
    
    Domain():label(node::inf_cost_node){}
    Domain(const nodePtr& _label){
      label = _label;
      coordinate=vecFloor(label->state);
    }
    bool empty(){
      return label == node::inf_cost_node;
    }
    //Lexicographical order of integer tuple for sorting stl set of domains
    bool operator<(const Domain& y) const{
      assert(coordinate.size()==y.coordinate.size());
      return std::lexicographical_compare <std::vector<int>::const_iterator, std::vector<int>::const_iterator>
      (coordinate.begin(), coordinate.end(), y.coordinate.begin(), y.coordinate.end());
    }
  };
  
  class Spline;
  typedef std::shared_ptr<Spline> splinePtr;
  typedef std::vector< std::vector< std::valarray<double> > > splineTensor;
  
  class Spline{
    int degree;//coefficients in polynomial segments
    int dimension;//number of individual splines
    double collocation_interval;//time interval between collocation points
    double t0;//initial time
    
    //coefficient_array[time_interval_index][polynomial_coefficient_index][polynomial_coordinate_index]
  public: 
    splineTensor coefficient_array;
    Spline(const splineTensor& _coeff_array, 
           const double& _collocation_interval, 
           const double& _t0, 
           const double& _dimension, 
           const double& _degree) : 
           collocation_interval(_collocation_interval),
           t0(_t0),
           dimension(_dimension), 
           degree(_degree),
           coefficient_array(_coeff_array){}
           
           Spline(const double& _collocation_interval, 
                  const double& _t0, 
                  const double& _dimension, 
                  const double& _degree) : 
                  collocation_interval(_collocation_interval),
                  t0(_t0),
                  dimension(_dimension), 
                  degree(_degree){coefficient_array = splineTensor();}
                  
                  //Copy tail into the back of this Spline. Ignores t0 of the tail segment 
                  void concatenate(const splinePtr& tail){
                    assert(tail->dimension==dimension && tail->degree==degree && tail!=nullptr);
                    coefficient_array.insert(coefficient_array.end(),
                                             tail->coefficient_array.begin(),
                                             tail->coefficient_array.end());
                  }
                  //Push a single collocation point into the spline
                  void push(const std::vector< std::valarray<double> >& knot){coefficient_array.push_back(knot);}
                  
                  //Evaluate spline at time t
                  std::valarray<double> at(const double& t){
                    int index = std::min( (int)coefficient_array.size()-1,std::max(0,(int)std::floor((t-t0)/collocation_interval)));
                    double time = (t-t0)-collocation_interval*((double)index);
                    std::valarray<double> eval(0.0,dimension);
                    for(int i=0;i<degree;i++){
                      eval+=coefficient_array[index][i]*pow(time,i);
                    }
                    return eval;
                  }
                  // Allocates memory in coefficient_array for "size" collocation points
                  void reserve(const int& size){
                    assert(size>=0 && "Cannot reserve negative space for Spline coefficients");
                    coefficient_array.reserve(size);
                  }
                  int numberOfIntervals(){return coefficient_array.size();}
                  double intervalLength(){return collocation_interval;}
                  double initialTime(){return t0;}
                    
  };
  
  Spline makeDeepCopy(const splinePtr& to_copy){
    return *(to_copy.get());
  }
  
  //Write a vector out on the screen
  void printVector(const vctr& x){
    std::cout << "(";
    for(int i=0;i<(int)x.size()-1;i++){
      std::cout << x[i] << ",";
    }
    std::cout << x[x.size()-1] << ")" << std::endl;
  }
  
  void printVector(const std::vector<int>& x){
    std::cout << "(";
    for(int i=0;i<(int)x.size()-1;i++){
      std::cout << x[i] << ",";
    }
    std::cout << x[x.size()-1] << ")";
  }
  
  //Write trajectory out on the screen
  void printSpline(const splinePtr& sol,int num_points, const std::string& msg){
    std::cout << std::endl << "*****"<< msg << "*****" << std::endl;
    double t=sol->initialTime();
    double dt=(sol->intervalLength()*sol->numberOfIntervals())/(double)num_points;
    for(int i=0;i<num_points+1;i++){
      printVector(sol->at(t));
      t+=dt;
    }
  }
  
  
  struct PlannerOutput{
    double cost;
    double time;
    bool solution_found;
  };
  
  class Parameters{
  public:
    //Initial condition
    vctr x0;
    //Discretization resolution
    int res;
    //State space dimension
    int state_dim;
    //Input space dimension
    int control_dim;
    //Maximum iterations
    int max_iter;
    //Change time coordinate to be appropriate
    double time_scale;
    //Initial partition size
    double partition_scale;
    //Adjust initial depth limit
    int depth_scale;
    //integration step
    double dt_max;
    //scaling of grid
    
    void print_params(){
      std::cout << "state_dim " << state_dim << std::endl;
      std::cout << "control_dim " << control_dim << std::endl;
      std::cout << "res " << res << std::endl;
      std::cout << "max_iter " << max_iter << std::endl;
      std::cout << "time_scale " << time_scale << std::endl;
      std::cout << "partition_scale " << partition_scale << std::endl;
      std::cout << "depth_scale " << depth_scale << std::endl;
      std::cout << "dt_max_scale " << partition_scale << std::endl;
      std::cout << "size of x0 " << x0.size() << std::endl;
      
      return;        
    }
  };
  void nodesToFile(const std::string& name, const std::string& path, const std::set<Domain>& domains){
    std::ofstream points;
    points.open(path+name);
    
    for(auto& x : domains){
      glc::vctr state = x.label->state;
      for(int j=0;j<state.size()-1;j++){
        points << state[j] << ",";
      }
      points << state[state.size()-1] << std::endl;
    }
    points.close();
  }
  
  void trajectoryToFile(const std::string& name, const std::string& path, const splinePtr& traj, int num_points)
  {
    std::ofstream points;
    points.open(path+name);
    double t=traj->initialTime();
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/num_points;
    for(int i=0;i<num_points;i++){
      glc::vctr state = traj->at(t);
      for(int j=0;j<state.size()-1;j++){
        points << state[j] << ",";
      }
      points << state[state.size()-1] << std::endl;
      t+=dt;
    }
    points.close();
  }
  
}//end namespace
#endif
