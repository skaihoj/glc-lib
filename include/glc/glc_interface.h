/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Incorporation into open source software is not permitted.
 * Use in private (closed source) projects for academic research is permitted.
 */


#ifndef GLC_INTERFACE_H
#define GLC_INTERFACE_H

#include<glc/glc_interpolation.h>


namespace glc{
  
  class Inputs{
  public:
    std::deque<std::valarray<double>> points;//TODO private and
    
    void addInputSample(std::valarray<double>& _input){points.push_back(_input);}
  };
  
  class Heuristic{
  protected:
    std::valarray<double> goal;
  public: 
    void setGoal(std::valarray<double> _goal){goal=_goal;}
    virtual double costToGo(const std::valarray<double>& x0)=0;
  }; 
  
  class CostFunction{
  protected:
    const double lipschitz_constant;
  public:
    CostFunction(double _lipschitz_constant):lipschitz_constant(_lipschitz_constant){}
    
    double getLipschitzConstant(){return lipschitz_constant;}
    
    //Integrates a running cost of the trajectroy and control from t0 to tf.
    virtual double cost(const std::shared_ptr<InterpolatingPolynomial>& trajectory, const std::shared_ptr<InterpolatingPolynomial>& control, double t0, double tf)=0;
  };
  
  class GoalRegion{
  public:
    //Returns true if traj is in the goal and sets t to min time where intersects goal
    virtual bool inGoal(const std::shared_ptr<InterpolatingPolynomial>& traj, double& time)=0;//TODO get crossing time for accurate cost eval
  };
  
  class Obstacles{
  public:
    int collision_counter=0;
    virtual bool collisionFree(const std::shared_ptr<InterpolatingPolynomial>& x)=0; 
  };
  
  //Vector field with numerical integration
  class DynamicalSystem{
  protected:
    const double max_time_step;
  public:
    int sim_counter=0;
    
    DynamicalSystem(double _max_time_step) : max_time_step(_max_time_step){}
    
    //This is the function defining the dynamic model x'(t)=f(x(t),u(t))
    virtual void flow(std::valarray<double>& dx, const std::valarray<double>& x,const std::valarray<double>& u)=0;
    
    //Planner requires access to a Lipschitz constant on the vector field
    virtual double getLipschitzConstant()=0;
    
    //Assigns segment to a polynomial connecting x(t1)=x0 to x(t2) 
    virtual void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2)=0;
    
    //Calls step repeatedly to create a spline approximating the solution to an ODE 
    void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u){
      assert(tf>t0);
      double num_steps=ceil((tf-t0)/max_time_step);
      double integration_step=(tf-t0)/num_steps;
      solution = std::shared_ptr<InterpolatingPolynomial>(new InterpolatingPolynomial(integration_step,t0,x0.size(),4));
      solution->reserve(num_steps);
      //set initial state and time
      std::valarray<double> state=x0;
      double time=t0;
      std::shared_ptr<InterpolatingPolynomial> traj_segment;
      //integrate
      for(int i=0;i<num_steps;i++){
        //Use numerical integration scheme to compute a spline extending from state with input u([t,t+integration_step])
        step(traj_segment,state,u,time,time+integration_step);
        //add traj_segment to solution
        solution->concatenate(traj_segment);
        time+=integration_step;
        state=traj_segment->at(time);
      }
      sim_counter++;
      return;
    }
  };
  
}

#endif