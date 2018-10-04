/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Incorporation into open source software is not permitted.
 * Use in private (closed source) projects for academic research is permitted.
 */


#ifndef GLC_INTERFACE_H
#define GLC_INTERFACE_H

#include<deque>

#include<glc/glc_interpolation.h>

namespace glc{
  
  class Inputs{
    std::deque<std::valarray<double>> points;
  public:
    void addInputSample(std::valarray<double>& _input);
    const std::deque<std::valarray<double>>& readInputs();
  };
  
  class Heuristic{
  protected:
  public: 
    virtual double costToGo(const std::valarray<double>& x0)=0;
  }; 
  
  class CostFunction{
  protected:
    const double lipschitz_constant;
  public:
    CostFunction(double _lipschitz_constant);
    
    virtual double cost(const std::shared_ptr<InterpolatingPolynomial>& trajectory, const std::shared_ptr<InterpolatingPolynomial>& control, double t0, double tf)=0;

    double getLipschitzConstant();
  };
  
  class GoalRegion{
  public:
    virtual bool inGoal(const std::shared_ptr<InterpolatingPolynomial>& traj, double& time)=0;//TODO get crossing time for accurate cost eval
  };
  
  class Obstacles{
  public:
    int collision_counter=0;
    virtual bool collisionFree(const std::shared_ptr<InterpolatingPolynomial>& x)=0; 
  };
  
  class DynamicalSystem{
  protected:
    const double max_time_step;
  public:
    int sim_counter=0;
    
    DynamicalSystem(double _max_time_step);
    
    //This is the function defining the dynamic model x'(t)=f(x(t),u(t))
    virtual void flow(std::valarray<double>& dx, const std::valarray<double>& x,const std::valarray<double>& u)=0;
    
    //Planner requires access to a Lipschitz constant on the vector field
    virtual double getLipschitzConstant()=0;
    
    //Assigns segment to a polynomial connecting x(t1)=x0 to x(t2) 
    virtual void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2)=0;
    
    //Calls step repeatedly to create a spline approximating the solution to an ODE 
    void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u);
  };
}

#endif