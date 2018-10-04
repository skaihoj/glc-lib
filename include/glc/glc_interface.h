/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */


#ifndef GLC_INTERFACE_H
#define GLC_INTERFACE_H

//External dependencies
#include<vector>

//Local libraries
#include<glc/glc_interpolation.h>

namespace glc{

/** 
 * \brief This class defines a finite set of control inputs from the input space that are used by the planner to forward simulate the system dynamics
 * 
 * Mathematically, the GLC method produces an out put converting to a globally 
 * optimal solution for a particular problem instance as the resolution of the 
 * algorithm is increased. The user is responsible for implementing a derived 
 * class for Inputs which is parameterized by the algorithm resolution such 
 * that with increasing resolution, the dispersion of the discrete set of 
 * control inputs within the set of admissible control inputs converges to 
 * zero. 
 */  
class Inputs{
  //! \brief points stores the finite set of control inputs 
  std::vector<std::valarray<double>> points;
public:
  /**
   * \brief inserts a control input to the set of control inputs
   * 
   * \param[in] input_ is the control input added to the set of inputs
   * 
   * Two simple ways of generating a set of control inputs meeting the requirements
   * of the algorithm are to sample randomly from the set of admissible control 
   * inputs with the number of samples increasing with resolution or to intersect a
   * uniform grid with the set of admissible control inputs and refine the grid 
   * with increasing algorithm resolution.
   */
  void addInputSample(std::valarray<double>& input_);
  /**
   * \brief returns a read-only reference to the set of control inputs
   */
  const std::vector<std::valarray<double>>& readInputs() const ;
};


class Heuristic{
protected:
public: 
  virtual double costToGo(const std::valarray<double>& x0) const = 0;
}; 
  
class CostFunction{
protected:
  const double lipschitz_constant;
public:
  CostFunction(double _lipschitz_constant);
  
  virtual double cost(const std::shared_ptr<InterpolatingPolynomial>& trajectory, 
                      const std::shared_ptr<InterpolatingPolynomial>& control, 
                      double t0, 
                      double tf) const = 0;
  
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
}//namespace glc

#endif