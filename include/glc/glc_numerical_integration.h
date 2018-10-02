#ifndef GLC_NUMERICAL_INTEGRATION_H
#define GLC_NUMERICAL_INTEGRATION_H

#include <glc/glc_interface.h>
#include <glc/glc_interpolation.h>

namespace glc{

  //An integration scheme which has local error of O(dt^3)  
  class SymplecticEuler : public DynamicalSystem{
  protected:
    std::valarray<double> x1,x2,f0,f1,f2;
    double h;
  public:
    SymplecticEuler(double _max_time_step, int state_dim) : DynamicalSystem(_max_time_step),x1(state_dim),x2(state_dim),f0(state_dim),f1(state_dim),f2(state_dim){}
    //Step returns a spline between collocation points of the integrator
    void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2) override {  
      assert(t1<t2 && "Integration step must be positive in SymplecticEuler");
      
      h=t2-t1;
      flow(f0,x0,u->at(t1));
      x1=x0+0.5*h*f0;
      flow(f1,x1,u->at(t1+0.5*h));
      x2=x0+h*f1;
      flow(f2,x2,u->at(t2));
      
      //Cubic interpolation between x0 and x2 with x'(t1)=f(x0,u(t0)) and x'(t2)=f(x2,u(t2))
      std::vector< std::valarray<double> > cubic;
      cubic.push_back(x0);//t^0 term
      cubic.push_back(f0);//t^1 term
      cubic.push_back((-2.0*f0+3.0*f1-f2)/h);//t^2 term
      cubic.push_back((f0-2.0*f1+f2)/(h*h));//t^3 term
      std::vector<std::vector< std::valarray<double>>> knot_point({cubic});
      segment=std::shared_ptr<InterpolatingPolynomial>(new InterpolatingPolynomial(knot_point,t2-t1,t1,x0.size(),4));
    }
  };  
  
  
}//namespace
#endif