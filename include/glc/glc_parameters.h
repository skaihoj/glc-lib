#ifndef GLC_PARAMETERS_H
#define GLC_PARAMETERS_H

#include<valarray>
#include<iostream>

namespace glc{
  struct Parameters{
    //Initial condition
    std::valarray<double> x0;
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
    
    void printParams();
  };
  
}//namespace glc

#endif