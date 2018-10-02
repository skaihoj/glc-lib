#ifndef GLC_PARAMETERS_H
#define GLC_PARAMETERS_H

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
    
    void printParams(){
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
  
}//namespace glc

#endif