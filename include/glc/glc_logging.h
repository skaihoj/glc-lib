#ifndef GLC_LOGGING_H
#define GLC_LOGGING_H

#include<glc/glc_state_equivalence_class.h>
#include<glc/glc_interpolation.h>
#include <fstream>

namespace glc{

void nodesToFile(const std::string& name, const std::string& path, const std::set<StateEquivalenceClass>& domains){
  std::ofstream points;
  points.open(path+name);
  
  for(auto& x : domains){
    std::valarray<double> state = x.label->state;
    for(int j=0;j<state.size()-1;j++){
      points << state[j] << ",";
    }
    points << state[state.size()-1] << std::endl;
  }
  points.close();
}

void trajectoryToFile(const std::string& name, 
                      const std::string& path, 
                      const std::shared_ptr<InterpolatingPolynomial> traj, 
                      int num_points){
  std::ofstream points;
  points.open(path+name);
  double t=traj->initialTime();
  double dt=(traj->numberOfIntervals()*traj->intervalLength())/num_points;
  for(int i=0;i<num_points;i++){
    std::valarray<double> state = traj->at(t);
    for(int j=0;j<state.size()-1;j++){
      points << state[j] << ",";
    }
    points << state[state.size()-1] << std::endl;
    t+=dt;
  }
  points.close();
}

//Write a vector out on the screen
void printVector(const std::valarray<double>& x){
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

}//namespace atg
#endif