#ifndef GLC_LOGGING_H
#define GLC_LOGGING_H

#include <fstream>
#include <set>

#include<glc/glc_state_equivalence_class.h>
#include<glc/glc_interpolation.h>

namespace glc{

void nodesToFile(const std::string& name, const std::string& path, const std::set<StateEquivalenceClass>& domains);

void trajectoryToFile(const std::string& name, 
                      const std::string& path, 
                      const std::shared_ptr<InterpolatingPolynomial> traj, 
                      int num_points);

}//namespace atg
#endif