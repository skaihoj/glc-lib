// /* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
//  * Written by Brian Paden
//  * Incorporation into open source software is not permitted.
//  * Use in private (closed source) projects for academic research is permitted.
//  */
// #ifndef GLC_ADAPTER_H
// #define GLC_ADAPTER_H
// 
// #include <glc_interface.h>
// namespace glc{
//     
//     class ZeroHeuristic: public Heuristic{
//     public:    
//         double costToGo(const vctr& x0) override {return 0;}
//     };
//     
//     class MinTimeCost: public CostFunction //Min time 
//     {
//     public:
//         MinTimeCost():CostFunction(0.0){}
//         
//         double cost(const splinePtr& traj, const splinePtr& control) override {
//             return traj->numberOfIntervals()*traj->intervalLength();
//         }
//     };
//     
//     class SphericalGoal: public GoalRegion{
//         double goal_radius, goal_radius_sqr;
//         vctr error;
//         vctr x_g;
//         int resolution;
//     public:
//         
//         SphericalGoal(const int& _state_dim, const double& _goal_radius,int resolution):x_g(_state_dim,0.0){
//             //state_dim=_state_dim;
//             goal_radius=_goal_radius;
//             goal_radius_sqr=sqr(goal_radius);
//             error.resize(_state_dim);
//         }
//         bool inGoal(const splinePtr& traj) override {
//             double t=traj->initialTime();
//             double dt=traj->numberOfIntervals()*traj->intervalLength()/resolution;
//             for(int i=0;i<resolution;i++){
//               t+=dt;//don't need to check t0 since it was part of last traj
//               error=diff(x_g,traj->at(t));
//               if(dot(error,error)<goal_radius_sqr){return true;}
//             }
//             return false;
//         }
//         void setRadius(double r){
//             goal_radius = r;
//             goal_radius_sqr = r*r;
//         }
//         double getRadius(){return goal_radius;}
//         
//         void setGoal(vctr& _x_g){x_g=_x_g;}
//         
//         vctr getGoal(){return x_g;}
//     };
//     
//     class NoObstacles: public Obstacles{
//     public:        
//         bool collisionFree(const splinePtr& traj) override {
//           collision_counter++;
//           return true;
//         }
//     };
// }//end namespace
// 
// #endif