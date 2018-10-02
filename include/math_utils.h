// /* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
//  * Written by Brian Paden
//  * Incorporation into open source software is not permitted.
//  * Use in private (closed source) projects for academic research is permitted.
//  */
// 
// #ifndef GLC_MATH_H
// #define GLC_MATH_H
// 
// #include <iostream>
// #include <cstdio>
// #include <vector>
// #include <cassert>
// #include <ctime>
// #include <cmath>
// #include <float.h>
// #include <string.h>
// #include <time.h>
// #include <sstream>
// #include <fstream>
// #include <string>
// #include <valarray>
// 
// namespace glc{
// typedef std::valarray<double> vctr;
// 
// 
// /**
//  * \brief 
//  */
// double sqr(const double x){
//     return x*x;
// }
// 
// double normSquare(const vctr& x){
//     double y=0.0;
//     for(int i=0;i<x.size();i++){
//         y+=x[i]*x[i];
//     }
//     return y;
// }
// 
// void normSquare(const vctr& x, double& y){
//     y=0.0;
//     for(int i=0;i<x.size();i++){
//         y+=x[i]*x[i];
//     }
//     return;    
// }
// vctr diff(const vctr& x, const vctr& y){
//     return x-y;
// }
// 
// std::vector<int> vecFloor(const vctr& x){
//     std::vector<int> floored(x.size());
//     for(int i=0;i<x.size();i++){
//         floored.at(i)=(int)floor(x[i]);
//     }
//     return floored;
// }
// 
// //inner product
// double dot(const vctr& x,const vctr& y){
//     assert(x.size()==y.size());
//     double z=0;
//     for(int i=0;i<y.size();i++){
//         z += x[i]*y[i];
//     }
//     return z;
// }
// 
// double norm2(const vctr& x){
//     double norm=0;
//     for(int i=0; i<x.size(); i++){
//         norm=norm+sqr(x[i]);
//     }
//     return std::sqrt(norm);
// }
// 
// double norm_sqr(const vctr& x){
//     double norm=0;
//     for(int i=0; i<x.size(); i++){
//         norm=norm+sqr(x[i]);
//     }
//     return norm;
// }
// 
// std::valarray<double> linearSpace(const double& start, const double& end, const int points){
//   glc::vctr lin_space(points);
//   double step = (end-start)/double(points);
//   lin_space[0]=start;
//   for(int i=1;i<points;i++){lin_space[i]=lin_space[i-1]+step;}
//   return lin_space;
// }
// }//namespace
// #endif