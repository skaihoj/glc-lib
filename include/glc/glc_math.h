/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Incorporation into open source software is not permitted.
 * Use in private (closed source) projects for academic research is permitted.
 */

#ifndef GLC_MATH_H
#define GLC_MATH_H

#include <iostream>
// #include <cstdio>
#include <vector>
#include <cassert>
// #include <float.h>
// #include <string.h>
// #include <time.h>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>

namespace glc{

/**
 * \brief Computes the square of a floating point number
 * \param x a double that is squared
 * \return The square of x
 */  
double sqr(const double x);

/**
 * \brief Calculates the square of the L2-norm of a vector.
 * \param x is the input vector whose norm will be squared.
 * \returns The square of the norm of the parameter x.
 */
// double normSquare(const std::valarray<double>& x);

/**
 * \brief Calculates the square of the L2-norm of a vector.
 * \param x is the input vector whose norm will be squared
 * \param y is set to the square of the norm of the parameter x.
 */

// void normSquare(const std::valarray<double>& x, double& y){
//     y=0.0;
//     for(int i=0;i<x.size();i++){
//         y+=x[i]*x[i];
//     }
//     return;    
// }

// std::valarray<double> diff(const std::valarray<double>& x, const std::valarray<double>& y){
//     return x-y;
// }

std::vector<int> vecFloor(const std::valarray<double>& x);

double dot(const std::valarray<double>& x,const std::valarray<double>& y);

double norm2(const std::valarray<double>& x);

double norm_sqr(const std::valarray<double>& x);

std::valarray<double> linearSpace(const double& start, const double& end, const int points);

}//namespace glc
#endif
