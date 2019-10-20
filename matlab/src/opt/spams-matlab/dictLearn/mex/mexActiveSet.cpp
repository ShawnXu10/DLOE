/* Software SPAMS v2.5 - Copyright 2009-2014 Julien Mairal
 *
 * This file is part of SPAMS.
 *
 * SPAMS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SPAMS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with SPAMS.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
 * \file
 *                toolbox dictLearn
 *
 *                by Yuansi Chen and Julien Mairal 
 *                julien.mairal@inria.fr
 *
 *                File mexArchetypalAnalysis.cpp
 * \brief mex-file, function mexArchetypalAnalysis
 * Usage: [Z ,[A],[B]] = mexArchetypalAnalysis(X,param,[Z0]);
 * output a dictionary Z
 */

#include <mexutils.h>
#include <arch.h>

template <typename T>
inline void callFunction(mxArray* plhs[], const mxArray*prhs[],const int nrhs,
      const int nlhs) {
    Matrix<T> Z;
    Vector<T> X_cur;
    Vector<T> alpha;
    Matrix<T> alphaX;
    
    T lambda2 = getScalarStructDef<T>(prhs[2],"lambda2",1e-5);
    T epsilon = getScalarStructDef<T>(prhs[2],"epsilon",1e-5);
    bool warm = false;
    int p;

    getMatrix(prhs[0], Z);
    getVector(prhs[1], X_cur);
    p = Z.n();
    plhs[0]=createMatrix<T>(p,1);
    getMatrix(plhs[0], alphaX);
    //alphaX.refCol(1,alpha);   
    activeSet<T>(Z, X_cur, alpha, lambda2, epsilon, warm );
    alphaX.addVecToCols(alpha, 1);
    /*for(int i=0; i<alpha.n(); ++i)
      std::cout << alphaX[i] << ' ';*/
};

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

   if (mxGetClassID(prhs[0]) == mxDOUBLE_CLASS) {
      callFunction<double>(plhs,prhs,nrhs,nlhs);
   } else {
      callFunction<float>(plhs,prhs,nrhs,nlhs);
   }
}