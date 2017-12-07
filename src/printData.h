//
// Created by francesco on 01/11/17.
//
#include "yocto_gl.h"
using namespace std;
using namespace ygl;
namespace pd{
  void printFrame(const frame<float,3>& M){
    printf("%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n\n",
           M.x.x,M.y.x,M.z.x,
           M.x.y,M.y.y,M.z.y,
           M.x.z,M.y.z,M.z.z,
           M.o.z,M.o.z,M.o.z);
  }

  template <typename T, int N, int M>
  void printMatrix(mat<T, N, M> m){
    for(int i=0; i<M; i++) {
      for (int j = 0; j < N*M; j=j+N) {
        printf(" %.6g;    ", m.operator[](i+j));
      }
      printf("\n");
    }
    printf("\n");
  }

  template <typename T, int N>
  void printVector(vec<T, N> v){
    for(int i=0; i<N; i++) {
      printf(" %.6g;    ", v.operator[](i));
    }
    printf("\n");
  }
}

#ifndef GRAPHICS17B_HW01_PRINTDATA_H
#define GRAPHICS17B_HW01_PRINTDATA_H

#endif //GRAPHICS17B_HW01_PRINTDATA_H
