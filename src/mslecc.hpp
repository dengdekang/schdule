#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#include <iostream>
#include <string>
#include <vector>
#ifndef __MSLECC
#define __MSLECC
class Mslecc :public Algorithm{
    public:
        Mslecc ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();
};
#endif