#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#include <iostream>
#include <string>
#include <vector>
#ifndef __DERG
#define __DERG
class Derg :public Algorithm{
    public:
        Derg ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();

};
#endif