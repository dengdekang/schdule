#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __ISAECC
#define __ISAECC
class Isaecc :public Algorithm{
    public:
        Isaecc ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();
};
#endif