#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __HEFT__
#define __HEFT__
class Heft :public Algorithm{
    public:
        Heft ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();
};
#endif