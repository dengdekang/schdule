#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __ESRG__
#define __ESRG__
class Esrg :public Algorithm{
    public:
        Esrg ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();
};
#endif