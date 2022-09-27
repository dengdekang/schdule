#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#include <iostream>
#ifndef __LESA__
#define __LESA__
class Lesa :public Algorithm{
    using Iter=std::vector<Task>::iterator;
    Lesa( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
    }
    void compute();
    double get_procs_avg_energy_consume(const Task &t)const;
    double get_task_energy_consume_weight(Iter beg,Iter end )const ;
};
#endif