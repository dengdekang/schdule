#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __EFSRG__
#define __EFSRG__
class Efsrg :public Algorithm{
    public:
        Efsrg ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute();

        virtual double get_schedule_length()const override{
            double max_aft=0;
            for(auto &item:result){
                for(auto &item_aft:item.get_procs_aft())
                    max_aft=std::max(item_aft,max_aft);
            }
            // for(auto &item:result.back().get_procs_aft())
            //     max_aft=std::max(max_aft,item);
            return max_aft;
        }

};
#endif