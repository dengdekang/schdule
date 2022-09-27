#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __XXXX__
#define __XXXX__
    class XXXX :public Algorithm{
    public:
        XXXX ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute()override;
        void set_alpha(double a){
            alpha=a;
        }
        double get_alpha()const{
            return alpha;
        }
        virtual double get_schedule_length()const override{
            double max_aft=0;
            for(auto &item:result.back().get_procs_aft())
                max_aft=std::max(max_aft,item);
            return max_aft;
        }
    private:
        double alpha;

};


#endif