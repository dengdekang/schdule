#include "schedule_base.hpp"
#include "algorithm_base.hpp"
#ifndef __SARC__
#define __SARC__
class Sarc :public Algorithm{
    public:
        Sarc ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):Algorithm(dag,wect,p) {
        }
        void compute() override;
        // virtual double get_reliabilitys() override{
        //     if(reliability>0)
        //         return reliability ;
        //     double total=0;
        //     for(auto &item:result)
        //         total+=item.get_reliability();
        //     reliability =  std::pow(2,total/get_factor());
        //     return reliability ;
        // }
        
        virtual void print() override{
            for(auto & task : result){
                std::cout<<"task id:"<<task.get_task_id()+1<< "\t\test:"<<task.get_est()<<"\t\taft:"<<task.get_aft()<<"\t\tenergy consume:"
                        <<task.get_energy_consume()<<"\t\tproc id:"<<task.get_running_proc_id()+1
                        <<"\t\tfreq:"<<task.get_running_freq()<<"\t"<<"reliability:"<<task.get_reliability()<<std::endl;
            }
        }

        virtual double get_reliabilitys()const override{

            double relia=0;
            for(auto &item:result)
                relia +=item.get_reliability();
            
            return std::pow(2,relia/get_factor()) ;

        }

};
#endif