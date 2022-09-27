#include "schedule_base.hpp"
#include <limits>
#ifndef __ALGORITHM_
#define __ALGORITHM_
class Algorithm{
    protected:
        DAG dag ;
        WCET wect ;
        std::vector<Process> procs;
        double energy_given;
        std::vector<Task> result;
        double total_energy;
        double goal_reliability ;
        double reliability ;
        double factor ;
    public:
        Algorithm ( const  DAG & dag,const WCET & wect,const std::vector<Process> &p):dag(dag),wect(wect),procs(p){
            energy_given=0;
            reliability = -1;
            goal_reliability = 0;
            total_energy= 0;
            factor = 0;
        }

        double get_goal_reliability ()const{
            return goal_reliability ;
        }

        void init_process(const std::string &proc_file){
            procs=init_proc_from_file(proc_file);
        }
        virtual void compute()=0;
        void set_energy(double energy){
            if(energy>0)
                energy_given=energy;
        }

        void set_factor(double fac){
            factor =fac;
        }
        double get_factor()const{
            return factor;
        }

        void set_goal_reliability(double goal){
            goal_reliability=goal ;
        }

        virtual double get_reliabilitys()const{

            double multi=1;
            for(auto &item:result){
                if(item.get_reliability()<1e-5)
                    continue;
                multi *=item.get_reliability();
            }
            return multi ;
        }

        double get_total_energy_consume(){
            if(total_energy>0)
                return total_energy;
            double sum=0;
            for(auto &item:result)
                sum+=item.get_energy_consume();
            total_energy=sum;
            return sum ;
        }

        virtual double get_schedule_length()const{
            double len=-1;
            for(auto &item:result)
                if(item.get_aft()>len)
                    len=item.get_aft();
            return len;
        }

        virtual void print() {
            for(auto & task : result){
                std::cout<<"task id:"<<task.get_task_id()+1<< "\t\test:"<<task.get_est()<<"\t\taft:"<<task.get_aft()<<"\t\tenergy consume:"
                        <<task.get_energy_consume()<<"\t\tproc id:"<<task.get_running_proc_id()+1
                        <<"\t\tfreq:"<<task.get_running_freq()<<" reliability:"<<task.get_reliability()<<std::endl;
            }
        }
};

#endif