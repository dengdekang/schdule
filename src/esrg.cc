/*
    c++ implement of An energy-efficient scheduling with a reliability goal
    (ESRG) algorithm 

    @ref `Energy-Efficient Fault-Tolerant Scheduling of Reliable Parallel Applications on Heterogeneous
          Distributed Embedded Systems`

    author : dk deng
    mail   : goodmandengdekang@gmail.com         
    Date   : 2022/7/29
*/

#include "esrg.hpp"
#include "schedule_base.hpp"
#include <numeric>


void Esrg::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    if(task_queue.back().get_avg_weigth()==0)
        task_queue.pop_back();
    std::vector<double> procs_used_time(procs.size(),0);
    std::vector<double>pre_relia_goal;
    int size=task_queue.size();
    double n_sqrt_relia=std::pow(goal_reliability , double(1)/size); 
    for(int i=0;i<task_queue.size();++i){
        Task &t=task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        if(std::abs(t.get_avg_weigth())<1e-5)
            continue;
        double aft = std::numeric_limits<double>::max();
        double cur_goal =0;
        if(i==0){
            cur_goal = n_sqrt_relia ;
        } else{
            double phase1=1;
            for(auto pos=pre_relia_goal.begin();pos!=pre_relia_goal.end();++pos){
                phase1 *=*pos ;
            }
            double phase2 = std::pow(n_sqrt_relia,task_queue.size()-i-1);
            cur_goal = goal_reliability/( phase1* phase2 );
        }
        bool exit_flag=false ;
        double e_min=std::numeric_limits<double>::max();
        double relia_goal= -1 ;
        while(!exit_flag){
            for(auto proc=procs.begin();proc!=procs.end();++proc){
                int proc_index=proc-procs.begin();
                for(auto freq=proc->get_frequencys().begin();freq!=proc->get_frequencys().end();++freq){
                    double relia=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(t.get_task_id(),proc_index),proc->constant_fac,proc->lambda_max);
                    double eft=0;
                    double est=0;
                    if(relia > cur_goal){
                        double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                        if(energy_use<e_min){
                            e_min = energy_use ;
                            relia_goal = relia ;
                            exit_flag = true ;
                        }else{
                            continue;
                        }
                        if(i==0){ // root node
                            t.set_est(0);
                            original.set_est(0);
                            eft=get_earliest_finish_tm(t.get_est(),t,*proc,wect,*freq);
                        }else{      
                            double max_parent_aft=std::numeric_limits<double>::min();
                            for(auto pos=t.get_parent().begin();pos!=t.get_parent().end();++pos){
                                //int index=pos-t.get_parent().begin();
                                Task &parent_task=dag.get_task_list()[*pos];
                                double tmp_aft=parent_task.get_aft();
                                double commu_tm=0;
                                if(parent_task.get_running_proc_id()==proc_index)
                                    commu_tm =0;
                                else
                                    commu_tm= dag.get_edge_weight(parent_task.get_task_id(),t.get_task_id());
                                if(commu_tm+tmp_aft>max_parent_aft)
                                    max_parent_aft = commu_tm+tmp_aft ;
                            }
                            est=std::max(procs_used_time[proc_index], max_parent_aft);
                            eft=get_earliest_finish_tm(est,t,*proc,wect,*freq);
                        }
                        if(eft<aft){
                            original.set_proc_id(proc->proc_id);
                            original.set_running_freq(*freq);
                            original.set_energy_consume(energy_use);
                            original.set_aft(eft);
                            original.set_est(est);

                            t.set_proc_id(proc->proc_id);
                            t.set_running_freq(*freq);
                            t.set_energy_consume(energy_use);
                            t.set_aft(eft);
                            t.set_est(est);
                            aft=eft;
                        }
                        procs_used_time[t.get_running_proc_id()] = t.get_aft();
                        break;
                    }
                }
            }
            if(!exit_flag)
                throw std::runtime_error("process can't satisfy the goal");
        }
        if(relia_goal < 0)
            throw std::runtime_error("calculate error");
        pre_relia_goal.push_back(relia_goal);
        t.set_reliabiity(relia_goal);
        t.set_energy_consume(e_min);
    }
    result = std::move(task_queue);
}
