/*
    c++ implement of minimum schedule length with energy consumption
    constraint(MSLECC) algorithm

    @ref `Minimizing Schedule Length of Energy Consumption Constrained Parallel Applications on
          Heterogeneous Distributed Systems `

    author : dk deng
    mail   : goodmandengdekang@gmail.com         
    Date   : 2022/7/27
*/

#include "schedule_base.hpp"
#include "mslecc.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <limits>

void Mslecc::compute(){
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    double energy_left=energy_given ;
    double e_given_task=0;
    std::vector<double> procs_used_time(procs.size(),0);
    for(auto &item:task_queue)
        item.compute_min_and_max_energy_consume(procs,wect);
    for(int i=0;i<task_queue.size();++i){
        Task & t =task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double aft = std::numeric_limits<double>::max();
        double e_min=get_min_or_max_energy_consume(t,procs,wect,false);
        double e_max=get_min_or_max_energy_consume(t,procs,wect,true);
        double e_given_left_task=get_range_sum_min_nenrgy_comsume(task_queue.begin()+1+i,task_queue.end(),procs,wect);
        e_given_task=energy_left-e_given_left_task ;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->proc_frequencys.begin();freq!=proc->proc_frequencys.end();++freq){
                double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                if(energy_use> std::min(e_given_task,e_max) ){
                    continue;
                }
                double eft=0;
                double est=0;
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
            }
        }
        energy_left -= t.get_energy_consume() ;
        procs_used_time[t.get_running_proc_id()] = t.get_aft();
        if(energy_left<0)
            throw std::runtime_error("schedule can't execute");
    }
    result = std::move(task_queue);
}
