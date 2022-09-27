/*
    c++ implement of improved scheduling approach for energy
    consumption constrained(ISAECC) algorithm

    @ref `Task Scheduling for Energy Consumption Constrained Parallel Applications on
          Heterogeneous Computing Systems`

    author : dk deng
    mail   : goodmandengdekang@gmail.com         
    Date   : 2022/7/29
*/

#include "isaecc.hpp"
#include "schedule_base.hpp"
#include <numeric>
void Isaecc::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    double energy_left=energy_given ;
    std::vector<double> e_mins(task_queue.size());
    std::vector<double> e_maxs(task_queue.size());
    std::vector<double> e_pres(task_queue.size());
    std::vector<double> procs_used_time(procs.size(),0);
    for(auto pos=0;pos<task_queue.size();++pos){
        double e_max=get_min_or_max_energy_consume(task_queue[pos],procs,wect,true);
        e_maxs[pos]=e_max;
        double e_min=get_min_or_max_energy_consume(task_queue[pos],procs,wect,false);
        e_mins[pos]=e_min;
    }
    double e_min_graph=std::accumulate(e_mins.begin(),e_mins.end(),0.0);
    double e_max_graph=std::accumulate(e_maxs.begin(),e_maxs.end(),0.0);
    double e_avg_graph=(e_max_graph+e_min_graph)/2;
    double e_ie_graph=energy_given-e_min_graph ;
    for(int index=0;index<task_queue.size();++index){
        double el=(e_mins[index]+e_maxs[index])/2/e_avg_graph;
        double wa=e_ie_graph*el+e_mins[index];
        double e_pre=std::min(wa , e_maxs[index] );
        e_pres[index]=e_pre;
    }
    for(int i=0;i<task_queue.size();++i){
        Task & t= task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double aft=std::numeric_limits<double>::max();
        double e_given=energy_left -std::accumulate(e_pres.begin()+i+1,e_pres.end(),0.0);
        double energy_ues=0;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->proc_frequencys.begin();freq!=proc->proc_frequencys.end();++freq){
                energy_ues=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                if(energy_ues>e_given)
                    continue;
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
                    t.set_proc_id(proc->proc_id);
                    t.set_running_freq(*freq);
                    t.set_energy_consume(energy_ues);
                    t.set_aft(eft);
                    t.set_est(est);

                    original.set_proc_id(proc->proc_id);
                    original.set_running_freq(*freq);
                    original.set_energy_consume(energy_ues);
                    original.set_aft(eft);
                    original.set_est(est);
                    aft=eft ;
                }
            }
        }
        energy_left -= original.get_energy_consume();
        procs_used_time[t.get_running_proc_id()] =t.get_aft();
        if(energy_left<0)
            throw std::runtime_error("schedule can't execute");
    }
    result = std::move(task_queue);
}