#include "schedule_base.hpp"
#include <numeric>
#include "derg.hpp"


void Derg::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    if(task_queue.back().get_avg_weigth()==0)
        task_queue.pop_back();
    double avg_wect_total=0;
    for(int i=0;i<task_queue.size();++i){
        auto list=wect.get_task_wect(task_queue[i].get_task_id());
        avg_wect_total += std::accumulate(list.begin(),list.end(),0.0)/list.size();
    }

    double max_reliability=1;
    std::vector<double> r_maxs(task_queue.size());
    for(auto pos=0;pos<task_queue.size();++pos){
        double r_max =-std::numeric_limits<double>::max();;
        double relia ;
        for(auto &proc:procs){
            relia=get_raliability(proc.get_f_max(),proc.get_f_ee(),proc.get_f_max(),wect.get_wcet_info(task_queue[pos].get_task_id(),proc.proc_id),proc.constant_fac,proc.lambda_max);
            if(relia>r_max)
                r_max = relia ;
        }
        r_maxs[pos]=r_max;
    }
    double max_reliability_G=1;
    for(auto &item:r_maxs)
        max_reliability_G *= item ;

    double rgr_g=get_goal_reliability()/max_reliability_G ;
    std::vector<double> r_pres(task_queue.size());
    for(int i=0;i<task_queue.size();++i){
        auto list=wect.get_task_wect(task_queue[i].get_task_id());
        double avg_wect=std::accumulate(list.begin(),list.end(),0.0)/list.size();
        r_pres[i]=r_maxs[i]* std::pow( rgr_g,avg_wect/avg_wect_total);
    }
    double r_assign=1;
    std::vector<double> procs_used_time(procs.size(),0);
    for(int i=0;i<task_queue.size();++i){
        Task & t= task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double energy=std::numeric_limits<double>::max();
        double remain=1;
        double aft=std::numeric_limits<double>::max();
        for(int n=i+1;n<r_pres.size();n++)
            remain *= r_pres[n];
        double r_goal= get_goal_reliability()/(remain * r_assign) ;
        double r_use = 0;
        double e_min=std::numeric_limits<double>::max();
        double relia_goal= -1 ;
        double energy_use=0  ;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->proc_frequencys.begin();freq!=proc->proc_frequencys.end();++freq){
                r_use=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(task_queue[i].get_task_id(),proc->proc_id),proc->constant_fac,proc->lambda_max);
                if( r_use < r_goal )
                    continue;
                energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                if(energy_use<e_min){
                    e_min = energy_use ;
                    relia_goal = r_use ;
                }else{
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
                t.set_proc_id(proc->proc_id);
                t.set_running_freq(*freq);
                t.set_energy_consume(energy_use);
                t.set_aft(eft);
                t.set_est(est);
                //t.set_reliabiity(std::pow(2,relia_goal/get_factor()));
                t.set_reliabiity(relia_goal);

                original.set_reliabiity(relia_goal);
                original.set_proc_id(proc->proc_id);
                original.set_running_freq(*freq);
                original.set_energy_consume(energy_use);
                original.set_aft(eft);
                original.set_est(est);
                aft=eft ;
            }
        }
        r_assign *= relia_goal ;
        procs_used_time.at(t.get_running_proc_id()) =t.get_aft();
    }
    result = std::move(task_queue);
}