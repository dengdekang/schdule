#include "efsrg.hpp"
#include <numeric>
#include <cmath>

#include <utility>


struct tmp_data{
    double relia ;
    double enery ;
    int proc_id;
    double freq ;
    double spend_time ;
    tmp_data(double r,double e,int id,double f,double sp_tm):relia(r),enery(e),proc_id(id),freq(f),spend_time(sp_tm){

    }

    bool operator<(const tmp_data & d){
        return enery< d.enery ;
    }
};

void Efsrg::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    if(task_queue.back().get_avg_weigth()==0) //for multi exit
        task_queue.pop_back();
    std::vector<double> procs_used_time(procs.size(),0);
    std::vector<double>pre_relia_goal;
    double n_sqrt_relia=std::pow(goal_reliability , double(1)/task_queue.size());
    double total_energy=0;
    std::vector<int> pre_proces;
    double max_sl=0;
    for(int i=0;i<task_queue.size();++i){
        Task &t=task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double aft = std::numeric_limits<double>::max();
        double cur_goal =0;
        if(i==0){
            cur_goal = n_sqrt_relia ;
            t.set_est(0);
        } else{
            double phase1=1;
            for(auto pos=pre_relia_goal.begin();pos!=pre_relia_goal.end();++pos){
                phase1 *=*pos ;
            }
            double phase2 = std::pow(n_sqrt_relia,task_queue.size()-i-1);
            cur_goal = goal_reliability/( phase1* phase2 );
        }
        std::vector< tmp_data > datas;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->get_frequencys().cbegin();freq!=proc->get_frequencys().cend();++freq){
                double relia=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(t.get_task_id(),proc_index),proc->constant_fac,proc->lambda_max);
                double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                
                double spend_tm= proc->get_f_max()*wect.get_wcet_info(t.get_task_id(),proc_index)/(*freq); 
                datas.push_back( tmp_data (relia,energy_use,proc_index,*freq,spend_tm ) );
            }
        }
        std::sort(datas.begin(),datas.end());
        std::vector< tmp_data > act_assi_list;
        double relia= 0;
        double use_energy=0;
        while(relia < cur_goal){
            for(auto &item : datas){
                for(auto pos=act_assi_list.begin();pos!=act_assi_list.end();)
                    if(pos->proc_id == item.proc_id){
                        pos=act_assi_list.erase(pos);
                    }else
                        ++pos;
                act_assi_list.push_back(item);
                double fault=1;
                for(auto &act_item:act_assi_list)
                    fault *= (1-act_item.relia);
                relia = 1-fault ;
                if(relia < cur_goal)
                    continue;
                use_energy=0;
                pre_relia_goal.push_back(relia);
                for(auto &act_item:act_assi_list)
                    use_energy+=act_item.enery ;
                break;
            }
        }
        original.set_reliabiity(relia);
        original.set_energy_consume(use_energy);
        t.set_reliabiity(relia);
        t.set_energy_consume(use_energy);
        total_energy+= use_energy ;
        
        std::vector<double> proces_aft_local(procs.size(),0);
        for(auto &act_item:act_assi_list)
            proces_aft_local[act_item.proc_id]=act_item.spend_time ;
        if(i==0){
            t.set_procs_aft(proces_aft_local);
            original.set_procs_aft(proces_aft_local);
        }else{
            double max_parent_aft=std::numeric_limits<double>::min();
            for(auto pos=t.get_parent().begin();pos!=t.get_parent().end();++pos){
                Task &parent_task=dag.get_task_list()[*pos];
                double max_parent_aft=std::numeric_limits<double>::min();
                int id=0;
                for(int pos=0;pos<parent_task.get_procs_aft().size();pos++){
                    if(max_parent_aft<parent_task.get_procs_aft()[pos]){
                        id=pos;
                        max_parent_aft=parent_task.get_procs_aft()[pos];
                    }
                }
                
                for(auto &act_item:act_assi_list){
                    double comm=dag.get_edge_weight(parent_task.get_task_id(),t.get_task_id());
                    if(act_item.proc_id==id){
                        comm= 0;
                    }
                    //max_parent_aft=parent_task.get_procs_aft()[act_item.proc_id];
                    proces_aft_local[act_item.proc_id]= std::max(max_parent_aft+act_item.spend_time+comm,proces_aft_local[act_item.proc_id]);
                }
            }
            t.set_procs_aft(proces_aft_local);
            original.set_procs_aft(proces_aft_local);
        }

        // if(i==0){
        //     t.set_procs_aft(proces_aft_local);
        //     original.set_procs_aft(proces_aft_local);
        // }else{
        //     double max_parent_aft=std::numeric_limits<double>::min();
        //     for(auto pos=t.get_parent().begin();pos!=t.get_parent().end();++pos){
        //         Task &parent_task=dag.get_task_list()[*pos];
        //         double max_parent_aft=std::numeric_limits<double>::min();
        //         for(auto &aft_item:parent_task.get_procs_aft())
        //             max_parent_aft=std::max(aft_item,max_parent_aft);
        //         for(auto &act_item:act_assi_list){
        //             double comm=0;
        //             if(parent_task.get_procs_aft().at(act_item.proc_id)==0){
        //                 comm= dag.get_edge_weight(parent_task.get_task_id(),t.get_task_id());
        //             }else{
        //                 max_parent_aft=parent_task.get_procs_aft()[act_item.proc_id];
        //             }
        //             proces_aft_local[act_item.proc_id]= std::max(max_parent_aft+act_item.spend_time+comm,proces_aft_local[act_item.proc_id]);
        //         }
        //     }
        //     t.set_procs_aft(proces_aft_local);
        //     original.set_procs_aft(proces_aft_local);
        // }
    }
    result=std::move(task_queue);
}