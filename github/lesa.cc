/*
    c++ implement of  List-based Energy-Aware Scheduling Algorithm(LESA) algorithm

    @ref` Energy-aware scheduling for dependent tasks in heterogeneous
          ultiprocessor systems `

    author : dk deng
    mail   : goodmandengdekang@gmail.com         
    Date   : 2022/8/2
*/

#include "lesa.hpp"
void Lesa::compute(){
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    double energy_left=energy_given ;
    std::vector<double> procs_used_time(procs.size(),0);
    for(int i=0;i<task_queue.size();++i){
        double e_weight=get_task_energy_consume_weight(task_queue.begin()+i,task_queue.end());
        double e_limite=0; // need more detail
        Task & t=task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double eft_min=std::numeric_limits<double>::max();
        for(auto proc=procs.begin();proc!=procs.end();++proc) {
            double eft=0;
            double est=0;
            double e_use ;
            double exec_min= std::numeric_limits<double>::max();
            double exec=0;
            int proc_index=proc-procs.begin();
            for(auto freq=proc->get_frequencys().begin();freq!=proc->get_frequencys().end();++freq){
                e_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                if(e_use>e_limite)
                    break;  // freq from low to max.
                exec=wect.get_wcet_info(t.get_task_id(),proc_index)*( proc->get_f_max() )/(*freq);
                if(exec>exec_min)
                    continue;    
                exec_min=exec;
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
                if(eft<eft_min){
                    eft_min=eft;
                    original.set_proc_id(proc->proc_id);
                    original.set_running_freq(*freq);
                    original.set_energy_consume(e_use);
                    original.set_aft(eft);
                    original.set_est(est);

                    t.set_proc_id(proc->proc_id);
                    t.set_running_freq(*freq);
                    t.set_energy_consume(e_use);
                    t.set_aft(eft);
                    t.set_est(est);
                }
            }
        }
        energy_left -= original.get_energy_consume();
        procs_used_time[t.get_running_proc_id()] =t.get_aft();
        if(energy_left<0)
            throw std::runtime_error("schedule can't execute");
    }
    result=std::move(task_queue);
}

double Lesa::get_procs_avg_energy_consume(const Task &t)const{
    double avg_energy=0;
    for(auto proc=procs.begin();proc!=procs.end();++proc){
        double sum=0;
        int proc_index=proc-procs.begin();
        for(auto freq=proc->get_frequencys().begin();freq!=proc->get_frequencys().end();++freq){
            sum+= get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
        }
        avg_energy+=sum / proc->get_frequencys().size();
    }
    avg_energy=avg_energy / procs.size();
    return avg_energy ;
}


double Lesa::get_task_energy_consume_weight(Iter beg,Iter end )const{
    double cur=this->get_procs_avg_energy_consume(*beg);
    ++beg;
    double total=cur ;
    for(;beg!=end;++beg)
        total+=this->get_procs_avg_energy_consume(*beg);
    return cur / total;
}