#include "heft.hpp"

void Heft::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    std::vector<double> procs_used_time(procs.size(),0);
    for(int i=0;i<task_queue.size();++i){
        Task & t =task_queue[i];
        Task & original=dag.get_task_list()[t.get_task_id()];
        double aft = std::numeric_limits<double>::max();
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            double eft=0;
            double est=0;
            if(i==0){ // root node
                t.set_est(0);
                original.set_est(0);
                eft=get_earliest_finish_tm(t.get_est(),t,*proc,wect,proc->get_f_max());
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
                eft=get_earliest_finish_tm(est,t,*proc,wect,proc->get_f_max());
            }
            if(eft<aft){
                original.set_proc_id(proc->proc_id);
                original.set_running_freq(proc->get_f_max());
                original.set_aft(eft);
                original.set_est(est);

                t.set_proc_id(proc->proc_id);
                t.set_running_freq(proc->get_f_max());
                t.set_aft(eft);
                t.set_est(est);
                aft=eft;
            }
        }
        procs_used_time[t.get_running_proc_id()] = t.get_aft();
    }
    result = std::move(task_queue);
}