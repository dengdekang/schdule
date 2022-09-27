#include "sarc.hpp"
#include <numeric>
#include <cmath>
#include <unordered_map>


bool comp(const  Task &t1,const  Task &t2)
{
    if(std::abs(t1.get_avg_weigth()-t2.get_avg_weigth())>1e-5){
        return t1.get_avg_weigth()>t2.get_avg_weigth();
    }else{
        return t1.get_task_id()<t2.get_task_id();
    }

}

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

void Sarc::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::vector<Task> task_weight=dag.get_task_list();
    std::sort(task_weight.begin(),task_weight.end(),comp);
    std::sort(task_queue.begin(),task_queue.end());
    std::vector<double> procs_used_time(procs.size(),0);
    std::vector<double> pre_relia_goal;
    if(task_queue.back().get_avg_weigth()==0)
        task_queue.pop_back();
    if(task_weight.back().get_avg_weigth()==0)
        task_weight.pop_back();
    double k= get_factor(); // factor 
    std::vector<double> r_maxs(task_queue.size());
    std::vector<double> r_pres(task_queue.size());
    for(auto pos=0;pos<task_weight.size();++pos){
        double r_max =-std::numeric_limits<double>::max();
        double relia ;
        for(auto &proc:procs){
            relia=get_raliability(proc.get_f_max(),proc.get_f_ee(),proc.get_f_max(),wect.get_wcet_info(task_weight[pos].get_task_id(),proc.proc_id),proc.constant_fac,proc.lambda_max);
            relia = k* std::log2( relia );
            if(relia>r_max)
                r_max = relia ;
        }
        r_maxs[pos]=r_max;
    }
    double r_max_graph=std::accumulate(r_maxs.begin(),r_maxs.end(),0.0);
    double r_goal= k* std::log2(goal_reliability) ;
    double r_gap=r_max_graph - r_goal ;
    double cal_total=0;
    for(int i=0;i<task_weight.size();++i){
        auto list=wect.get_task_wect(task_weight[i].get_task_id());
        cal_total += std::accumulate(list.begin(),list.end(),0.0)/list.size();
    }
    for(int i=0;i<task_weight.size();++i){
        auto list=wect.get_task_wect(task_weight[i].get_task_id());
        double el= std::accumulate(list.begin(),list.end(),0.0)/list.size();
        r_pres[i]=r_maxs[i]-r_gap*el/cal_total ;
    }

    double r_pre_total= std::accumulate(r_pres.begin(),r_pres.end(),0.0);
    double r_left= 0;
    std::vector<std::pair<Task,Task>> tasks_pair;
    for(int i=0,n=task_weight.size()-1;i<=n;i++,n--){
        Task &t1=task_weight[i];
        Task &t2=task_weight[n];
        tasks_pair.push_back({ t1,t2 });
    }
    int i=0;
    int n=task_weight.size()-1;
    double left=0;
    double relia=0;
    std::unordered_map<int,Task> task_result ;  
    for (auto & task_pair :tasks_pair){
        std::vector< tmp_data > high_datas;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->get_frequencys().cbegin();freq!=proc->get_frequencys().cend();++freq){
                double relia=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(task_pair.first.get_task_id(),proc_index),proc->constant_fac,proc->lambda_max);
                double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(task_pair.first.get_task_id(),proc_index),*freq);                
                double spend_tm= proc->get_f_max()*wect.get_wcet_info(task_pair.first.get_task_id(),proc_index)/(*freq); 
                high_datas.push_back( tmp_data (relia,energy_use,proc_index,*freq,spend_tm ) );
            }
        }
        std::vector< tmp_data > low_datas;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->get_frequencys().cbegin();freq!=proc->get_frequencys().cend();++freq){
                double relia=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(task_pair.second.get_task_id(),proc_index),proc->constant_fac,proc->lambda_max);
                double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(task_pair.second.get_task_id(),proc_index),*freq);
                
                double spend_tm= proc->get_f_max()*wect.get_wcet_info(task_pair.second.get_task_id(),proc_index)/(*freq); 
                low_datas.push_back( tmp_data (relia,energy_use,proc_index,*freq,spend_tm ) );
            }
        }
        //std::sort(high_datas.begin(),high_datas.end());
        //std::sort(low_datas.begin(),low_datas.end(),[](const tmp_data & d1,const tmp_data & d2){return d1.relia>d2.relia;});
        relia=r_pres[i++]+r_pres[n--]-left;
        double energy=std::numeric_limits<double>::max();
        for(auto &low_item:low_datas ){
            double l_relia=k* std::log2(low_item.relia);
            // if(task_pair.first.get_task_id()==task_pair.second.get_task_id()){
            //     if(low_item.relia>relia){
            //         if(low_item.enery<energy){
            //             energy=low_item.enery;
            //             task_pair.first.set_energy_consume(low_item.enery);
            //             task_pair.first.set_proc_id(low_item.proc_id);
            //             task_pair.first.set_running_freq(low_item.freq);
            //             task_pair.first.set_reliabiity( low_item.relia);
            //             task_result.insert({task_pair.first.get_task_id(),task_pair.first});
            //         }
            //     }
            //     continue;
            // }
            for(auto &high_item:high_datas){
                double h_relia=k* std::log2(high_item.relia);
                if(h_relia+l_relia>relia){
                    if(energy>(low_item.enery+high_item.enery)){
                        energy=low_item.enery+high_item.enery;
                        task_pair.first.set_energy_consume(low_item.enery);
                        task_pair.first.set_proc_id(low_item.proc_id);
                        task_pair.first.set_running_freq(low_item.freq);
                        task_pair.first.set_reliabiity( l_relia);
                        task_pair.first.set_aft(low_item.spend_time);

                        task_pair.second.set_aft(high_item.spend_time);
                        task_pair.second.set_reliabiity(h_relia);
                        task_pair.second.set_energy_consume(high_item.enery);
                        task_pair.second.set_proc_id(high_item.proc_id);
                        task_pair.second.set_running_freq(high_item.freq);
                        left=h_relia+l_relia-relia;
                        task_result[task_pair.first.get_task_id()]=task_pair.first;
                        task_result[task_pair.second.get_task_id()]=task_pair.second;
                    //    task_result.insert({task_pair.first.get_task_id(),task_pair.first});
                      //  task_result.insert({ task_pair.second.get_task_id(),task_pair.second});        
                    }
                }
            }
        }
    }
    double total_energy=0;
    double relia_all=0;
    for(auto &item:tasks_pair){
        if(item.first.get_task_id()==item.second.get_task_id()){
            total_energy+= item.first.get_energy_consume();
            relia_all+= item.first.get_reliability();
        }
        else{
            total_energy+= item.first.get_energy_consume()+ item.second.get_energy_consume();
            relia_all+= item.first.get_reliability()+item.second.get_reliability();
        }
    }
    for(int i=0;i<task_queue.size();i++){
        Task & t=task_queue[i];
        Task & t1=task_result.at(t.get_task_id());
        Task & original=dag.get_task_list()[t.get_task_id()];
        double eft=0;
        double est=0;
        if(i==0){
            procs_used_time.at(t1.get_running_proc_id())=t1.get_aft();

            original.set_est(0);
            original.set_proc_id(t1.get_running_proc_id());
            original.set_running_freq(t1.get_running_freq());
            original.set_aft( get_earliest_finish_tm(original.get_est(),original, procs[ t1.get_running_proc_id()],wect,t1.get_running_freq()));
            original.set_reliabiity(t1.get_reliability());
            original.set_energy_consume(t1.get_energy_consume());

            t.set_reliabiity(t1.get_reliability());
            t.set_energy_consume(t1.get_energy_consume());
            t.set_est(0);
            t.set_proc_id(t1.get_running_proc_id());
            t.set_running_freq(t1.get_running_freq());
            t.set_aft( get_earliest_finish_tm(t.get_est(),t, procs[ t1.get_running_proc_id()],wect,t1.get_running_freq()));
            int m=0;
        }else{      
            double max_parent_aft=std::numeric_limits<double>::min();
            for(auto pos=t.get_parent().begin();pos!=t.get_parent().end();++pos){
                Task &parent_task=dag.get_task_list()[*pos];
                double tmp_aft=parent_task.get_aft();
                double commu_tm=0;
                if(parent_task.get_running_proc_id()==t1.get_running_proc_id())
                    commu_tm =0;
                else
                    commu_tm= dag.get_edge_weight(parent_task.get_task_id(),t.get_task_id());
                if(commu_tm+tmp_aft>max_parent_aft)
                    max_parent_aft = commu_tm+tmp_aft ;
            }
            est=std::max(procs_used_time[t1.get_running_proc_id()], max_parent_aft);
            eft=get_earliest_finish_tm(est,t1,procs[ t1.get_running_proc_id()],wect,t1.get_running_freq());
            t.set_proc_id(t1.get_running_proc_id());
            t.set_running_freq(t1.get_running_freq());
            t.set_est(est);
            t.set_aft(eft);
            t.set_reliabiity(t1.get_reliability());
            t.set_energy_consume(t1.get_energy_consume());

            original.set_proc_id(t1.get_running_proc_id());
            original.set_running_freq(t1.get_running_freq());
            original.set_est(est);
            original.set_aft(eft);
            original.set_reliabiity(t1.get_reliability());
            original.set_energy_consume(t1.get_energy_consume());
        }
        procs_used_time.at(t1.get_running_proc_id()) =t.get_aft(); 
    }
    result= std::move(task_queue);

}