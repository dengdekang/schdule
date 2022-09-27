#include "xxxx.hpp"
#include <numeric>
#include <cmath>

struct tmp_data_t{
    double relia ;
    double enery ;
    int proc_id;
    double freq ;
    double spend_time ;
    double f_se;
    tmp_data_t(double r,double e,int id,double f,double sp_tm,double f_se):relia(r),enery(e),proc_id(id),freq(f),spend_time(sp_tm),f_se(f_se){

    }

    bool operator<(const tmp_data_t & d){
        return f_se< d.f_se ;
    }
};



void XXXX::compute()
{
    dag.compute_up_rank(wect);
    std::vector<Task> task_queue=dag.get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    if(task_queue.back().get_avg_weigth()==0)
        task_queue.pop_back();
    std::vector<double> procs_used_time(procs.size(),0);
    std::vector<double> pre_relia_goal;
    
    double k= get_factor(); // factor 
    std::vector<double> r_maxs(task_queue.size());
    std::vector<double> r_pres(task_queue.size());
    for(auto pos=0;pos<task_queue.size();++pos){
        double r_max =-std::numeric_limits<double>::max();
        double relia =1;
        double tmp=1;
        for(auto &proc:procs){
            relia=get_raliability(proc.get_f_max(),proc.get_f_ee(),proc.get_f_max(),wect.get_wcet_info(task_queue[pos].get_task_id(),proc.proc_id),proc.constant_fac,proc.lambda_max);
            tmp*=(1-relia);
        }
        r_maxs[pos]=get_factor()*std::log(1-tmp);
    }
    double r_max_graph=std::accumulate(r_maxs.begin(),r_maxs.end(),0.0);
    double r_goal= k* std::log2(goal_reliability) ;
    double r_gap=r_max_graph - r_goal ;

    double cal_total=0;
    for(int i=0;i<task_queue.size();++i){
        auto list=wect.get_task_wect(task_queue[i].get_task_id());
        cal_total += std::accumulate(list.begin(),list.end(),0.0)/list.size();
    }
    for(int i=0;i<task_queue.size();++i){
        auto list=wect.get_task_wect(task_queue[i].get_task_id());
        double el= std::accumulate(list.begin(),list.end(),0.0)/list.size();
        r_pres[i]=r_maxs[i]-r_gap*el/cal_total ;
    }
    double r_left= 0;
    for(int  i=0 ;i<task_queue.size();++i){
        Task &t=task_queue[i];
        Task &original=dag.get_task_list()[t.get_task_id()];
        double aft = std::numeric_limits<double>::max();
        std::vector< tmp_data_t > datas;
        for(auto proc=procs.begin();proc!=procs.end();++proc){
            int proc_index=proc-procs.begin();
            for(auto freq=proc->get_frequencys().cbegin();freq!=proc->get_frequencys().cend();++freq){
                double relia=get_raliability(proc->get_f_max(),proc->get_f_ee(),*freq,wect.get_wcet_info(t.get_task_id(),proc_index),proc->constant_fac,proc->lambda_max);
                double energy_use=get_energy_consume(proc->p_ind,proc->c_ef,proc->mk,proc->get_f_max(),wect.get_wcet_info(t.get_task_id(),proc_index),*freq);
                
                double spend_tm= proc->get_f_max()*wect.get_wcet_info(t.get_task_id(),proc_index)/(*freq);
                double f_se= get_alpha()*energy_use*(1-get_alpha())*spend_tm+get_alpha()*energy_use+(1-get_alpha())*spend_tm;
                datas.push_back( tmp_data_t (relia,energy_use,proc_index,*freq,spend_tm ,f_se) );
            }
        }
        std::sort(datas.begin(),datas.end());
        
        double relia=r_pres[i]-r_left ;
        double cur_goal =-1;
        std::vector< tmp_data_t > final_assi_list;
        std::vector< tmp_data_t > act_assi_list;
        double max_use_energy=std::numeric_limits<double>::max();
        double use_energy=0;
        double debug_fault=1;
        bool flag=false;
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
            double goal=0;
            goal = get_factor() * std::log(1-fault);
            if(relia > goal)
                continue;
            flag=true;
            use_energy=0;
            for(auto &act_item:act_assi_list)
                use_energy+=act_item.enery ;
            if(use_energy<max_use_energy){
                debug_fault=fault;
                cur_goal=goal;
                max_use_energy=use_energy;
                final_assi_list=act_assi_list;
            }
        }
        if(flag==false){
            std::cout<<"i"<<i<<std::endl;
            throw std::runtime_error(" error");
        }

        r_left=cur_goal-relia;
        original.set_reliabiity(std::pow(2,cur_goal/get_factor()));
        original.set_energy_consume(use_energy);
        t.set_reliabiity(std::pow(2,cur_goal/get_factor()));
        t.set_energy_consume(use_energy);
        total_energy+= max_use_energy ;

        std::vector<double> proces_aft_local(procs.size(),0);
        for(auto &act_item:final_assi_list)
            proces_aft_local[act_item.proc_id]=act_item.spend_time ;
        if(i==0){
            t.set_procs_aft(proces_aft_local);
            original.set_procs_aft(proces_aft_local);
        }else{
            double max_parent_aft=std::numeric_limits<double>::min();
            for(auto pos=t.get_parent().begin();pos!=t.get_parent().end();++pos){
                Task &parent_task=dag.get_task_list()[*pos];
                double max_parent_aft=std::numeric_limits<double>::min();
                for(auto &aft_item:parent_task.get_procs_aft())
                    max_parent_aft=std::max(aft_item,max_parent_aft);
                for(auto &act_item:final_assi_list){
                    double comm=0;
                    if(parent_task.get_procs_aft().at(act_item.proc_id)==0){
                        comm= dag.get_edge_weight(parent_task.get_task_id(),t.get_task_id());
                    }else{
                        max_parent_aft=parent_task.get_procs_aft()[act_item.proc_id];
                    }
                    proces_aft_local[act_item.proc_id]= std::max(max_parent_aft+act_item.spend_time+comm,proces_aft_local[act_item.proc_id]);
                }
            }
            t.set_procs_aft(proces_aft_local);
            original.set_procs_aft(proces_aft_local);
        }
        pre_relia_goal.push_back(relia);
    }
    result=std::move(task_queue);
}