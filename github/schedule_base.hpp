#ifndef __SCHEDULE_BASE
#define __SCHEDULE_BASE
#include <vector>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>
struct  Edge {
        int start_node;
        int end_node;
        double weight;
        Edge(int s,int e,double w=0):start_node(s),end_node(e),weight(w){
        }

        void set_weight(double w){
            if(w>=0)
                weight =w ;
        }

        int get_end_note()const{
            return end_node;
        }

        double get_weight()const{
            return weight ;
        }
};

class Process ;
class WCET ;
class Task{
    int task_id ;  // task reprensent a node
    double mpis ;
    int running_proc_id ;
    std::vector<int> parent;
    double est ;  // earliest start time 
    double aft;    // actual finish time
    double up_rank ; // The upward rank value
    double down_rank ;
    double running_freq ;
    double energy_consume;
    double e_min ;
    double e_max ;
    double reliability ;
    double avg_weight ;
    std::vector<double> afts;// for duplication ;
    std::vector<int>running_proces ; //for duplication
    public:
        Task():Task(-1,-1){}

        Task(int id,double m):task_id(id),mpis(m),running_proc_id(-1),up_rank(-1){
            parent={};
            energy_consume=0;
            running_freq = 0;
            aft=0;
            e_min =0 ;
            e_max = 0;
            reliability =0;
        }
        Task(const Task&)=default;
        Task & operator=(const Task& t)=default;

        void set_reliabiity(double r){
            reliability =r ;
        }
        double get_reliability()const{
            return reliability;
        }

        void compute_min_and_max_energy_consume(const std::vector<Process> &procs,const WCET &wect);
        void set_min_energy_consume(double energy){
            e_min = energy;
        }
        double get_min_nergy_consume()const{
            return e_min ;
        }
        void set_max_energy_consume(double energy){
            e_max = energy;
        }
        double get_max_nergy_consume()const{
            return e_max ;
        }
        int get_running_proc_id()const{
            return running_proc_id ;
        }
        void set_est(double earl_start_tm){
            est=earl_start_tm ;
        }
        double get_est()const{
            return est;
        }
        void set_aft(double t){
            aft =t ;
        }

        double get_aft()const{
            return aft;
        }

        void set_energy_consume(double energy){
            energy_consume=energy;
        }
        double get_energy_consume()const{
            return energy_consume;
        }
        
        void set_avg_weight(double weight){
            avg_weight=weight ;
        }

        double get_avg_weigth()const{
            return avg_weight ;
        }

        const std::vector<int> &get_running_proces()const{
            return running_proces;
        }

        void set_running_proces(const std::vector<int> &procs){
            running_proces=procs ;
        }

        const std::vector<double> &get_procs_aft()const{
            return afts;
        }

        void set_procs_aft(const std::vector<double> &procs_aft){
            afts=procs_aft ;
        }
        
        void set_running_freq(double freq){
            running_freq=freq;
        }
        double get_running_freq()const{
            return running_freq;
        }

        void set_proc_id(int id){
            
            running_proc_id = id ;
            
        }
        void set_up_rank(double up_rank){
            if(up_rank>=0)
                this->up_rank=up_rank;
        }
        void set_down_rank(double down_rank){
            if(down_rank>=0)
                this->down_rank=down_rank;
        }
        double get_up_rank()const{
            return up_rank;
        }
        const std::vector<int> &get_parent()const{
            return parent ;
        }
        double get_down_rank()const{
            return down_rank;
        }
        void set_parent_list(const std::vector<int> & node){
            parent=node ;
        }
        void add_parent(int node){
            parent.push_back(node);
        }
        int get_task_id()const{
            return task_id;
        }
        void assign_proc(int proc_id){
            running_proc_id = proc_id ;
        }
        bool operator<(const Task &t){
            if(std::abs(up_rank-t.up_rank)>1e-5){
                return up_rank>t.up_rank ;
            }else{
                return task_id<t.task_id;
            }
        }

};

class WCET ;
class Process;
class DAG {
    private:
        std::vector < std::vector <Edge> > graph;
        std::vector < Task > task_list ; // assume that node 0 is root 
    public:
        DAG (const std::string & edge_file,const std::string &weight_file) ;
        void compute_up_rank(const WCET &);
        void compute_down_rank(const WCET & );
        void init_parent(const std::string &);
        const std::vector<Task> &get_task_list()const{
            return task_list;
        }
        std::vector<Task> &get_task_list(){
            return task_list;
        }
        void compute_parent();
        double get_edge_weight(int parent,int child);
    protected :
        
        void init_dag(const std::string &edge_file,const std::string &weight_file) ;
};
//get energy-efficient frequency
double get_fee(double p_ind,double m,double c_ef);
double get_energy_consume(double p_ind,double c_ef,double mk,double f_max,double w_ik,double f_h);
double get_raliability(double f_max,double f_min, double f_kv,double w_ik,double d_constant ,double lamda_k_max);
double get_min_or_max_energy_consume(const Task &t,const std::vector<Process> &procs,const WCET &wect,bool is_max);

double get_range_sum_min_nenrgy_comsume(std::vector<Task>::iterator beg, std::vector<Task>::iterator end,
                                        const std::vector<Process> &procs,const WCET &wect);

double get_earliest_finish_tm(double est ,const Task &t,const Process&proc,const WCET&wect,double f_h);

class Process{
    public:
        std::vector<double> proc_frequencys ;
        int proc_id ;
        double p_s;   //static power is not consider in this project
        double p_ind;   //frequency-independent
        double c_ef ;   //effective capacitance
        double mk;      //dynamic power exponent
    
    private:
        int f_low ;  //
        int f_ee ;   //The minimum energy-efficient frequency
        int f_max;   // 
    public:

        int step ;   //the frequent  step 
        double current_freq ;
        double lambda_max  ;
        double constant_fac ;

    public:
        double get_f_max()const{
            return double(f_max)/100;
        }
        double get_f_ee()const{
            return double(f_ee)/100;
        }

        double get_power_static()const{
            return p_s ;
        }

        void set_power_static(double pow_static){
            p_s =pow_static ;
        }

        Process (int id, double p_ind, double c_ef,double mk,int f_low,int f_max,int step):Process(id,p_ind,c_ef,mk,f_low,f_max,step,1,1){

        }
        
        Process (int id, double p_ind, double c_ef,double mk,int f_low,int f_max,int step,double lambda_max,double fac=1):
            proc_id(id),p_ind(p_ind),c_ef(c_ef),mk(mk),f_low(f_low),f_max(f_max),step(step),lambda_max(lambda_max),constant_fac(fac){
                //f_ee=get_fee(p_ind,mk,c_ef);
                current_freq = f_low ; 
                p_s = 0 ;
                f_ee=f_low ; //we set f_ee equal f_low  
                if(f_low>=f_max)
                    throw std::runtime_error("freq low must less than f_max");
                for(int pos=f_low;pos<=f_max;pos+=step)
                    proc_frequencys.push_back(double(pos)/100);
        }


        double operator()(){
            double tmp_freq=current_freq;
            current_freq+=step ;
            return tmp_freq;
        }
        const std::vector<double> &get_frequencys()const{
            return proc_frequencys;
        }
} ;

// worst case execution time
//N task m process matrix
class WCET {
    private:
        //std::vector< std::vector< double> > task_proc_info  ; 
        std::vector<double> task_proc_info ; // if a  N x M matrix ,if Task N can't run in specific frequency ,it will set to -1 ;
        int n_task;
        int m_proc;
    public:

        double get_wcet_info(int row,int  col)const{
            if(row<0||col < 0 || row>=n_task || col >= m_proc)
                throw std::runtime_error("out of range");
            return task_proc_info[row*m_proc+col] ;
        }
        std::vector<double> get_task_wect(int task_id)const{
            std::vector<double> result;
            for(int col=0;col<m_proc;++col)
                result.push_back( get_wcet_info(task_id,col) );
            return result ;
        }

        WCET (const std::string &wect_file){
            init(wect_file);
        }
        int get_num_of_task()const{
            return n_task;
        }

        int get_num_of_process()const{
            return m_proc;
        }

    protected:
        void init(const std::string &wcet_file);
};

std::vector<Process> init_proc_from_file(const std::string &);

#endif
