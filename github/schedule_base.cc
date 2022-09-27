#include "schedule_base.hpp"
#include <boost/algorithm/string.hpp>

void Task::compute_min_and_max_energy_consume(const std::vector<Process> &procs,const WCET &wect)
{
    e_min=get_min_or_max_energy_consume(*this,procs,wect,false);
    e_max=get_min_or_max_energy_consume(*this,procs,wect,true);
}

DAG::DAG(const std::string &edge_file,const std::string &weight_file)
{
    init_dag(edge_file,weight_file);

}

double DAG::get_edge_weight(int parent,int child)
{
    if(parent<0||parent>=graph.size())
        throw std::runtime_error("parent node not exist");
    for(auto pos=graph[parent].begin();pos!=graph[parent].end();++pos)
        if(pos->get_end_note()==child)
            return pos->get_weight();
    throw std::runtime_error(std::string("child:")+std::to_string(child)+" not belong to parent:" +std::to_string(parent));
}

void DAG::init_dag(const std::string &edge_file,const std::string &weight_file){
    std::ifstream in(edge_file);
    if(!in){
        std::cerr<<"file not found\n";
        throw std::runtime_error("file not found");
    }
    std::ifstream in_w(weight_file); 
    if(!in){
        std::cerr<<"file not found\n";
        throw std::runtime_error("file not found");
    }
    std::string line;
    std::vector<std::vector<Edge>> edge ;
    
    while (std::getline(in,line)){
        if(line.empty()){
            continue;
        }
        if(line[0]=='/')
            continue;
        if(line[0]=='#'){
            edge.push_back({});
            continue;
        }
        std::vector<std::string> nums ;
        boost::split(nums,line,isspace);
        nums.erase(std::remove_if(nums.begin(),nums.end(),[](const std::string & s){return s.empty();}) ,nums.end());
        if(nums.empty())
            throw std::runtime_error("empty string number");
        for(auto &item:nums){
            std::string::size_type index=item.find(':');
            if(index==std::string::npos)
                throw std::runtime_error("format error");
            int node = std::stoi(item.substr( 0,index) );
            double weight=std::stod(item.substr(index+1));
            edge.back().push_back( Edge(edge.size()-1,node,weight)  ) ;
        }
    }

    std::vector<Task> weights;
    int id=0;
    while (std::getline(in_w,line)){
        if(line.empty()){
            continue;
        }
        if(line[0]=='/'||line[0]=='#')
            continue;
        double w=std::stod(line) ;
        weights.push_back( Task(id++,w) );
    }
    graph=std::move(edge);
    task_list=std::move(weights);
}

void WCET::init(const std::string &wcet_file)
{
    std::ifstream in(wcet_file);
    if(!in){
        std::cout<<wcet_file<<" open fail\n";
        throw std::runtime_error("file not found");
    }
    std::string line;
    int count =0;
    task_proc_info.clear();
    while(std::getline(in,line)){
        if(line.empty())
            continue;
        if(line[0]=='/')
            continue; 
        if(line[0]=='#'){
            count++;
            continue;
        }
        std::vector<std::string> nums ;
        boost::split(nums,line,isspace);
        nums.erase(std::remove_if(nums.begin(),nums.end(),[](const std::string & s){return s.empty();}) ,nums.end());
        for(auto &item:nums){
            task_proc_info.push_back(std::stod(item));
        }
    }
    if(task_proc_info.size()%count!=0)
        throw std::runtime_error("task info size not match");
    n_task=count ;
    m_proc=task_proc_info.size()/count ;
}



void DAG::compute_up_rank(const WCET &proc_info)
{
    for(int r_index=graph.size()-1;0<=r_index;--r_index){
        double avg_w=0;
        double sum=0;
        for(int i=0;i<proc_info.get_num_of_process();i++){
            sum+=proc_info.get_wcet_info(r_index,i);
        }
        avg_w=sum/proc_info.get_num_of_process();
        double max_succ=0;
        for(auto pos=graph[r_index].begin();pos!=graph[r_index].end();++pos){
            double succ_rank=task_list[pos->end_node].get_up_rank()+pos->get_weight();
            if(succ_rank>max_succ)
                max_succ = succ_rank ;               
        }
        task_list[r_index].set_avg_weight(avg_w);
        task_list[r_index].set_up_rank(max_succ+avg_w);
    }
}

void DAG::compute_parent()
{
    std::vector<Task> task_queue=get_task_list();
    std::sort(task_queue.begin(),task_queue.end());
    for(auto &task:task_queue){
        for( auto &edge: graph[task.get_task_id()] ){
            task_list[edge.get_end_note()].add_parent(task.get_task_id());
        }
    }
}


void DAG::compute_down_rank(const WCET & proc_info)
{
    for(int index=0;index<task_list.size();++index){
        double max_parent=0;
        auto parent=task_list[index].get_parent();
        for(int pos=0;pos<parent.size();++pos){
            double weight=0; //weight is communication time between parent and child
            for(auto ptr=graph[parent[pos]].begin();ptr!=graph[parent[pos]].end();++ptr)
                if(ptr->get_end_note()==index)
                    weight=ptr->get_weight();
            double parent_rank=task_list[ parent[pos] ].get_down_rank()+weight;
            double avg_w=0;
            double sum=0;
            for(int i=0;i<proc_info.get_num_of_process();i++){
                sum+=proc_info.get_wcet_info(parent[pos],i);
            }
            avg_w=sum/proc_info.get_num_of_process();
            parent_rank+=avg_w ;
            if(parent_rank>max_parent)
                max_parent=parent_rank ;
        }
        task_list[index].set_down_rank(max_parent);
    }
}

void DAG::init_parent(const std::string &parent_file)
{
    std::ifstream in(parent_file);
    if(!in){
        std::cout<<parent_file<<" open fail\n";
        throw std::runtime_error("file not found");
    }
    std::string line;
    std::vector<int> p;
    int count = -1;
    while (std::getline(in,line)){
        if(line.empty()){
            continue;
        }
        if(line[0]=='/')
            continue;
        if(line[0]=='#'){
            p={};
            count++;
            continue;
        }
        std::vector<std::string> nums ;
        boost::split(nums,line,isspace);
        nums.erase(std::remove_if(nums.begin(),nums.end(),[](const std::string & s){return s.empty();}) ,nums.end());
        if(nums.empty())
            throw std::runtime_error("empty string number");
        for(auto &item:nums){
            int node = std::stoi(item);
            p.push_back(node);
        }
        if(count<graph.size())
            task_list[count].set_parent_list(p);
    }
}

double get_energy_consume(double p_ind,double c_ef,double mk,double f_max,double w_ik,double f_h)
{
    double P_h=p_ind+ c_ef* pow(f_h,mk);
    return P_h*w_ik*f_max/f_h ;
}

double get_raliability(double f_max,double f_min, double f_kv,double w_ik,double d_constant ,double lamda_max)
{

    double run1= w_ik*f_max/f_kv ;
    double run2 = d_constant *(f_max-f_kv)/(f_max-f_min);
    double exponent= -lamda_max * pow(10,run2)*run1 ;
    return exp(exponent);
}
//get energy-efficient frequency
double get_fee(double p_ind,double m,double c_ef){
    if(m==1)
        throw std::runtime_error("m must greater than 1");
    return pow(p_ind/((m-1)*c_ef) ,1/m );
}

std::vector<Process> init_proc_from_file(const std::string &proc_file)
{
    std::ifstream in(proc_file);
    if(!in){
        std::cout<<proc_file<<" open fail\n";
        throw std::runtime_error("file not found");
    }
    std::string line;
    std::vector<Process> procs;
    int count=0;
    while (std::getline(in,line)){
        if(line.empty()){
            continue;
        }
        if(line[0]=='/')
            continue;
        if(line[0]=='#'){
            continue;
        }
        std::vector<std::string> proc_info;
        boost::split(proc_info,line,isspace);
        proc_info.erase(std::remove_if(proc_info.begin(),proc_info.end(),[](const std::string & s){return s.empty();}) ,proc_info.end());
        if(proc_info.empty())
            throw std::runtime_error("empty string number");
        int size=proc_info.size();
        if (!(size==6 || size==7 ||size==8) )
            throw std::runtime_error("size error");
        double p_ind=std::stod(proc_info[0]);
        double c_ef=std::stod(proc_info[1]);
        double m_k=std::stod(proc_info[2]);
        int f_low=std::stoi(proc_info[3]);
        int f_max=std::stoi(proc_info[4]);
        int step=std::stoi(proc_info[5]);
        double lambda_max=1;
        double fac=1 ;
        if(size >= 7)
            lambda_max = std::stod(proc_info[6]);
        if(size == 8)
            fac= std::stod(proc_info[7]);
        procs.push_back( Process(count++,p_ind,c_ef,m_k,f_low,f_max,step,lambda_max,fac));
    }
    return procs ;
}

double get_min_or_max_energy_consume(const Task &t,const std::vector<Process> &procs,const WCET &wect,bool is_max)
{
    double max_or_min_energy=0;
    if(is_max)
        max_or_min_energy = std::numeric_limits<double>::min();
    else
        max_or_min_energy = std::numeric_limits<double>::max();
    double freq=0;
    std::vector<double> task_wect=wect.get_task_wect(t.get_task_id());
    for(auto pos=procs.begin();pos!=procs.end();++pos){
        if(is_max)
            freq=pos->get_f_max() ;
        else
            freq=pos->get_f_ee() ;
        int index=pos-procs.begin();
        double energy=get_energy_consume(pos->p_ind,pos->c_ef,pos->mk,pos->get_f_max(),task_wect[index],freq);
        if(is_max){
            if(energy>max_or_min_energy)
                max_or_min_energy=energy;
        }else{
            if( energy<max_or_min_energy)
                max_or_min_energy=energy;
        }
    }
    return max_or_min_energy;
}

double get_range_sum_min_nenrgy_comsume(std::vector<Task>::iterator beg, std::vector<Task>::iterator end,
                                        const std::vector<Process> &procs,const WCET &wect)
{        
    double energy_min_total=0;
    for(;beg!=end;++beg){
        energy_min_total+=beg->get_min_nergy_consume();
    }
    return energy_min_total;
}

double get_earliest_finish_tm(double est,const Task &t,const Process& proc,const WCET& wect,double f_h){
    int task_id= t.get_task_id();
    int proc_id=proc.proc_id;
    double tmp= wect.get_wcet_info(task_id,proc_id)* proc.get_f_max()/f_h;    // equation: W_i_k*F_k_max/F_k_h
    return tmp+est ;
}