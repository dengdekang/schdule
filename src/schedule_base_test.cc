//#include <boost/test/unit_test.hpp>
#include "schedule_base.hpp"
#include <iostream>
#include "algorithm_base.hpp"
#include "mslecc.hpp"
#include "isaecc.hpp"
#include "heft.hpp"
#include "esrg.hpp"
#include "sarc.hpp"
#include "efsrg.hpp"
#include "derg.hpp"
#include "xxxx.hpp"
void test()
{

    WCET w_case("wect.txt");
    DAG grap("edge.txt","task_node.txt");
    grap.init_parent("parent.txt");
    std::vector<Process> procs=init_proc_from_file("process.txt");
    grap.compute_up_rank(w_case);
    grap.compute_down_rank(w_case);

    double max_rank=std::numeric_limits<double>::min();
    for(auto &item:grap.get_task_list()){
        if(max_rank<item.get_down_rank()+item.get_up_rank())
            max_rank=item.get_down_rank()+item.get_up_rank();
    }
    std::vector<Task> critical_nodes;
    for(auto &item:grap.get_task_list()){
        if(abs(item.get_down_rank()+item.get_up_rank()-max_rank)<1e-5)
            critical_nodes.push_back(item);
    }
    int proc_index=-1;
    int last_node=0;
    double w_tmp=0;
    double w_min=std::numeric_limits<double>::max();
    for(auto pos=critical_nodes.begin();pos!=critical_nodes.end();++pos){       
        double  comu=0;
        for(auto &proc:procs){
            if(pos!=critical_nodes.begin()){
                if(proc_index==proc.proc_id)
                    comu=0;
                else
                    comu=grap.get_edge_weight(last_node,pos->get_task_id());
            }
            w_tmp=w_case.get_wcet_info(pos->get_task_id(),proc.proc_id)+comu;
            if(w_tmp<w_min){
                w_min =w_tmp;
                proc_index=proc.proc_id;
            }
        }
        last_node=pos->get_task_id() ;
        std::cout<<proc_index<<" :"<<w_min<<"  \t";
    }
}


void print(std::vector<double> & datas,const std::string &title,const std::string &item){
    std::cout<<title<<"\n";
    std::cout<<item<<":";
    for(auto &data:datas)
        std::cout<<data<<" ";
    std::cout<<std::endl;
}


int main(int argc,char*argv[]){
    if(argc!=5){
        std::cerr<<"usage :\n"<<"program_name  wect adge task_node process"<<std::endl;
        return -1;
    }

    WCET w_case(argv[1]);
    DAG grap(argv[2],argv[3]);
    grap.compute_up_rank(w_case);
    grap.compute_parent();
    //grap.init_parent("parent.txt");
    std::vector<Process> procs=init_proc_from_file(argv[4]);

    double target_relia=0.93;
    //grap.compute_down_rank(w_case);
    // range from 0.93 to 0.98
    std::vector<double> esrg_energy;
    std::vector<double> sarc_energy;
    std::vector<double> efsrg_energy;
    std::vector<double> derg_energy;
    std::vector<double> xxxx_energy;

    std::vector<double> esrg_relia;
    std::vector<double> sarc_relia;
    std::vector<double> efsrg_relia;
    std::vector<double> derg_relia;
    std::vector<double> xxxx_relia;

    std::vector<double> esrg_length;
    std::vector<double> sarc_length;
    std::vector<double> efsrg_length;
    std::vector<double> derg_length;
    std::vector<double> xxxx_length;



    for(int i=0;i<6;i++,target_relia+=0.01){
        std::cout<<"+++++++++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"taget reliability goal:"<<target_relia<<std::endl;
        Esrg sg(grap,w_case,procs);
        sg.set_goal_reliability(target_relia);
        std::cout<<"Esrg:\n";
        try{
            sg.compute();
        //sg.print();
            esrg_energy.push_back(sg.get_total_energy_consume());
            esrg_length.push_back(sg.get_schedule_length());
            esrg_relia.push_back(sg.get_reliabilitys());

            std::cout<<"energy:"<<sg.get_total_energy_consume()<<std::endl;
            std::cout<<"reliability:"<<sg.get_reliabilitys()<<std::endl;
            std::cout<<"SL(G):"<<sg.get_schedule_length()<<std::endl;
            std::cout<<"----------------------------"<<std::endl;
        }catch(std::runtime_error &err){
            std::cout<<"esrg can't run"<<std::endl;
            esrg_energy.push_back(0);
            esrg_length.push_back(0);
            esrg_relia.push_back(0);
        }

        Sarc sc (grap,w_case,procs);
        sc.set_goal_reliability(target_relia);
        sc.set_factor(1000);
        try{
            sc.compute();
        //sc.print();
            std::cout<<"sarc:\n";
            std::cout<<"energy:"<<sc.get_total_energy_consume()<<std::endl;
            std::cout<<"reliability:"<<sc.get_reliabilitys()<<std::endl;
            std::cout<<"SL(G):"<<sc.get_schedule_length()<<std::endl;
            std::cout<<"----------------------------"<<std::endl;
            sarc_energy.push_back(sc.get_total_energy_consume());
            sarc_length.push_back(sc.get_schedule_length());
            sarc_relia.push_back(sc.get_reliabilitys());
        }catch(std::out_of_range &err){
            std::cout<<"sarc can't run\n";
            sarc_energy.push_back(0);
            sarc_length.push_back(0);
            sarc_relia.push_back(0);
        }
        // std::cout<<"energy:"<<sc.get_total_energy_consume()<<std::endl;
        // std::cout<<"reliability:"<<sc.get_reliabilitys()<<std::endl;
        // sc.print();

        Efsrg eg(grap,w_case,procs);
        eg.set_goal_reliability(target_relia);
        eg.compute();
        std::cout<<"efsrg:\n";
        std::cout<<"energy:"<<eg.get_total_energy_consume()<<std::endl;
        std::cout<<"reliability:"<<eg.get_reliabilitys()<<std::endl;
        std::cout<<"SL(G):"<<eg.get_schedule_length()<<std::endl;
        efsrg_energy.push_back(eg.get_total_energy_consume());
        efsrg_length.push_back(eg.get_schedule_length());
        efsrg_relia.push_back(eg.get_reliabilitys());
        //eg.print();
        std::cout<<"-------------------\n";
        Derg rg(grap,w_case,procs);
        try{
            rg.set_goal_reliability(target_relia);
            rg.compute(); 
        //rg.print();
            std::cout<<"derg:\n";
            std::cout<<"energy:"<<rg.get_total_energy_consume()<<std::endl;
            std::cout<<"reliability:"<<rg.get_reliabilitys()<<std::endl;
            std::cout<<"SL(G):"<<rg.get_schedule_length()<<std::endl;
            std::cout<<"------------------"<<std::endl;
            derg_energy.push_back(rg.get_total_energy_consume());
            derg_length.push_back(rg.get_schedule_length());
            derg_relia.push_back(rg.get_reliabilitys());
        }catch(std::out_of_range &err){
            std::cout<<"derg can't run\n";
            derg_energy.push_back(0);
            derg_length.push_back(0);
            derg_relia.push_back(0);
        }

        XXXX xal(grap,w_case,procs);
        xal.set_goal_reliability(target_relia);
        xal.set_alpha(0.2);
        xal.set_factor(1000);
        xal.compute();
        //xal.print();
        std::cout<<"xxxx:\n";
        std::cout<<"energy:"<<xal.get_total_energy_consume()<<std::endl;
        std::cout<<"reliability:"<<xal.get_reliabilitys()<<std::endl;
        std::cout<<"SL(G):"<<xal.get_schedule_length()<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++++++++"<<std::endl;
        xxxx_energy.push_back(xal.get_total_energy_consume());
        xxxx_length.push_back(xal.get_schedule_length());
        xxxx_relia.push_back(xal.get_reliabilitys());

    }
    print(esrg_energy,"esrg","energy");
    print(esrg_length,"esrg","length");
    print(esrg_relia,"esrg","reliability");

    print(sarc_energy,"sarc","energy");
    print(sarc_length,"sarc","length");
    print(sarc_relia,"sarc","reliability");

    print(efsrg_energy,"efsrg","energy");
    print(efsrg_length,"efsrg","length");
    print(efsrg_relia,"efsrg","reliability");


    print(derg_energy,"derg","energy");
    print(derg_length,"derg","length");
    print(derg_relia,"derg","reliability");

    print(xxxx_energy,"xxxx","energy");
    print(xxxx_length,"xxxx","length");
    print(xxxx_relia,"xxxx","reliability");



    //rg.engry_relia();
    // // Mslecc  ms(grap,w_case,procs);
    // ms.set_energy(80.9940);
    // ms.compute();
    // ms.print();
    // std::cout<<"\n";
    // Isaecc isa (grap,w_case,procs);
    // isa.set_energy(80.9940);
    // isa.compute();
    // isa.print();
    // std::cout<<"SL(G):"<<ms.get_schedule_length()<<"\t"<<"SL(G):"<<isa.get_schedule_length()<<std::endl;
    // std::cout<<"MS:"<<ms.get_total_energy_consume()<<"\tISA:"<<isa.get_total_energy_consume()<<std::endl;
    // Heft ht(grap,w_case,procs);
    // ht.compute();
    // std::cout<<"SL(G):"<<ht.get_schedule_length()<<std::endl;
    // std::cout<<"complete\n";
}
