CC=clang
CXX=clang++ 

ARGS= -std=c++11 -Wall -g -c
PREFIX = 

test:mslecc.o schedule_base.o schedule_base_test.o isaecc.o heft.o esrg.o sarc.o efsrg.o derg.o xxxx.o
	$(CXX)  mslecc.o isaecc.o schedule_base.o schedule_base_test.o heft.o esrg.o sarc.o -o schedule_base_test efsrg.o derg.o xxxx.o

esrg.o :esrg.cc algorithm_base.hpp schedule_base.hpp esrg.hpp
	$(CXX) $(ARGS) esrg.cc
isaecc.o : isaecc.cc isaecc.hpp algorithm_base.hpp schedule_base.hpp
	$(CXX) $(ARGS) isaecc.cc 
mslecc.o : mslecc.cc mslecc.hpp algorithm_base.hpp schedule_base.hpp
	$(CXX) $(ARGS) mslecc.cc
schedule_base.o:schedule_base.cc schedule_base.hpp
	$(CXX) $(ARGS) schedule_base.cc
schedule_base_test.o:schedule_base_test.cc mslecc.o schedule_base.o isaecc.o
	$(CXX) $(ARGS) schedule_base_test.cc
heft.o : heft.cc heft.hpp
	$(CXX) $(ARGS) heft.cc
sarc.o: sarc.cc sarc.hpp algorithm_base.hpp schedule_base.hpp
	$(CXX) $(ARGS) sarc.cc

efsrg.o : efsrg.cc efsrg.hpp algorithm_base.hpp schedule_base.hpp 
	$(CXX) $(ARGS) efsrg.cc

derg.o : derg.cc derg.hpp algorithm_base.hpp schedule_base.hpp 
	$(CXX) $(ARGS) derg.cc

xxxx.o : xxxx.hpp xxxx.cc algorithm_base.hpp schedule_base.hpp 
	$(CXX) $(ARGS) xxxx.cc
.PHONY :clean
clean :
	-rm -rf mslecc.o schedule_base.o schedule_base_test.o isaecc.o heft.o esrg.o efsrg.o sarc.o derg.o xxxx.o



