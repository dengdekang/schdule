
from random import randint,random

def main(file,proc_size):
    fptr=open(file,"w")
    fac=1e6
    for i in range(proc_size):
        p_ind=randint(3,7)/100
        c_ef=randint(8,12)/10
        m_k=randint(25,30)/10
        lambda_k=randint(1,9)
        fptr.write("#"+str(i)+'\n')
        fptr.write(  "%s %s %s %d %d %d %.6f \n"%(str(p_ind),str(c_ef),str(m_k),30,100,1 ,lambda_k/fac  )      )



if __name__=='__main__':
    main("process_32.txt" ,32)