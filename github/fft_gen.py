from math import log2
from random import randint

def get_random():
    return str(randint(10,100))


def cross(serial,size,level):

    if len(serial)%2!=0:
        raise
    half=int(len(serial)/2)
    result=[]
    for i in range(half):
        num1=serial[i]+size
        num2=serial[i]+size+level
        result.append([num1,num2])
    for n in range(half,len(serial)):
        num1=serial[n]+size
        num2=serial[n]+size-level
        result.append([num1,num2])
    return result
def fft_gen(file,size):
    if size <4:
        print("size to small")
        return
    if size &1 ==1:
        print("size must be even")
        return
    total=int(2*size-1 + size*log2(size))
    fptr=open(file,"w")
    merge_time=int(log2(size))
    for i in range(1,size):
        fptr.write("#"+str(i-1)+"\n")
        fptr.write(str(i*2-1)+":"+get_random()+"  "+str(2*i+1-1)+":"+get_random()+"\n")
    start_num=size
    n_m=2
    #要交叠的次数
    level=0
    idx=0
    for n in range(merge_time):
        #产生交叠的序列 以2开始 幂乘
        m=0
        serial=[i for i in range(start_num,start_num+size)]
        for t in range(int(size/n_m)):
            partial_serial=serial[m:m+n_m]
            ans=cross(partial_serial,size,2**level)
            for line in ans:
                fptr.write("#"+str(idx+size-1)+"\n")
                fptr.write(str(line[0]-1)+":"+get_random()+"  "+str(line[1]-1)+":"+get_random()+"\n")
                idx+=1
            #print(ans)
            m=n_m*(t+1)
        n_m*=2
        start_num+=size
        level+=1
    for n in range(total-size+1,total+1):
        fptr.write("#"+str(n-1)+"\n")
        fptr.write(str(total+1-1)+":"+"0\n")
    fptr.write("#"+str(total+1-1)+"\n")
    fptr.close()

def main():
    # ans=cross([24,25,26,27,28,29,30,31],8,4)
    # print(ans)
    fft_gen('fft_gen_128.txt',128)

if __name__=='__main__':
    main()