from logging import root
import random

def gau_gen(file,size):
    fptr=open(file,"w")
    sz=size
    if sz<1:
        raise Exception("error")
    total= int((sz*sz+sz-2)/2 )
    root =0
    for i in range(total):
        fptr.write("#"+str(i)+"\n")
        if i==root:
            for n in range(i+1,i+size):
                fptr.write(str(n)+":"+str(random.randint(10,100))+" ")
            root= i+size
            size -=1
        elif i==total-1:
            pass
        else:
            fptr.write(str(i+size)+":"+str(random.randint(10,100)))
        fptr.write("\n")


def main():
    gau_gen("gaussian_48.txt",48)

if __name__=='__main__':
    main()
