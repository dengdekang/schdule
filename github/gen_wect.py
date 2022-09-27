from random import randint

def gen_wect(file,size,proc_size,multi_exit=True):
    fptr=open(file,"w")
    for i in range(size):
        fptr.write("#"+str(i)+"\n")
        for n in range(proc_size):
            if i>size-2:
                if multi_exit:
                    fptr.write("0 ")
                else:
                    fptr.write(str(randint(10,100))+" ")
            else:
                fptr.write(str(randint(10,100))+" ")
        fptr.write("\n")

def main():
    gen_wect("wect_1151_32.txt",1152,32,True)
if __name__=="__main__":
    main()