def gen_wect(file,size):
    fptr=open(file,"w")
    for i in range(size):
        fptr.write("#"+str(i)+"\n")
        fptr.write("0 ")
        fptr.write("\n")

def main():
    gen_wect("task_node1151.txt",1152)
if __name__=="__main__":
    main()