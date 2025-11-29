#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<math.h>
#include<time.h>
#include<sys/select.h>

int main(){
    char *myfifo1 = "/tmp/myfifofromOtoB";
    char *myfifo2 = "/tmp/myfifofromBtoO";
    mkfifo(myfifo1, 0666);
    mkfifo(myfifo2, 0666);

    int fd1, fd2;

}