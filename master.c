#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>

int main(){
    char* arg_list1[] = {"konsole", "-e", "./blackboard", NULL};
    char* arg_list2[] = {"konsole", "-e", "./drone", NULL};
    char* arg_list3[] = {"konsole", "-e", "./input_manager", NULL};

    pid_t child1 = fork();
    if(child1 < 0){
        perror("fork1");
        exit(EXIT_FAILURE);
    }
    if(child1 == 0){
        execvp(arg_list1[0], arg_list1);
    }

    pid_t child2 = fork();
    if(child2 < 0){
        perror("fork2");
        exit(EXIT_FAILURE);
    }
    if(child2 == 0){
        execvp(arg_list2[0], arg_list2);
    }

    pid_t child3 = fork();
    if(child3 < 0){
        perror("fork3");
        exit(EXIT_FAILURE);
    }
    if(child3 == 0){
        execvp(arg_list3[0], arg_list3);
    }

    return 0;
}