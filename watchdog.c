#include"functions.h"

int main(int argc, char* argv[]){
    if(argc < 11){
        fprintf(stderr,"No arguments passed to watchdog\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0);
    if (log_sem == SEM_FAILED) {
        perror("sem_open");
        exit(EXIT_FAILURE);
    }

    int fd_from_b = atoi(argv[1]); // reads signal from blackboard
    int fd_from_d = atoi(argv[2]); // reads signal from drone
    int fd_from_i = atoi(argv[3]); // reads signal from input_manager
    int fd_from_o = atoi(argv[4]); // reads signal from obstacles
    int fd_from_t = atoi(argv[5]); // reads signal from targets
    pid_t b_pid = atoi(argv[6]); // blackboard PID
    pid_t d_pid = atoi(argv[7]); // drone PID
    pid_t i_pid = atoi(argv[8]); // input_manager PID
    pid_t o_pid = atoi(argv[9]); // obstacles PID
    pid_t t_pid = atoi(argv[10]); // targets PID

    const char *names[5] = {"BLACKBOARD", "DRONE", "INPUT_MANAGER", "OBSTACLES", "TARGETS"};
    int fds[5] = {fd_from_b, fd_from_d, fd_from_i, fd_from_o, fd_from_t};
    pid_t pids[5] = {b_pid, d_pid, i_pid, o_pid, t_pid};
    time_t last_seen[5];
    int alerted[5] = {0,0,0,0,0};
    for(int i = 0; i < 5; ++i) last_seen[i] = time(NULL);

    write_log("watchdog.log", "WATCHDOG", "INFO", "Watchdog started", log_sem);

    while(1){
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = -1;
        for(int i = 0; i < 5; ++i){
            FD_SET(fds[i], &rfds);
            if(fds[i] > maxfd) maxfd = fds[i];
        }

        // Set timeout for select
        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 500000; // 500 ms
        int ret = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if(ret == -1){
            log_error("watchdog.log", "WATCHDOG", "select", log_sem);
            perror("select");
            exit(EXIT_FAILURE);
        }
        if(ret > 0){
            for(int i = 0; i < 5; ++i){
                // Check which fd is ready
                if(FD_ISSET(fds[i], &rfds)){
                    pid_t incoming;
                    ssize_t n = read(fds[i], &incoming, sizeof(incoming));
                    if(n == (ssize_t)sizeof(incoming)){
                        last_seen[i] = time(NULL);
                        if(alerted[i]){
                            // Process has sent heartbeat again after being alerted
                            alerted[i] = 0;
                            char msg[128];
                            snprintf(msg, sizeof msg, "%s recovered", names[i]);
                            write_log("watchdog.log", "WATCHDOG", "INFO", msg, log_sem);
                        }
                    }
                    else if(n == 0){
                        char msg[128];
                        snprintf(msg, sizeof msg, "%s pipe closed", names[i]);
                        write_log("watchdog.log", "WATCHDOG", "WARNING", msg, log_sem);
                    }
                    else{
                        log_error("watchdog.log", "WATCHDOG", "read heartbeat", log_sem);
                    }
                }
            }
        }

        time_t now = time(NULL);
        for(int i = 0; i < 5; ++i){
            if(!alerted[i] && difftime(now, last_seen[i]) > 3.0){
                // Process did not send heartbeat for more than 3 seconds
                char msg[160];
                snprintf(msg, sizeof msg, "%s (pid %d) timed out", names[i], (int)pids[i]);
                write_log("watchdog.log", "WATCHDOG", "ERROR", msg, log_sem);
                alerted[i] = 1;
            }
        }
    }

    for(int i = 0; i < 5; ++i) close(fds[i]);
    sem_close(log_sem);
    return 0;
}