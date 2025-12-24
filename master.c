#include"functions.h"

int main(){
    // Arrays to store file descriptors
    int pipe_I_to_D[2];
    int pipe_D_to_B_Req[2];
    int pipe_B_to_D_Pos[2];
    int pipe_D_to_B_NPos[2];
    int pipe_O_to_B[2];
    int pipe_B_to_O[2];
    int pipe_B_to_T[2];
    int pipe_T_to_B[2];
    int pipe_B_to_I[2];
    int pipe_B_to_W[2];
    int pipe_D_to_W[2];
    int pipe_I_to_W[2];
    int pipe_O_to_W[2];
    int pipe_T_to_W[2];
    int blackboard_pid, drone_pid, input_manager_pid, obstacles_pid, targets_pid;

    sem_t *log_sem = sem_open("/log_sem", O_CREAT, 0666, 1);// Create semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("sem_open");
        exit(EXIT_FAILURE);
    }

    if(pipe(pipe_I_to_D) < 0 || pipe(pipe_D_to_B_Req) < 0 || pipe(pipe_B_to_D_Pos) < 0 || pipe(pipe_D_to_B_NPos) < 0 || 
        pipe(pipe_O_to_B) < 0 || pipe(pipe_B_to_O) < 0 || pipe(pipe_B_to_T) < 0 || pipe(pipe_T_to_B) < 0 || pipe(pipe_B_to_I) < 0 ||
        pipe(pipe_B_to_W) < 0 || pipe(pipe_D_to_W) < 0 || pipe(pipe_I_to_W) < 0 || pipe(pipe_O_to_W) < 0 || pipe(pipe_T_to_W) < 0){

        log_error("application.log", "MASTER", "pipe", log_sem);
        perror("pipe");
        exit(EXIT_FAILURE);
    }

    // ---------- spawn blackboard ----------
    char argb1[32], argb2[32], argb3[32], argb4[32], argb5[32], argb6[32], argb7[32], argb8[32], argb9[32];
    snprintf(argb1, sizeof argb1, "%d", pipe_D_to_B_Req[0]); // read requests from drone
    snprintf(argb2, sizeof argb2, "%d", pipe_D_to_B_NPos[0]); // read new positions from drone
    snprintf(argb3, sizeof argb3, "%d", pipe_B_to_D_Pos[1]); // replies to drone
    snprintf(argb4, sizeof argb4, "%d", pipe_B_to_O[1]); // writes positions to obstacles
    snprintf(argb5, sizeof argb5, "%d", pipe_O_to_B[0]); // reads positions from obstacles
    snprintf(argb6, sizeof argb6, "%d", pipe_B_to_T[1]); // writes positions to targets
    snprintf(argb7, sizeof argb7, "%d", pipe_T_to_B[0]); // reads positions from targets
    snprintf(argb8, sizeof argb8, "%d", pipe_B_to_I[1]); // writes positions and score to input manager for display visualization
    snprintf(argb9, sizeof argb9, "%d", pipe_B_to_W[1]); // writes signal to watchdog
    char *args_blackboard[] = { "konsole", "-e", "./blackboard", argb1, argb2, argb3, argb4, argb5, argb6, argb7, argb8, argb9, NULL };
    blackboard_pid = spawn(args_blackboard[0], args_blackboard);

    // ---------- spawn drone ----------
    char argd1[32], argd2[32], argd3[32], argd4[32], argd5[32];
    snprintf(argd1, sizeof argd1, "%d", pipe_D_to_B_Req[1]); // writes requests to blackboard
    snprintf(argd2, sizeof argd2, "%d", pipe_D_to_B_NPos[1]); // writes new positions to blackboard
    snprintf(argd3, sizeof argd3, "%d", pipe_B_to_D_Pos[0]); // reads replies from blackboard
    snprintf(argd4, sizeof argd4, "%d", pipe_I_to_D[0]); // reads commands from input_manager
    snprintf(argd5, sizeof argd5, "%d", pipe_D_to_W[1]); // writes signal to watchdog
    char *args_drone[] = {"./drone", argd1, argd2, argd3, argd4, argd5, NULL };
    drone_pid = spawn(args_drone[0], args_drone);

    // ---------- spawn input_manager ----------
    char argi1[32], argi2[32], argi3[32];
    snprintf(argi1, sizeof argi1, "%d", pipe_I_to_D[1]); // writes commands to drone
    snprintf(argi2, sizeof argi2, "%d", pipe_B_to_I[0]); // reads positions and score from blackboard
    snprintf(argi3, sizeof argi3, "%d", pipe_I_to_W[1]); // writes signal to watchdog
    char *args_input_manager[] = { "konsole", "-e", "./input_manager", argi1, argi2, argi3, NULL };
    input_manager_pid = spawn(args_input_manager[0], args_input_manager);

    // ---------- spawn obstacles ----------
    char argo1[32], argo2[32], argo3[32];
    snprintf(argo1, sizeof argo1, "%d", pipe_B_to_O[0]); // reads positions from blackboard
    snprintf(argo2, sizeof argo2, "%d", pipe_O_to_B[1]); // writes positions to blackboard
    snprintf(argo3, sizeof argo3, "%d", pipe_O_to_W[1]); // writes signal to watchdog
    char *args_obstacles[] = {"./obstacles", argo1, argo2, argo3, NULL };
    obstacles_pid = spawn(args_obstacles[0], args_obstacles);

    // ---------- spawn targets ----------
    char argt1[32], argt2[32], argt3[32];
    snprintf(argt1, sizeof argt1, "%d", pipe_B_to_T[0]); // reads positions from blackboard
    snprintf(argt2, sizeof argt2, "%d", pipe_T_to_B[1]); // writes positions to blackboard
    snprintf(argt3, sizeof argt3, "%d", pipe_T_to_W[1]); // writes signal to watchdog
    char *args_targets[] = {"./targets", argt1, argt2, argt3, NULL };
    targets_pid = spawn(args_targets[0], args_targets);

    // ---------- spawn watchdog ----------
    char argw1[32], argw2[32], argw3[32], argw4[32], argw5[32], argw6[32], argw7[32], argw8[32], argw9[32], argw10[32];
    snprintf(argw1, sizeof argw1, "%d", pipe_B_to_W[0]); // reads signal from blackboard
    snprintf(argw2, sizeof argw2, "%d", pipe_D_to_W[0]); // reads signal from drone
    snprintf(argw3, sizeof argw3, "%d", pipe_I_to_W[0]); // reads signal from input_manager
    snprintf(argw4, sizeof argw4, "%d", pipe_O_to_W[0]); // reads signal from obstacles
    snprintf(argw5, sizeof argw5, "%d", pipe_T_to_W[0]); // reads signal from targets

    // Passing PIDs of all processes to watchdog
    snprintf(argw6, sizeof argw6, "%d", blackboard_pid);
    snprintf(argw7, sizeof argw7, "%d", drone_pid);
    snprintf(argw8, sizeof argw8, "%d", input_manager_pid);
    snprintf(argw9, sizeof argw9, "%d", obstacles_pid);
    snprintf(argw10, sizeof argw10, "%d", targets_pid);
    char *args_watchdog[] = {"./watchdog", argw1, argw2, argw3, argw4, argw5, argw6, argw7, argw8, argw9, argw10, NULL };
    spawn(args_watchdog[0], args_watchdog);

    // Application has started successfully
    write_log("application.log", "MASTER", "INFO", "Application started successfully", log_sem);

    // Closing all pipes
    close(pipe_I_to_D[0]); close(pipe_I_to_D[1]);
    close(pipe_D_to_B_Req[0]); close(pipe_D_to_B_Req[1]);
    close(pipe_B_to_D_Pos[0]); close(pipe_B_to_D_Pos[1]);
    close(pipe_D_to_B_NPos[0]); close(pipe_D_to_B_NPos[1]);
    close(pipe_B_to_O[0]); close(pipe_B_to_O[1]);
    close(pipe_O_to_B[0]); close(pipe_O_to_B[1]);
    close(pipe_B_to_T[0]); close(pipe_B_to_T[1]);
    close(pipe_T_to_B[0]); close(pipe_T_to_B[1]);
    close(pipe_B_to_I[0]); close(pipe_B_to_I[1]);
    close(pipe_B_to_W[0]); close(pipe_B_to_W[1]);
    close(pipe_D_to_W[0]); close(pipe_D_to_W[1]);
    close(pipe_I_to_W[0]); close(pipe_I_to_W[1]);
    close(pipe_O_to_W[0]); close(pipe_O_to_W[1]);
    close(pipe_T_to_W[0]); close(pipe_T_to_W[1]);

    write_log("application.log", "MASTER", "INFO", "Master process terminated successfully", log_sem);

    sem_close(log_sem);

    return 0;
}