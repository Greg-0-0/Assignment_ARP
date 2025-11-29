// gcc -Wall -Wextra blackboard.c -lncurses -o blackboard
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#define _XOPEN_SOURCE_EXTENDED
#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<ncurses.h>
#include <locale.h>

static void layout_and_draw(WINDOW *win) {
    int H, W;
    getmaxyx(stdscr, H, W);

    // Windows with fixed margins
    int wh = (H > 6) ? H - 6 : H;
    int ww = (W > 10) ? W - 10 : W;
    if (wh < 3) wh = 3;
    if (ww < 3) ww = 3;

    // Resize and recenter the window
    wresize(win, wh, ww);
    mvwin(win, (H - wh) / 2, (W - ww) / 2);

    // Klean up and redraw
    werase(stdscr);
    werase(win);
    box(win, 0, 0);
    getmaxyx(win, H, W);
    // Drawable region goes from y:1 to H-2 and x:1 to W-2

    // Spawn drone
    mvwprintw(win,H/2,W/2,"+");

    refresh();
    wrefresh(win);
}

int main(void) {
    // Receives request of new position from drone, non blocking otherwise it would block resize of window
    char* myfifo1 = "/tmp/myfifoFromDToBRequest";
    char* myfifo2 = "/tmp/myfifoFromBToD"; // Used to send current drone position, obstacle positions and borders position to drone
    // Receives new position of drone from drone (movement), can be blocking, since we are waiting for a new position, 
    // after pressing a key, and realistically we wouldn't choose to resize window at that moment
    char* myfifo3 = "/tmp/myfifoFromDToBNewPos";
    mkfifo(myfifo1, 0666);
    mkfifo(myfifo2, 0666);
    mkfifo(myfifo3, 0666);
    int fd1, fd2, fd3, H, W;
    char input_string[80];
    char output_string[80];
    char positions_format[80] = "pos: %d,%d, borders: %d,%d";
    int drone_position[2]; // (y,x)
    int borders[2];

    setlocale(LC_ALL, "");

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE); // To receive KEY_RESIZE
    nodelay(stdscr, TRUE); // getch() becomes non-blocking, this way there is no wait for resizing, and code can keep executing

    // Window with temporary dimensions
    WINDOW *win = newwin(3, 3, 0, 0);
    layout_and_draw(win);
    getmaxyx(win, H, W);

    // Initial drone position
    drone_position[0] = H/2; // y
    drone_position[1] = W/2; // x
    
    fd1 = open(myfifo1, O_RDONLY | O_NONBLOCK);
    fd3 = open(myfifo3, O_RDONLY | O_NONBLOCK);
    fd2 = open(myfifo2, O_WRONLY);
    if(fd2 < 0){
        perror("open");
        return 0;
    }
    refresh();

    // Saving borders value
    borders[0] = H-7;
    borders[1] = W-7;
    
    
    while (1) {
        if (getch() == KEY_RESIZE) {
            // Resize window
            resize_term(0, 0);
            layout_and_draw(win);
            getmaxyx(win, H, W);

            // Reset drone position
            drone_position[0] = H/2; // y
            drone_position[1] = W/2; // x

            // Updating borders (margins of 5 pixels)
            borders[0] = H-7;
            borders[1] = W-7;
        }
        read(fd1, input_string, sizeof(input_string)); // Checks for position request from drone (non blocking)
        if(input_string[0] == 'p'){
            // Reset string to avoid rexecuting if statement
            strcpy(input_string,"");
            // Drone is asking for position
            sprintf(output_string,positions_format, drone_position[0], drone_position[1], borders[0], borders[1]);
            write(fd2,output_string,strlen(output_string)+1); // Sends current position
            // If drone is asking for position it wants to move, thus reading on fd3
            while(1){
                //printf("try1\n");
                read(fd3,output_string,sizeof(output_string)); // Receiving new position
                //printf("try2\n");
                if(output_string[0] == 's')
                    break;
                if(output_string[0] == 'q')
                    exit(EXIT_SUCCESS);
                mvwaddch(win,drone_position[0],drone_position[1],' ');
                sscanf(output_string,positions_format,&drone_position[0],&drone_position[1],&borders[0],&borders[1]);
                mvwprintw(win,drone_position[0],drone_position[1],"+");
                wrefresh(win);
            }
        }
        else if(input_string[0] == 'q'){
            exit(EXIT_SUCCESS);
        }
        wrefresh(win);
    }

    close(fd1);
    close(fd2);
    close(fd3);

    delwin(win);
    endwin();
    return 0;
}