gcc master.c functions.c -o master -lncurses -lm
gcc -Wall -Wextra blackboard.c functions.c -lncurses -lm -o blackboard 
gcc drone.c functions.c -o drone -lncurses -lm
gcc input_manager.c -lncurses -o input_manager
gcc obstacles.c functions.c -o obstacles -lncurses -lm
gcc targets.c functions.c -o targets -lncurses -lm
./master