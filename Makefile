# ----------------------------
#   COMPILATION SETTINGS
# ----------------------------

CC      = gcc
CFLAGS  = -Wall -Wextra
LIBS    = -lncurses -lm

# Common source file
COMMON  = functions.c

# Executables to build
TARGETS = master blackboard drone input_manager obstacles targets

# Default target
all: $(TARGETS)
	@echo "Running master..."
	./master

# ----------------------------
#   BUILD RULES
# ----------------------------

master: master.c $(COMMON)
	$(CC) $(CFLAGS) master.c $(COMMON) -o master $(LIBS)

blackboard: blackboard.c $(COMMON)
	$(CC) $(CFLAGS) blackboard.c $(COMMON) -o blackboard $(LIBS)

drone: drone.c $(COMMON)
	$(CC) $(CFLAGS) drone.c $(COMMON) -o drone $(LIBS)

input_manager: input_manager.c $(COMMON)
	$(CC) $(CFLAGS) input_manager.c $(COMMON) -o input_manager $(LIBS)

obstacles: obstacles.c $(COMMON)
	$(CC) $(CFLAGS) obstacles.c $(COMMON) -o obstacles $(LIBS)

targets: targets.c $(COMMON)
	$(CC) $(CFLAGS) targets.c $(COMMON) -o targets $(LIBS)

# ----------------------------
#   UTILITY TARGETS
# ----------------------------

clean:
	rm -f $(TARGETS) *.o

.PHONY: all clean
