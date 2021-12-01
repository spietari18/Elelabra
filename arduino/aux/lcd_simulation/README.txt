This is just here to develop button code for the arduino. Hardware buttons
are simulated via a /dev/input device. You can change the number of keys
that the program wants in main.c and compile against a source file that
declares a void code(unsigned long) which will be repeatedly called by main.c
until a SIGINT or SIGTERM is received. code() will be called with a bit vector
containing the currently polled keystates.
