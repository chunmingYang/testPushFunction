COMMON = -O2 -I../include -L../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LDFLAGS = -lnlopt -lm
CC = gcc
ROOT = main
CFLAGS = -c -w -Wno-implicit-function-declaration

all: main_all

main_all: main.o
	#$(CC) $(COMMON) $(LDFLAGS) $(ROOT) main.o -w -lnlopt 
	$(CC) $(COMMON) main.o -w -lnlopt $(LDFLAGS) ../bin/libnlopt.so -o ../bin/$(ROOT)
main.o: main.c
	$(CC) $(COMMON) $(CFLAGS) main.c
clean:
	rm main.o