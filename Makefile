
CFLAGS=-Wall

all:	pitadiag

install: all
	install -m 755 pitadiag /usr/bin/pitadiag

pitadiag:	pitadiag.c fftsg.c
	cc -Wall pitadiag.c fftsg.c -o pitadiag -lasound -lpthread -lm


