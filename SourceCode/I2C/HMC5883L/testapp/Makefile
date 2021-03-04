OBJS=app.o

CC=arm-linux-gnueabihf-gcc

CFLAGS+=-c -Wall -g

LDLIBS=-lm

app:$(OBJS)
	$(CC) $^ -o $@ $(LDLIBS)

%.o:%.c
	$(CC) $^ $(CFLAGS) -o $@

clean:
	$(RM) *.o mytool -r
