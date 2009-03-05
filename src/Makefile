CC=gcc
LIBS=-lusb -lbluetooth -lpthread `pkg-config --libs glib-2.0`
OBJS=utils.o bluetooth_layer.o j_protocol.o j_command_manager.o
INCS=`pkg-config --cflags glib-2.0`

all: johnnyCon

johnnyCon: johnnyCon.c $(OBJS)
	$(CC) -Wall $(INCS) $(LIBS) -o johnnyCon johnnyCon.c $(OBJS)

bluetooth_layer.o: bluetooth_layer.h bluetooth_layer.c
	$(CC) $(INCS) -Wall -o bluetooth_layer.o -c bluetooth_layer.c

j_protocol.o: j_protocol.h j_protocol.c
	$(CC) $(INCS) -Wall -o j_protocol.o -c j_protocol.c

j_command_manager.o: j_command_manager.h j_command_manager.c
	$(CC) $(INCS) -Wall -o j_command_manager.o -c j_command_manager.c

utils.o: utils.h utils.c
	$(CC) $(INCS) -Wall -o utils.o -c utils.c
	
clean:
	rm -rf *.o
	rm johnnyCon
	
