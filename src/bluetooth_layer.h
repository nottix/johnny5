#ifndef BLUETOOTH_LAYER_H
#define BLUETOOTH_LAYER_H

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

int bt_connect(char *dest);

void bt_close(int s);

#endif //BLUETOOTH_LAYER_H
