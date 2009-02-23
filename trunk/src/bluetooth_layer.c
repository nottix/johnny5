#include "bluetooth_layer.h"

int bt_connect(char *dest) {
	struct sockaddr_rc addr = { 0 };
	int s, status;

	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = 1;
	str2ba( dest, &addr.rc_bdaddr );

	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	
	return s;
}

void bt_close(int s) {
	close(s);
}
