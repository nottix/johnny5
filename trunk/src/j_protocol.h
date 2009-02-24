#ifndef J_PROTOCOL
#define J_PROTOCOL

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <glib.h>
#include <time.h>
#include "utils.h"

#define J_FLAGS			0
#define J_DATA_LEN		7
#define J_PACKET_LEN		4+J_DATA_LEN
#define J_LEN			J_PACKET_LEN
#define J_START_CODE		0x55
#define J_END_CODE		0x66

/*
 *    Johnny Packet
 * +----+------+-----+-----+
 * | ID | type | len |data |
 * +----+------+-----+-----+
 */
struct j_packet {
	u_int1	start;
	u_int1	type;
	u_int1	len;
	u_int1	data[J_DATA_LEN];
	u_int1	end;
};
typedef struct j_packet j_packet;

u_int1 j_packet_bin[J_PACKET_LEN];

extern GQueue *recvQueue;

j_packet *j_create_packet(u_int1 type, u_int1 data[J_DATA_LEN], u_int1 len);

u_int1 *j_struct_to_bin(j_packet *packet);

j_packet *j_bin_to_struct(u_int1 *bin);

int j_send(int socket, u_int1 *data, u_int1 len);

char *j_recv(int socket, int *len);

#endif //J_PROTOCOL
