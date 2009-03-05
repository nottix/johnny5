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
#define J_LEN			1
#define J_DATA_LEN		11
#define J_PACKET_LEN	4+J_DATA_LEN
#define J_START_CODE	0x55
#define J_END_CODE		0x66

//Packet type
#define J_COMMAND		0x10

//Johnny command
//Johnny command
#define J_MOVE_MOTORS	0xA0	// Muove i motori dei passi desiderati
#define J_MOTORS_CONF	0xA1
#define J_SET_DIRS		0xA2	// Imposta le direzioni dei motori
#define J_READ_ENCS		0xA3	// Legge i valori degli encoder

//Motors directions
#define J_DIR1_CLOAK		0x20	// Direzione oraria del motore 1
#define J_DIR1_ANTICLOAK	0x21	// Direzione antioraria del motore 1
#define J_DIR2_CLOAK		0x22	// Direzione oraria del motore 2
#define J_DIR2_ANTICLOAK	0x23	// Direzione antioraria del motore 2

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
