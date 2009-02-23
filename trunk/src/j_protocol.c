#include "j_protocol.h"

j_packet *j_create_packet(u_int1 type, u_int1 data[J_DATA_LEN], u_int1 len) {
	j_packet *packet = (j_packet*)calloc(1, sizeof(j_packet));
	packet->id = rand();
	packet->type = type;
	packet->len = len;
	memset(packet->data, 0, J_DATA_LEN);
	memcpy(packet->data, data, len);
	return packet;
}

u_int1 *j_struct_to_bin(j_packet *packet) {
	memcpy(j_packet_bin, (u_int1*)packet, J_PACKET_LEN);

	return j_packet_bin;
}

j_packet *j_bin_to_struct(u_int1 *bin) {
	j_packet *packet = calloc(1, sizeof(j_packet));
	packet = (j_packet*)bin;
	
	return packet;
}

int j_send(int socket, u_int1 *data, u_int1 len) {
	if(socket<0)
		return -1;
		
	printf("[j_send] sent\n");
	return send(socket, data, len, J_FLAGS);
}

char *j_recv(int socket, int *len) {
	if(socket<0)
		return NULL;
	
	recvQueue = g_queue_new();
	g_queue_init(recvQueue);
	char *data = calloc(J_LEN, 1);
	*len = 0;
	int n_read;
	while((n_read=recv(socket, data, J_LEN, J_FLAGS))>0) {
		printf("[j_recv] Received: %s\n", hex_dump(data, n_read, 10));
		//g_queue_push_tail(recvQueue, (gpointer)data);
	}
	return data;
}
