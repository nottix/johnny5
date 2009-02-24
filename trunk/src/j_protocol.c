#include "j_protocol.h"

j_packet *j_create_packet(u_int1 type, u_int1 data[J_DATA_LEN], u_int1 len) {
	j_packet *packet = (j_packet*)calloc(1, sizeof(j_packet));
	packet->start = J_START_CODE;
	packet->type = type;
	packet->len = len;
	memset(packet->data, 0, J_DATA_LEN);
	memcpy(packet->data, data, len);
	packet->end = J_END_CODE;
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
	char *packet = calloc(J_LEN, 1);
	*len = 0;
	int n_read;
	
	printf("[j_recv] Enter\n");
//	while((n_read=recv(socket, data, J_LEN, J_FLAGS))>0) {
	while((n_read=read(socket, data, J_LEN))>0) {
		printf("[j_recv] Received: %s\n", hex_dump(data, n_read, 10));
		if(data[0]==J_START_CODE && n_read<J_PACKET_LEN) {
			memcpy(packet, data, n_read);
			*len = n_read;
		}
		else if(data[0]==J_START_CODE && n_read==J_PACKET_LEN && data[n_read-1]==J_END_CODE) {
			memcpy(packet, data, n_read);
			*len = n_read;
		}
		else if(data[0]!=J_START_CODE && *len<J_PACKET_LEN) {
			char *iter = packet;
			iter += *len;
			memcpy(iter, data, n_read);
			*len += n_read;
		}
		printf("[j_recv] Received: %s\n", hex_dump(packet, *len, 10));
		//g_queue_push_tail(recvQueue, (gpointer)data);
	}
	printf("[j_recv] Exit\n");
	return data;
}
