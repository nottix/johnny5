/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Project: JohnnyCon
 * Author: Simone Notargiacomo, Giuseppe Orlando
 * Version: 0.1
 */

#include <stdio.h>
#include <usb.h>
#include <string.h>
#include "hw_defs.h"
#include "utils.h"
#include "bluetooth_layer.h"
#include "j_protocol.h"
#include "j_command_manager.h"
#include <pthread.h>
#include <termios.h>

GQueue *recvQueue;

void printMenu(void) {
	printf("\t\t-->Menu<--\n\n");
	printf("\te: Enable USART\n");
	printf("\tr: Send byte on USART\n");
	printf("\t8: Hello World with printf\n");
	printf("\t9: Hello World with putrs\n");
	printf("\t7: Read USART\n");
	printf("\tl: Led swap\n");
	printf("\ts: Switch status\n");
	printf("\tw: Enable PWM2\n");
	printf("\tc: Close PWM2\n");
	printf("\tz: Enable PWM1\n");
	printf("\tc: Close PWM1\n");
	printf("\t+: Dir1 HIGH\n");
	printf("\t-: Dir1 LOW\n");
	printf("\th: Dir2 HIGH\n");
	printf("\t.: Dir2 LOW\n");
	printf("\t1: Encoder 1\n");
	printf("\t2: Encoder 2\n");
	printf("\tj: Tolerance\n");
	printf("\tb: Read Accelerometer register (0x3A)\n");
	printf("\tn: Write Accelerometer register (0x3A)\n");
	printf("\tu: Read Termometer register (0x90)\n");
	printf("\tx: Write Termometer register (0x90)\n");
	printf("\tu: Read RTC register (0xD0)\n");
	printf("\tx: Write RTC register (0xD0)\n");
	printf("\tq: Quit\n");
	printf("\nCommand: ");
}

void printMenuBt(void) {
	printf("\t\t-->Menu<--\n\n");
	printf("\tc: Motors conf\n");
	printf("\tm: Move motors\n");
	printf("\tr: Read I2C Register\n");
	printf("\tw: Write I2C Register\n");
	printf("\tt: LCD Init\n");
	printf("\tl: LCD Print\n");
	printf("\tp: PWM Test (20, 40)\n");
	printf("\tq: Quit\n");
	printf("\tCommand: ");
}

void *recv_thread(void *arg) {
	int *socket = (int*)arg;
	printf("recv on socket %d\n", *socket);
	
	int len;
	char *data;
	
	while(1)
	data = j_recv(*socket, &len);
	
}

int main(int argc,char **argv) {
	if(argc!=2) {
		printf("Usage: johnnyCon <usb|bt>\n");
		return -1;
	}
	
	if(strcmp(argv[1], "bt")==0) {
		u_int4 period;
		u_int4 duty;
		u_int4 tolerance;
		u_int4 enc1, enc2;
		u_int1 dir1, dir2;
		u_int4 reg, adr, dat, pwm1, duty1;
		u_int1 data[J_DATA_LEN];
		j_packet *p;
//		int start = 1;
//		u_int4 out = ((0xFF000000 & (packet->data[start+0]<<3))+(0x00FF0000 & (packet->data[start+1]<<2))+(0x0000FF00 & (packet->data[start+2]<<1))+(0x000000FF & (packet->data[start+3])));
//		printf("sizeof: %d, u_int4: %lu\n", sizeof(u_int4), out);
//		return 0;

	
		int socket=bt_connect(BT_JOHNNY);
		/*if(socket<=0) {
			printf("[johnnyCon] Error while connecting to Johnny\n");
			return -1;
		}*/
		
		pthread_t *recv_t;
		recv_t = (pthread_t*)malloc(sizeof(pthread_t));
		pthread_create(recv_t, NULL, recv_thread, (void*)(&socket));
		
	/*	enc1 = 255;
		enc2 = 255;
		dir1 = J_DIR1_CLOAK;
		dir2 = J_DIR2_CLOAK;
		while(1) {
//			j_blink(socket);
//			j_motors_conf(socket, 50,40,4);
			//sleep(20);
//			sleep(4);
			j_move_motors(socket, enc1, enc2, dir1, dir2);
			sleep(10);
		}*/
		char ch=0, n, n1;
		int d;
		while(ch!='q') {
			printMenuBt();
			scanf("%c", &ch);
			scanf("%c", &n);
			switch(ch) {
				case 'c':
					printf("Period, Duty, Tolerance\n");
					scanf("%u,%u,%u", &period, &duty, &tolerance);
					scanf("%c", &n1);
					printf("Period %u, Duty %u, Tolerance %u\n", period, duty, tolerance);
					j_motors_conf(socket, period, duty, tolerance);
					break;
				case 'm':
					printf("Enc1, Enc2\n");
					scanf("%u,%u", &enc1, &enc2);
					scanf("%c", &n);
					printf("Dir1: \n\t1: Cloak \n\t2: Anticloak\n");
					scanf("%d", &d);
					scanf("%c", &n1);
					if(d==1)
						dir1=J_DIR1_CLOAK;
					else if(d==2)
						dir1=J_DIR1_ANTICLOAK;
					
					printf("Dir2: \n\t1: Cloak \n\t2: Anticloak\n");
					scanf("%d", &d);
					scanf("%c", &n1);
					if(d==1)
						dir2=J_DIR2_CLOAK;
					else if(d==2)
						dir2=J_DIR2_ANTICLOAK;
						
					printf("Enc1 %u, Enc2 %u, Dir1 0x%x, Dir2 0x%x\n", enc1, enc2, dir1, dir2);
					j_move_motors(socket, enc1, enc2, dir1, dir2);
					break;
				case 'r':
					printf("Address: ");
					scanf("%x", &adr);
					scanf("%c", &n1);
					printf("Register: ");
					scanf("%x", &reg);
					scanf("%c", &n1);
					data[0] = 0x30;
					data[1] = adr;
					data[2] = reg;
					p = j_create_packet(J_COMMAND, data, 3);
					j_send(socket, p, J_PACKET_LEN);
					break;
				case 'w':
					printf("Address: ");
					scanf("%x", &adr);
					scanf("%c", &n1);
					printf("Register: ");
					scanf("%x", &reg);
					scanf("%c", &n1);
					printf("Data: ");
					scanf("%x", &dat);
					scanf("%c", &n1);
					printf("Address: 0x%x, Register: 0x%x, Data: 0x%x\n", adr, reg, dat);
					data[0] = 0x31;
					data[1] = adr;
					data[2] = reg;
					data[3] = dat;
					p = j_create_packet(J_COMMAND, data, 4);
					j_send(socket, p, J_PACKET_LEN);
					break;
				case 't':
					data[0] = J_LCD_INIT;
					p = j_create_packet(J_COMMAND, data, 1);
					j_send(socket, p, J_PACKET_LEN);
					break;
				case 'l':
					printf("String: ");
					u_int1 *iter = data;
					iter++;
					scanf("%s", iter);
					scanf("%c", &n1);
					data[0] = J_LCD_PRINT;
					p = j_create_packet(J_COMMAND, data, strlen(data));
					j_send(socket, p, J_PACKET_LEN);
					break;
				case 'p':
					data[0] = J_PWM_TEST;
					printf("PWM: ");
					scanf("%u", &pwm1);
					scanf("%c", &n1);
					printf("Duty: ");
					scanf("%u", &duty1);
					scanf("%c", &n1);
					printf("PWM: %u, Duty: %u\n", pwm1, duty1);
					p = j_create_packet(J_COMMAND, data, 10);
					SET_UINT4(pwm1, p, 1);
					SET_UINT4(duty1, p, 5);
					j_send(socket, p, J_PACKET_LEN);
					break;
				default:
					break;
					
			}
			//j_send(socket, bin, J_PACKET_LEN);
//			printf("p %u\n", var);
			//char *data = (char*)g_queue_pop_head(recvQueue);
			//if(data!=NULL)
			//	printf("pop: %s\n", hex_dump(data, 1, 10));
		}
		
		bt_close(socket);
	
	}
	else if(strcmp(argv[1], "usb")==0) {
		struct usb_dev_handle *j_handle = NULL;
		struct usb_device *j_device = NULL;
		int send_status;
		int open_status;
		unsigned char send_data[64];
		memset(send_data, 0, 64);
		unsigned char receive_data[64];
		memset(receive_data, 0, 64);

		usb_init();
		usb_set_debug(3);
	
		unsigned char located = 0;
		struct usb_bus *bus;
		struct usb_device *dev;
		usb_dev_handle *device_handle = 0;

		usb_find_busses();
		usb_find_devices();
	 
	 	for (bus = usb_busses; bus; bus = bus->next)
		{
			for (dev = bus->devices; dev; dev = dev->next)	
			{
				if (dev->descriptor.idVendor == DEV_VEND_ID) 
				{	
					located++;
					device_handle = usb_open(dev);
					j_device = dev;
					j_handle = device_handle;
					printf("MCU Device Found @ Address %s \n", dev->filename);
					printf("MCU Vendor ID 0x0%x\n",dev->descriptor.idVendor);
					printf("MCU Product ID 0x0%x\n",dev->descriptor.idProduct);
					printf("MCU Num Conf %d\n", dev->descriptor.bNumConfigurations);
				}
				else {
					//printf("** usb device %s found **\n", dev->filename);
				}
			}	
		}

		if(j_device!=NULL && j_handle!=NULL) {

			open_status = usb_set_configuration(j_handle, j_device->config[0].bConfigurationValue);
			printf("conf_stat=%d\n",open_status);
	
			open_status = usb_claim_interface(j_handle,0);
			printf("claim_stat=%d\n",open_status);
	
			open_status = usb_set_altinterface(j_handle,0);
			printf("alt_stat=%d\n",open_status);

			char ch=0, n;
			while(ch!='q') {
				printMenu();
				scanf("%c", &ch);
				scanf("%c", &n);
				memset(send_data, 0, 64);
				memset(receive_data, 0, 64);
				if(ch=='l')
					send_data[0]=0x09;
				else if(ch=='7')
					send_data[0]=0x04;
				else if(ch=='8') {
					send_data[0]=0x05;
				}
				else if(ch=='9') {
					send_data[0]=0x06;
				}
				else if(ch=='e') {
					send_data[0]=0x07;
					unsigned int val;
					scanf("%x", &val);
					scanf("%c", &n);
					send_data[1]=val;
				}
				else if(ch=='r')
					send_data[0]=0x08;
				else if(ch=='s')
					send_data[0]=0x81;
				else if(ch=='1')
					send_data[0]=0x83;
				else if(ch=='2')
					send_data[0]=0x82;
				else if(ch=='p') {
					send_data[0]=0x84;
					unsigned int val;
					printf("Step: ");
					scanf("%x", &val);
					scanf("%c", &n);
					send_data[1]=val;
					send_data[2]=val;
				}
				else if(ch=='w') {
					unsigned int val = 1023;
					unsigned int period = 255;
					printf("Period 8bit: ");
					scanf("%u", &period);
					scanf("%c", &n);
					printf("\nPeriod -> %u\n", period);
					printf("Duty 10bit: ");
					scanf("%u", &val);
					scanf("%c", &n);
					printf("\nDuty -> %u\n", val);
					send_data[0]=0x12;
					send_data[1]=period;
					send_data[2]=val & 0xFF;
					send_data[3]=((val & 0x300)>>8);
				}
				else if(ch=='z') {
					unsigned int val = 1023;
					unsigned int period = 255;
					printf("Period 8bit: ");
					scanf("%u", &period);
					scanf("%c", &n);
					printf("\nPeriod -> %u\n", period);
					printf("Duty 10bit: ");
					scanf("%u", &val);
					scanf("%c", &n);
					printf("\nDuty -> %u\n", val);
					send_data[0]=0x10;
					send_data[1]=period;
					send_data[2]=val & 0xFF;
					send_data[3]=((val & 0x300)>>8);
				}
				else if(ch=='+') {
					send_data[0]=0x14;
				}
				else if(ch=='-') {
					send_data[0]=0x15;
				}
				else if(ch=='h') {
					send_data[0]=0x16;
				}
				else if(ch=='.') {
					send_data[0]=0x17;
				}
				else if(ch=='j') {
					send_data[0]=0x90;
					unsigned int val = 1023;
					printf("Tolerance: ");
					scanf("%u", &val);
					scanf("%c", &n);
					send_data[1]=val;
					printf("Period: ");
					scanf("%u", &val);
					scanf("%c", &n);
					send_data[2]=val;
					printf("Duty: ");
					scanf("%u", &val);
					scanf("%c", &n);
					send_data[3]=val;

				}
				else if(ch=='c') {
					send_data[0]=0x11;
				}
				else if(ch=='i') {
					send_data[0]=0x20;
				}
				else if(ch=='a') {
					send_data[0]=0x22;
				}
				else if(ch=='b') {
					unsigned int val;
					printf("Register: ");
					scanf("%x", &val);
					scanf("%c", &n);
					printf("Reg: 0x%x\n", val);
					send_data[0]=0x23;
					send_data[1]=val;
				}
				else if(ch=='n') {
					unsigned int val;
					unsigned int data;
					printf("Register: ");
					scanf("%x", &val);
					scanf("%c", &n);
					printf("Data: ");
					scanf("%x", &data);
					scanf("%c", &n);
					printf("Reg: 0x%x, Data: 0x%x\n", val, data);
					send_data[0]=0x24;
					send_data[1]=val;
					send_data[2]=data;
				}
				else if(ch=='j')
					send_data[0]=0x21;
				else if(ch=='u') {
					unsigned int val;
					unsigned int ind;
					printf("Address: ");
					scanf("%x", &ind);
					scanf("%c", &n);
					printf("Register: ");
					scanf("%x", &val);
					scanf("%c", &n);
					printf("Reg: 0x%x\n", val);
					send_data[0]=0x30;
					send_data[1]=ind;
					send_data[2]=val;
				}
				else if(ch=='x') {
					unsigned int val;
					unsigned int data;
					unsigned int ind;
					printf("Address: ");
					scanf("%x", &ind);
					scanf("%c", &n);
					printf("Register: ");
					scanf("%x", &val);
					scanf("%c", &n);
					printf("Data: ");
					scanf("%x", &data);
					scanf("%c", &n);
					printf("Address: 0x%x, Reg: 0x%x, Data: 0x%x\n", ind, val, data);
					send_data[0]=0x31;
					send_data[1]=ind;
					send_data[2]=val;
					send_data[3]=data;
				}
				else {
					continue;
				}

				send_status=usb_interrupt_write(j_handle, 1, &send_data, 64, 500);
				if(send_status<0)
					printf("Error TX\n");
				if(usb_interrupt_read(j_handle, 1, &receive_data, 64, 1000)<0)
					printf("Error RX\n");

				printf("Received hex_dump:\n%s", hex_dump(receive_data, 64, 10));
				if(ch=='1'|| ch=='2') {
					unsigned int enc = 0;
					enc += receive_data[0];
					enc += (receive_data[1]<<8);
					printf("enc: %u step\n", enc);
				}
			}
			//printf("RX stat=%d -> RX char=%d\n",send_status,receive_data);

			/* Write the data2send bytes to the 7-segs */
		  /*  	send_status = usb_control_msg(xsv_handle,0x20,0x09,0x0200,0x0001,send_data,2,500); 
			printf("TX stat=%d\n",send_status);
			usleep(10000);
		  */ 
			/* Read the bytes that were just sent to the 7-segs */
		  /*	
		    	send_status = usb_control_msg(xsv_handle,0xA0,0x01,0x0100,0x0001,receive_data,2,500); 
			printf("RX stat=%d data=0x%x,0x%x\n",send_status,receive_data[0],receive_data[1]);
			usleep(10000);
		  */	
			usb_close(j_handle);
		}
		else {
			printf("Connect Johnny, please..\n");
		}
	}
	return 0;
}	

