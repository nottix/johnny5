#include "utils.h"

/** 
 * Effettua il dump in esadecimale di un pacchetto.
 * Esempio di dump:
 *  50 4f 53 54 20 2a 20 48 54 54         POST * HTT
 *  50 2f 31 2e 31 0d 0a 55 73 65         P/1.1..Use
 *  72 2d 41 67 65 6e 74 3a 20 54         r-Agent: T
 *  6f 72 54 65 6c 6c 61 2f 30 2e         orTella/0.
 *  31 0d 0a 43 6f 6e 74 65 6e 74         1..Content    
 */
char *hex_dump(const char *packet, int len, int n)
{
        int i=0;
        int count = 0;
        int modulo =0;
        int div = len/n;
        int k = 0;
        int divtemp = div;
        if(len%n!=0) {
                modulo = n-(len%n);
                divtemp++;
        }

        int length =   (len*4+(divtemp)*4+(modulo)*3)*2;
        char *buffer = (char*)calloc(length, 1);
        char *strtemp = (char*)calloc(4, 1);

        strcat(buffer, "\n");

        if(len == 1) {
                sprintf(strtemp,"%02x ",(u_int1)packet[i]);
                strcat(buffer,strtemp);
                int templ = 0;
                templ = templ%n;

                for(k=n;k>templ+1;k--) {
                        sprintf(strtemp,"   ");
                        strcat(buffer,strtemp);
                }
                sprintf(strtemp,"\t  ");
                strcat(buffer,strtemp);
                sprintf(strtemp,"%c",packet[i]);
                strcat(buffer,strtemp); 
                sprintf(strtemp,"\n\n");
                strcat(buffer,strtemp);
                return buffer;
        }

        for(i=0;i<len;i++) {    
                
                if(i==len-1 && ((len)%n)!=0) {
                        if((len-1)%n==0) {
                                for(k=n;k>0;k--) {  
                                        sprintf(strtemp,"%02x ", (u_int1)packet[i-k]);
                                        strcat(buffer,strtemp);
                                }
                                sprintf(strtemp,"\t  ");
                                strcat(buffer,strtemp);
                                for(k=n;k>0;k--) {
                                        sprintf(strtemp,"%c", ((isprint(packet[i-k])!=0)?packet[i-k]:'.'));
                                        strcat(buffer,strtemp);
                                }
                                sprintf(strtemp,"\n");
                                strcat(buffer,strtemp);
                        }
                        int temp = 0;
                        for(k=div*n;k<len;k++) {
                                sprintf(strtemp,"%02x ",(u_int1)packet[k]);
                                strcat(buffer,strtemp);
                                temp = k;
                        }
                        
                        temp = temp%n;
                        for(k=n;k>temp+1;k--) {
                                sprintf(strtemp,"   ");
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\t  ");
                        strcat(buffer,strtemp);
                        for(k=div*n;k<len;k++) {  
                                sprintf(strtemp,"%c",((isprint(packet[k])!=0)?packet[k]:'.')); 
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\n");
                        strcat(buffer,strtemp);
                        break;
                }
                if(i==len-1 && (len)%n==0) {
                        for(k=count;k<len;k++) {
                                sprintf(strtemp,"%02x ",(u_int1)packet[k]);
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\t  ");
                        strcat(buffer,strtemp);
                        for(k=count;k<len;k++) {
                                sprintf(strtemp,"%c",((isprint(packet[k])!=0)?packet[k]:'.')); 
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\n");
                        strcat(buffer,strtemp);
                        break;
                }
                if(i!= 0 && i%n==0) {
                        count = i;
                        for(k=n;k>0;k--) {  
                                sprintf(strtemp,"%02x ", (u_int1)packet[i-k]);
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\t  ");
                        strcat(buffer,strtemp);
                        for(k=n;k>0;k--) {
                                sprintf(strtemp,"%c", ((isprint(packet[i-k])!=0)?packet[i-k]:'.'));
                                strcat(buffer,strtemp);
                        }
                        sprintf(strtemp,"\n");
                        strcat(buffer,strtemp);
                        continue;
                }

        }

        strcat(buffer, "\n");
        return buffer;
}

