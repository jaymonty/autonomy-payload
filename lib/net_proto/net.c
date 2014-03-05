/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* net.c - network send and receive code
* 
* Notes:
*  - Everything in UDP
**********************************************************************/
#include "net.h"

/* Establish a socket and do some initialization */
int acs_net_open(struct acs_net *an) {
  if (!an) return -1;
  
  if ((an->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    return -1;
  
  memset(&(an->snd_sa), 0, sizeof(struct sockaddr_in));
  an->snd_sa.sin_family	= AF_INET;
  
  return 0;
}

/* Set up a listening port on the open socket */
int acs_net_bind(struct acs_net *an, unsigned short port) {
  if ((!an) || (!port)) return -1;
  
  an->rcv_port = port;
  
  struct sockaddr_in rcvr;
  memset(&rcvr, 0, sizeof(struct sockaddr_in));
  rcvr.sin_family	= AF_INET;
  rcvr.sin_port		= htons(an->rcv_port);
  rcvr.sin_addr.s_addr	= htonl(INADDR_ANY);
  if (bind(an->sock, 
  	   (struct sockaddr *)&rcvr, 
  	   sizeof(struct sockaddr_in)) == -1)
    return -1;
  
  return 0;
}

/* Close the socket */
void acs_net_close(struct acs_net *an) {
  if (an) {
    close(an->sock);
    memset(an, 0, sizeof(struct acs_net));
  }
  return;
}

/* Send a datagram to a specified IP at the pre-decided port */
int acs_net_send(struct acs_net *an, char *addr, unsigned int port,
		 unsigned char *buf, unsigned short sz) {
  if ((!an) || (!buf) || (!sz)) return -1;
  
  if (inet_aton(addr, &(an->snd_sa.sin_addr)) == 0)
    return -1;
  
  an->snd_sa.sin_port = htons(port);
  
  return sendto(an->sock, buf, sz, 0, 
  		(struct sockaddr *)&(an->snd_sa), sizeof(an->snd_sa));
}

/* Receive a datagram on the listening port */
int acs_net_recv(struct acs_net *an, unsigned char *buf, unsigned short sz) {
  if ((!an) || (!buf) || (!sz)) return -1;
  
  an->rcv_sa_len = sizeof(an->rcv_sa);
  return recvfrom(an->sock, buf, sz, 0, 
  		  (struct sockaddr *)&(an->rcv_sa), &(an->rcv_sa_len));
}

/* Return the source IP address and UDP port of the last received datagram */
void acs_net_getlastsrc(struct acs_net *an, char *ip, unsigned short *port) {
  if (!an || !ip || !port) return;
  
  strcpy(ip, inet_ntoa(an->rcv_sa.sin_addr));
  *port = ntohs(an->rcv_sa.sin_port);
  
  return;
}
