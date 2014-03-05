/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* net.h - network send and receive headers
* 
* Notes:
*  - 
**********************************************************************/
#ifndef __ACS_NET
#define __ACS_NET

#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>

struct acs_net {
  int			sock;
  unsigned short	rcv_port;	/* Port bound for listening */
  socklen_t		rcv_sa_len;	/* Length of address (below) */
  struct sockaddr_in	rcv_sa;		/* SA used for receives */
  struct sockaddr_in	snd_sa;		/* SA ready for sending with,
  					   sans destination address */
};

int acs_net_open(struct acs_net *an);

int acs_net_bind(struct acs_net *an, unsigned short port);

void acs_net_close(struct acs_net *an);

int acs_net_send(struct acs_net *an, char *addr, unsigned int port,
		 unsigned char *buf, unsigned short sz);

int acs_net_recv(struct acs_net *an, unsigned char *buf, unsigned short sz);

void acs_net_getlastsrc(struct acs_net *an, char *ip, unsigned short *port);

#endif
