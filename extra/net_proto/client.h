/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* client.h - client-side networking
* 
* Notes:
*  - 
**********************************************************************/
#ifndef _ACS_CLIENT
#define _ACS_CLIENT

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "frames.h"
#include "net.h"

struct acs_client {
  time_t		start_time;  /* Game time starts on the second */
  unsigned short	srv_port;
  unsigned short	loc_port;
  unsigned short	id;
  unsigned short	seq;
  struct acs_net	net;
  char			srv_ip[16];
};

int acs_client_init(struct acs_client *ac, unsigned short id,
		    char *srv_ip, unsigned short srv_port, 
		    unsigned short loc_port);

unsigned int acs_client_gettime(struct acs_client *ac);

int acs_client_send_telem(struct acs_client *ac,
			  struct acs_report_telem *tlm);

int acs_client_send_fires(struct acs_client *ac,
			  struct acs_report_fires *frs);

int acs_client_recv_loop(struct acs_client *ac,
			 void (*hdl_sense)(struct acs_report_sense *));

#endif
