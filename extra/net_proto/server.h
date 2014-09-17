/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* server.h - server-side (Arbiter-side) networking
* 
* Notes:
*  - 
**********************************************************************/
#ifndef _ACS_SERVER
#define _ACS_SERVER

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "frames.h"
#include "net.h"

struct acs_server {
  time_t		start_time;  /* Game time starts on the second */
  unsigned short	rem_port;
  unsigned short	loc_port;
  struct acs_net	net;
};

int acs_server_init(struct acs_server *as, unsigned short recv_port);

unsigned int acs_server_gettime(struct acs_server *as);

void acs_server_resettime(struct acs_server *as);

int acs_server_send_sense(struct acs_server *as, char *ip, unsigned int port,
			  struct acs_report_sense *sen);

int acs_server_recv_loop(struct acs_server *as,
			 void (*hdl_telem)(struct acs_report_telem *, 
			 		   char *, unsigned short),
			 void (*hdl_fires)(struct acs_report_fires *, 
			 		   char *, unsigned short));

#endif
