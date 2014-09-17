/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* server.c - server-side (Arbiter-side) networking
* 
* Notes:
*  - 
**********************************************************************/
#include "server.h"

/* Initialize client */
int acs_server_init(struct acs_server *as, unsigned short recv_port) {
  if ((!as) || (!recv_port)) return -1;
  
  /* Initialize networking stuff */
  as->loc_port = recv_port;
  acs_net_open(&(as->net));
  
  /* Set game start time */
  as->start_time = time(NULL);
  
  return 0;
}



/* Produce number of milliseconds into the game */
unsigned int acs_server_gettime(struct acs_server *as) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  /* Returns 0 if start_time is not yet set */
  if (as->start_time)
    return ((tv.tv_sec - as->start_time) * 1000) + (tv.tv_usec / 1000);
  else
    return 0;
}

/* Reset server game time to start from current clock time
   Warning: clients are *not* required to re-sync their clocks, 
   only do this between games!!! */
void acs_server_resettime(struct acs_server *as) {
  as->start_time = time(NULL);
}



/* Send a single sense message to a specified client (by IP) */
/* TODO: use client ID rather than IP address (lookup table) */
int acs_server_send_sense(struct acs_server *as, char *ip, unsigned int port,
			  struct acs_report_sense *sen) {
  if ((!as) || (!sen)) return -1;
  
  /* Set gimme fields */
  sen->s_time = as->start_time;
  sen->id = 0;  /* Server is always ID 0 */
  
  /* Make some space for built message */
  unsigned short buf_sz = 1024;  /* TODO: set this from frame size */
  unsigned char buf[buf_sz];
  
  /* Attempt to build message frame */
  int bsz;
  if ((bsz = acs_frame_build_sense(sen, buf, buf_sz)) == -1)
    return -1;
  
  /* Send the frame, returning >0 on success and <0 on failure */
  return acs_net_send(&(as->net), ip, port, buf, bsz);
}



/* Set up a listening loop for incoming messages */
int acs_server_recv_loop(struct acs_server *as,
			 void (*hdl_telem)(struct acs_report_telem *, 
			 		   char *, unsigned short),
			 void (*hdl_fires)(struct acs_report_fires *, 
			 		   char *, unsigned short)) {
  if (!as) return -1;
  
  /* Make some space for receiving a message */
  unsigned short buf_sz = 1024;  /* TODO: make this number tunable */
  unsigned char buf[buf_sz];
  
  /* Make some space for storing the de-marshaled struct */
  struct acs_report_telem tlm;
  struct acs_report_fires frs;
  
  /* Establish listener */
  if (acs_net_bind(&(as->net), as->loc_port) == -1)
    return -1;
  
  /* Listen forever */
  while (1) {
    int r_sz, type;
    char src_addr[16];  /* XXX.XXX.XXX.XXX = 15 plus a null byte */
    unsigned short src_port;
    
    /* Attempt to receive a message */
    if ((r_sz = acs_net_recv(&(as->net), buf, buf_sz)) < 0)
      continue;
    
    /* TODO: Implement sequence number checking and NACK */
    /* Idea: NACK once if the latest sequence number is higher
       than the latest reliable (e.g. fires) message received.
       Need to track sequence numbers from all clients, and
       have some (timeout?) mechanism once NACK is sent. Client
       might NACK back if it cannot resend the message, in which
       case the server updates the sequence number and moves on.
       
       Note that if the Arbiter is waiting to evaluate fires until
       the NACK is processed, the code must be extended to allow
       the Arbiter to know the "safe" game time up to which it can
       evaluate everything. This may be determined by the determining
       the latest game time for each client where all reliable
       messages are known to be received, and taking the earliest
       from among those. */
    
    /* Determine message type, then dissect and send to handler */
    /* Note: Handler function does NOT get to keep struct */
    type = acs_frame_process(buf, r_sz);
    if ((type == ACS_FRAME_TYPE_TELEM) && (hdl_telem)) {
      /* Telem message from client */
      acs_frame_dissect_telem(&tlm, buf);
      acs_net_getlastsrc(&(as->net), src_addr, &src_port);
      hdl_telem(&tlm, src_addr, src_port);
    } else if ((type == ACS_FRAME_TYPE_FIRES) && (hdl_fires)) {
      /* Fires message from client */
      acs_frame_dissect_fires(&frs, buf);
      acs_net_getlastsrc(&(as->net), src_addr, &src_port);
      hdl_fires(&frs, src_addr, src_port);
    } else if (type == ACS_FRAME_TYPE_NACK) {
      /* NACK from client - negative response to server NACK */
    }
  }
}
