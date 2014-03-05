/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* client.c - client-side (UAS-side) networking
* 
* Notes:
*  - 
**********************************************************************/
#include "client.h"

/* Initialize client */
int acs_client_init(struct acs_client *ac, unsigned short id,
		    char *srv_ip, unsigned short srv_port, 
		    unsigned short loc_port) {
  
  /* Basic sanity checks */
  if ((!ac) || (!id) || (!srv_ip) || (!srv_port) || (!loc_port)) return -1;
  
  /* Set ID and init sequence numbering */
  ac->id = id;
  ac->seq = 0;  /* TODO: Add logic to handle this rolling over */
  
  /* Initialize networking */
  strcpy(ac->srv_ip, srv_ip);
  ac->srv_port = srv_port;
  ac->loc_port = loc_port;
  if (acs_net_open(&(ac->net)) == -1)
    return -1;
  
  /* Set game start time to 0 until first server message */
  ac->start_time = 0;
  
  return 0;
}



/* Produce number of milliseconds into the game */
unsigned int acs_client_gettime(struct acs_client *ac) {
  /* Note: there is contention between threads for access to 
     ac->start_time. Right now this doesn't seem to be an issue, 
     but might need a mutex if things go awry. */
  
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  /* Returns 0 if start_time is not yet set */
  if (ac->start_time)
    return ((tv.tv_sec - ac->start_time) * 1000) + (tv.tv_usec / 1000);
  else
    return 0;
}



/* Send a single telem message to the server */
int acs_client_send_telem(struct acs_client *ac,
			  struct acs_report_telem *tlm) {
  if ((!ac) || (!tlm)) return -1;
  
  /* If game start time is not set, can't send valid message 
     (Unneeded for now) */
  /* if (!(ac->start_time)) return -1; */
  
  /* Make some space for built message */
  unsigned short buf_sz = 128;  /* TODO: set this from frame size */
  unsigned char buf[buf_sz];
  
  /* Take care of gimme fields */
  tlm->id = ac->id;
  tlm->seq = ac->seq;
  
  /* Attempt to build message frame */
  int bsz;
  if ((bsz = acs_frame_build_telem(tlm, buf, buf_sz)) == -1)
    return -1;
  
  /* Send the frame, returning >0 on success and <0 on failure */
  return acs_net_send(&(ac->net), ac->srv_ip, ac->srv_port, buf, bsz);
}

/* Send a single firing message to the server */
int acs_client_send_fires(struct acs_client *ac,
			  struct acs_report_fires *frs) {
  if ((!ac) || (!frs)) return -1;
  
  /* If game start time is not set, can't send valid message 
     (Unneeded for now) */
  /* if (!(ac->start_time)) return -1; */
  
  /* Make some space for built message */
  unsigned short buf_sz = 128;  /* TODO: set this from frame size */
  unsigned char buf[buf_sz];
  
  /* Take care of gimme fields */
  frs->id = ac->id;
  frs->seq = ++(ac->seq);  /* Increment for reliable messages */
  
  /* Attempt to build message frame */
  int bsz;
  if ((bsz = acs_frame_build_fires(frs, buf, buf_sz)) == -1)
    return -1;
  
  /* TODO: Archive the fires message in case of a NACK */
  
  /* Send the frame, returning >0 on success and <0 on failure */
  return acs_net_send(&(ac->net), ac->srv_ip, ac->srv_port, buf, bsz);
}



/* Set up a listening loop for inbound messages */
int acs_client_recv_loop(struct acs_client *ac,
			 void (*hdl_sense)(struct acs_report_sense *)) {
  if ((!ac) || (!hdl_sense)) return -1;
  
  /* Make some space for receiving a message */
  unsigned short buf_sz = 4096;  /* TODO: make this number tunable */
  unsigned char buf[buf_sz];
  
  /* Make some space for storing de-marshaled structs */
  struct acs_report_nack nack;
  struct acs_report_sense sen;
  unsigned short max_sen_rep = 100;  /* TODO: make this number tunable */
  struct acs_sense sen_rep[max_sen_rep];
  sen.sense = sen_rep;
  
  /* Establish listener */
  if (acs_net_bind(&(ac->net), ac->loc_port) == -1)
    return -1;
  
  /* Listen forever */
  while (1) {
    int r_sz, type, recs;
    
    /* Attempt to receive a message */
    if ((r_sz = acs_net_recv(&(ac->net), buf, buf_sz)) < 0)
      continue;
    
    /* Handle based on message type */
    type = acs_frame_process(buf, r_sz);
    if (type == ACS_FRAME_TYPE_SENSE) {
      /* Make sure there is enough room for all sensing reports */
      if ((recs = acs_frame_getreccount(buf)) > max_sen_rep)
        continue;
      
      /* Fully dissect message */
      acs_frame_dissect_sense(&sen, buf);
      
      /* If local game start time is not set, fix that */
      if (!(ac->start_time))  /* Set start time locally */
        ac->start_time = sen.s_time;
      
      /* Finally, do callback */
      hdl_sense(&sen);  /* Handler function does NOT get to keep struct */
    } else if (type == ACS_FRAME_TYPE_NACK) {
      /* Dissect the NACK */
      acs_frame_dissect_nack(&nack, buf);
      
      /* TODO: Implement NACK */
      /* Idea: Check if the unreceived message is still buffered
         locally. If so, resend it as-is. If not, either ignore the
	 NACK or send a NACK back to indicate no resend will occur. */
    }
  }
}
