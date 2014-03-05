#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "server.h"

struct acs_server as;

/* Struct for holding entity status */
struct entity {
  unsigned int		last_rpt_time;
  struct acs_position	last_rpt_pos;
  unsigned char		shootable;
  
  char			src_ip[16];
  unsigned short	src_port;
};
#define MAX_ENTITY 10
struct entity entities[MAX_ENTITY];  /* Index = ID (0 to MAX_ENTITY-1) */
pthread_mutex_t ent_mut = PTHREAD_MUTEX_INITIALIZER;


/* Max allowable elapsed time before server ignores last known position */
#define MAX_TELEM_LATENCY 10000  /* 10000 ms = 10 seconds */



char * sprint_time(unsigned int t) {
  int ms, s, m, h;
  static char tm[20];
  
  ms = t % 1000;
  t /= 1000;
  s = t % 60;
  t /= 60;
  m = t % 60;
  t /= 60;
  h = t % 24;  /* Assume time never exceeds a day */
  
  snprintf(tm, 20, "%02u:%02u:%02u.%03u", h, m, s, ms);
  return tm;
}



/* Separate thread for periodically sending sense reports */
void * send_sense_reports(void *args) {
  struct acs_report_sense sen;
  struct acs_sense rpts[MAX_ENTITY];
  sen.sense = rpts;
  
  while (1) {
    /* Sleep for some time */
    usleep(1000000);  /* 1 second */
    
    /* Get lock on storage */
    if (pthread_mutex_lock(&ent_mut) != 0) {
      printf("Locking error\n");
      continue;
    }
    
    /* Set header fields */
    sen.g_time = acs_server_gettime(&as);
    sen.s_cnt = 0;
    
    /* Determine the time before which client state is too stale */
    unsigned int stale = (sen.g_time > MAX_TELEM_LATENCY)
    		       ? (sen.g_time - MAX_TELEM_LATENCY)
		       : 0;
    
    /* Iterate through entities */
    unsigned short i;
    for (i=1; i<MAX_ENTITY; i++) {
      /* If entity is populated and is "recent enough," add it to report */
      if ((entities[i].last_rpt_time > 0) &&
          (entities[i].last_rpt_time > stale)) {
        sen.sense[sen.s_cnt].id = i;
        sen.sense[sen.s_cnt].shootable = entities[i].shootable;
	
	/* Calculate telem time offset from sense message time */
	sen.sense[sen.s_cnt].time_offset 
	  = sen.g_time - entities[i].last_rpt_time;
        
	memcpy(&(sen.sense[sen.s_cnt].pos),
	       &(entities[i].last_rpt_pos), sizeof(struct acs_position));
        
	sen.s_cnt++;
      }
    }
    
    /* Unlock storage */
    pthread_mutex_unlock(&ent_mut);
    
    /* Note: since this message is what sets the game clock on clients,
       there is a bootstrapping problem (they must sense before they
       can send correct telem, but the conditional below requires telem
       before any sense message is ever sent. Currently fixed by 
       letting clients send telem with "null" time... */
    
    /* Send the report to all entities that have reported in, ever */
    int cli_cnt = 0;
    for (i=1; i<MAX_ENTITY; i++) {
      if (entities[i].src_port) {
        acs_server_send_sense(&as, entities[i].src_ip, 
			      entities[i].src_port, &sen);
	cli_cnt++;
      }
    }
    printf("%s Sent %u reports to %d clients\n", 
           sprint_time(sen.g_time), sen.s_cnt, cli_cnt);
    for (i=0; i<sen.s_cnt; i++) {
      printf("  id %u at game time %s\n", sen.sense[i].id, 
	     sprint_time(sen.g_time - sen.sense[i].time_offset));
  }
  }
}



/* Callback function for handling received telemetry */
void handle_telem(struct acs_report_telem *tlm, char *ip, unsigned short port) {
  /* Make sure entity ID is within range of storage */
  if ((tlm->id < 1) || (tlm->id > MAX_ENTITY-1)) {
    printf("Bad client ID\n");
    return;
  }
  
  /* Get lock on storage */
  if (pthread_mutex_lock(&ent_mut) != 0) {
    printf("Locking error\n");
    return;
  }
  
  /* Check if timestamp on message is newer than last known position */
  unsigned short i = tlm->id;
  if (entities[i].last_rpt_time <= tlm->g_time) {
    printf("%s Got telem from %u at %s\n", 
           sprint_time(tlm->g_time), tlm->id, ip);
    
    /* Copy elements over */
    entities[i].last_rpt_time = tlm->g_time;
    entities[i].shootable = 1;  /* TODO: Treat this with hits/penalties */
    memcpy(&(entities[i].last_rpt_pos), 
           &(tlm->pos), sizeof(struct acs_position));
    
    /* Update client IP and port */
    strcpy(entities[i].src_ip, ip);
    entities[i].src_port = port;
  }
  
  /* Unlock storage */
  pthread_mutex_unlock(&ent_mut);
}

/* Placeholder callback function for handling received fires messages */
void handle_fires(struct acs_report_fires *frs, char *ip, unsigned short port) {
  printf("%s Fire! from %u\n", sprint_time(frs->g_time), frs->id);
}



int main(int argc, char **argv) {
  /* Initialize state */
  memset(entities, 0, sizeof(struct entity) * MAX_ENTITY);
  
  if (argc != 2) {
    printf("Usage: %s <recv-port>\n", argv[0]);
    return -1;
  }
  
  /* Initialize server socket 
     Note: Assumes all clients using same send/recv ports */
  if (acs_server_init(&as, atoi(argv[1])) == -1) {
    printf("Error initializing socket\n");
    return -1;
  }
  
  printf("\nListening for incoming messages..\n\n");
  printf("Will send sense reports to all clients that have reported in;\n");
  printf("reports will contain aggregated telem < %u seconds old.\n\n", 
         MAX_TELEM_LATENCY / 1000);
  
  /* Initialize sense reporting thread */
  pthread_t sense_thread_id;
  if (pthread_create(&sense_thread_id, NULL, 
  		     send_sense_reports, NULL) != 0) {
    printf("Error initializing sense report thread\n");
    return -1;
  } else {
    pthread_detach(sense_thread_id);
  }
  
  /* If this succeeds, will never return */
  /* Note: if send/recv have thread issues, add mutexes to sockets in net.c */
  if (acs_server_recv_loop(&as, handle_telem, handle_fires) == -1) {
    printf("Error establishing listener\n");
    return -1;
  }
  
  /* Hopefully never get here */
  return 0;
}
