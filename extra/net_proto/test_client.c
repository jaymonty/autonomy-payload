#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "client.h"

struct acs_client ac;

/* Handle receiving of sensing messages */

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

void handle_sense(struct acs_report_sense *sen) {
  printf("Sensed %u entities at game time %s\n",
         sen->s_cnt, sprint_time(sen->g_time));
  
  unsigned int i;
  for (i=0; i<sen->s_cnt; i++) {
    printf("  id %u at game time %s\n", sen->sense[i].id, 
	   sprint_time(sen->g_time - sen->sense[i].time_offset));
  }
}

void * sense_thread(void *args) {
  if (acs_client_recv_loop(&ac, handle_sense) == -1) {
    printf("Error establishing listener\n");
  }
  return NULL;
}

/* Main function and sending of telem */

int main(int argc, char **argv) {
  if (argc != 5) {
    printf("Usage: %s <ID> <server-ip> <send-port> <recv-port>\n", argv[0]);
    return -1;
  }
  
  if (acs_client_init(&ac, atoi(argv[1]), argv[2], 
                      atoi(argv[3]), atoi(argv[4])) == -1) {
    printf("Error initializing\n");
    return -1;
  }
  
  printf("\nPress <Enter> to send an initial registering message\n");
  
  /* Attempt to fork sense-listening thread */
  pthread_t sense_thread_id;
  if (pthread_create(&sense_thread_id, NULL, 
  		     &sense_thread, NULL) != 0) {
    printf("Error initializing sensing thread\n");
    return -1;
  } else {
    pthread_detach(sense_thread_id);
  }
  
  /* Prep a dummy telem message */
  struct acs_report_telem t;
  memset(&t, 0x0, sizeof(struct acs_report_telem));
  t.pos.ns	= 0x11111111;
  t.pos.ew	= 0x22222222;
  t.pos.alt	= 0x33333333;
  t.pos.roll	= 0x4444;
  t.pos.pitch	= 0x5555;
  t.pos.yaw	= 0x6666;
  
  /* Do telem messages each time the user presses T */
  char c;
  while ((c = getchar()) != 0) {
    if (c == 0x0a) {
      /* Get current game time */
      t.g_time	= acs_client_gettime(&ac);
      
      /* Send telem message */
      int sres =  acs_client_send_telem(&ac, &t);
      
      if (!sres) {
        printf("Failed to send message, check IP and port settings\n\n");
      } else if (!t.g_time) {
        printf("Sent initial telem to register IP/Port, should receive sense reports now");
	printf("\n\nPress <Enter> to send a telemetry message\n\n");
      } else {
        printf("Sent telem at %s\n\n", sprint_time(t.g_time));
      }
    }
  }
  
  /* Hopefully never get here */
  return 0;
}
