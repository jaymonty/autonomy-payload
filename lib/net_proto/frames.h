/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* frames.h - message framing and de-framing headers
* 
* Notes:
*  - 
**********************************************************************/
#ifndef __ACS_FRAMES
#define __ACS_FRAMES

#include <arpa/inet.h>

/*
*  Message field constants
*/

#define ACS_FRAME_VERSION	0x1

#define ACS_FRAME_TYPE_TELEM	0x1
#define ACS_FRAME_TYPE_SENSE	0x2
#define ACS_FRAME_TYPE_FIRES	0x3
#define ACS_FRAME_TYPE_NACK	0x4

/*
*  User presentations of messages
*/

struct acs_position {
  unsigned int		ns;	/* north-south position, in cm */
  unsigned int		ew;	/* east-west position, in cm */
  unsigned int		alt;	/* altitude, in cm */
  unsigned short	roll;	/* aircraft roll, in degree hundredths */
  unsigned short	pitch;	/* aircraft pitch, in degree hundredths */
  unsigned short	yaw;	/* aircraft yaw, in degree hundredths */
};

struct acs_sense {
  unsigned short	id;		/* ID of sensed aircraft */
  unsigned short	time_offset;	/* Sense time offset from message time */
  					/* (Max ~30 seconds) */
  struct acs_position	pos;		/* Sensed position */
  unsigned char		shootable;	/* Flag; 1=shootable */
};

struct acs_report_telem {
  unsigned int		g_time;		/* Game timestamp in ms */
  unsigned short	id;		/* ID of sender */
  unsigned short	seq;		/* Sequence number for reliable messages */
  struct acs_position	pos;		/* Entity position */
};

struct acs_report_fires {
  unsigned int		g_time;		/* Game timestamp in ms */
  unsigned short	id;		/* ID of sender */
  unsigned short	seq;		/* Sequence number for reliable messages */
  struct acs_position	pos;		/* Entity position */
};

/* TODO: Add structs for penalty/hit messages */

struct acs_report_nack {
  unsigned int		g_time;		/* Game timestamp in ms */
  unsigned short	id;		/* ID of sender */
  unsigned short	seq;		/* Sequence number for reliable messages */
};

struct acs_report_sense {
  unsigned int		s_time;		/* Start time (real) in sec */
  unsigned int		g_time;		/* Game timestamp in ms */
  unsigned short	id;		/* ID of sender */
  unsigned char		s_cnt;		/* Count of senses */
  struct acs_sense *	sense;		/* List of senses */
};

/*
*  Message framing functions
*/

int acs_frame_build_telem(struct acs_report_telem *rpt,
			  unsigned char *buf,
			  unsigned short sz);

int acs_frame_build_sense(struct acs_report_sense *rpt,
			  unsigned char *buf,
			  unsigned short sz);

int acs_frame_build_fires(struct acs_report_fires *rpt,
			  unsigned char *buf,
			  unsigned short sz);

int acs_frame_build_nack(struct acs_report_nack *rpt,
			 unsigned char *buf,
			 unsigned short sz);

/*
*  Message de-framing functions
*/

unsigned char acs_frame_getreccount(unsigned char *buf);

int acs_frame_process(unsigned char *buf, unsigned short sz);

void acs_frame_dissect_telem(struct acs_report_telem *rpt,
			     unsigned char *buf);

void acs_frame_dissect_sense(struct acs_report_sense *rpt,
			     unsigned char *buf);
			     
void acs_frame_dissect_fires(struct acs_report_fires *rpt,
			     unsigned char *buf);

void acs_frame_dissect_nack(struct acs_report_nack *rpt,
			    unsigned char *buf);

#endif
