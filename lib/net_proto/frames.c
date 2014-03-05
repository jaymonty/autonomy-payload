/**********************************************************************
* Network messaging library for ARSENL Aerial Combat Swarms
* Developed by Mike Clement, Winter 2013
* 
* frames.c - message framing and de-framing code
* 
* Notes:
*  - 
**********************************************************************/
#include "frames.h"



/*
*  Message field types
*/

typedef unsigned char		UINT_8;
typedef unsigned short		UINT_16;
typedef short			INT_16;
typedef unsigned int		UINT_32;



/*
*  Frame structures
*/

/* Common header fields */
struct _acs_frame_common {
  UINT_8	version;	/* Message version */
  UINT_8	type;		/* Message type */
  UINT_16	id;		/* ID of sender */
  UINT_32	recs_time;	/* 8-bit record count (sense msgs only) 
  				   and 24-bit game timestamp */
  UINT_16	seq;		/* Reliable message sequence number */
  UINT_16	cksum;		/* 16-bit XOR'd checksum */
};

#define _ACS_FRAME_COMMON_SZ sizeof(struct _acs_frame_common)

/* Position struct */
struct _acs_frame_pos {
  UINT_32	ns;	/* north-south position, in cm */
  UINT_32	ew;	/* east-west position, in cm */
  UINT_32	alt;	/* altitude, in cm */
  INT_16	roll;	/* roll angle in 1/100ths of degrees, +=right wing down */
  INT_16	pitch;	/* pitch angle in 1/100ths of degrees, +=nose up */
  INT_16	yaw;	/* yaw angle in 1/100ths of degrees, +=nose right */
};

#define _ACS_FRAME_POS_SZ sizeof(struct _acs_frame_pos)-2

/* Virtual sensor struct */
struct _acs_frame_sense {
  UINT_16	id;
  UINT_16	flags_offset;
  	/* MSB: shootable flag */
	/* Remainder: offset into past from message timestamp, in ms */
  struct _acs_frame_pos pos;
};

#define _ACS_FRAME_SENSE_SZ sizeof(struct _acs_frame_sense)-2



/*
*  Utility functions
*/

unsigned short _acs_frame_cksum(unsigned char *b, unsigned int s) {
  unsigned int i = 0;
  unsigned short ck = 0;
  /* XOR each full short (16 bit) value */
  while (i < s-1) {
    ck ^= *(unsigned short *)(b+i);
    i += sizeof(unsigned short);
  }
  /* Handle any stray byte */
  if (i < s) {
    unsigned short z = 0x00ff & b[i];
    ck ^= z;
  }
  return ck;
}



/*
*  Packet building functions
*/

/* Package a telem report into allocated buffer *buf of size sz */
int acs_frame_build_telem(struct acs_report_telem *rpt,
			  unsigned char *buf,
			  unsigned short sz) {
  if (!rpt) return -1;
  
  /* Make sure buffer is not NULL and is large enough */
  unsigned int out_sz = _ACS_FRAME_COMMON_SZ + _ACS_FRAME_POS_SZ;
  if ((!buf) || (sz < out_sz))
    return -1;
    
  unsigned char *ptr = buf;
  
  /* Package common header fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)ptr;
  afc->version	 = ACS_FRAME_VERSION;
  afc->type	 = ACS_FRAME_TYPE_TELEM;
  afc->id	 = htons(rpt->id);
  afc->recs_time = htonl(rpt->g_time & 0x00ffffff);
  afc->seq	 = htons(rpt->seq);
  afc->cksum	 = 0;		/* zeroize for checksum calculation */
  
  ptr += _ACS_FRAME_COMMON_SZ;
  
  /* Package telem fields */
  struct _acs_frame_pos *pos = (struct _acs_frame_pos *)ptr;
  pos->ns	= htonl(rpt->pos.ns);
  pos->ew	= htonl(rpt->pos.ew);
  pos->alt	= htonl(rpt->pos.alt);
  pos->roll	= htons(rpt->pos.roll);
  pos->pitch	= htons(rpt->pos.pitch);
  pos->yaw	= htons(rpt->pos.yaw);
  
  /* Compute and fill checksum */
  afc->cksum = htons(_acs_frame_cksum(buf, out_sz));
  return out_sz;
}

/* Package a firing report into allocated buffer *buf of size sz */
int acs_frame_build_fires(struct acs_report_fires *rpt,
			  unsigned char *buf,
			  unsigned short sz) {
  if (!rpt) return -1;
  
  /* Make sure buffer is not NULL and is large enough */
  unsigned int out_sz = _ACS_FRAME_COMMON_SZ + _ACS_FRAME_POS_SZ;
  if ((!buf) || (sz < out_sz))
    return -1;
    
  unsigned char *ptr = buf;
  
  /* Package common header fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)ptr;
  afc->version	 = ACS_FRAME_VERSION;
  afc->type	 = ACS_FRAME_TYPE_FIRES;
  afc->id	 = htons(rpt->id);
  afc->recs_time = htonl(rpt->g_time & 0x00ffffff);
  afc->seq	 = htons(rpt->seq);
  afc->cksum	 = 0;		/* zeroize for checksum calculation */
  
  ptr += _ACS_FRAME_COMMON_SZ;
  
  /* Package telem fields */
  struct _acs_frame_pos *pos = (struct _acs_frame_pos *)ptr;
  pos->ns	= htonl(rpt->pos.ns);
  pos->ew	= htonl(rpt->pos.ew);
  pos->alt	= htonl(rpt->pos.alt);
  pos->roll	= htons(rpt->pos.roll);
  pos->pitch	= htons(rpt->pos.pitch);
  pos->yaw	= htons(rpt->pos.yaw);
  
  /* Compute and fill checksum */
  afc->cksum = htons(_acs_frame_cksum(buf, out_sz));
  return out_sz;
}

/* Package a NACK into allocated buffer *buf of size sz */
int acs_frame_build_nack(struct acs_report_nack *rpt,
			 unsigned char *buf,
			 unsigned short sz) {
  if (!rpt) return -1;
  
  /* Make sure buffer is not NULL and is large enough */
  unsigned int out_sz = _ACS_FRAME_COMMON_SZ + _ACS_FRAME_POS_SZ;
  if ((!buf) || (sz < out_sz))
    return -1;
    
  unsigned char *ptr = buf;
  
  /* Package common header fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)ptr;
  afc->version	 = ACS_FRAME_VERSION;
  afc->type	 = ACS_FRAME_TYPE_NACK;
  afc->id	 = htons(rpt->id);
  afc->recs_time = htonl(rpt->g_time & 0x00ffffff);
  afc->seq	 = htons(rpt->seq);
  afc->cksum	 = 0;		/* zeroize for checksum calculation */
  
  /* Compute and fill checksum */
  afc->cksum = htons(_acs_frame_cksum(buf, out_sz));
  return out_sz;
}

/* Package a sense report into allocated buffer *buf of size sz */
int acs_frame_build_sense(struct acs_report_sense *rpt,
			  unsigned char *buf,
			  unsigned short sz) {
  if (!rpt) return -1;
  
  /* Make sure buffer is not NULL and is large enough */
  unsigned int out_sz = _ACS_FRAME_COMMON_SZ + 4  /* 4-byte start time */
  		      + (_ACS_FRAME_SENSE_SZ * rpt->s_cnt);
  if ((!buf) || (sz < out_sz))
    return -1;
  
  unsigned char *ptr = buf;
  
  /* Package common header fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)ptr;
  afc->version	 = ACS_FRAME_VERSION;
  afc->type	 = ACS_FRAME_TYPE_SENSE;
  afc->id	 = htons(rpt->id);
  afc->recs_time = htonl((rpt->g_time & 0x00ffffff) | (rpt->s_cnt << 24));
  afc->seq	 = 0;		/* unused for this message type */
  afc->cksum	 = 0x0000;	/* zeroize for checksum calculation */
  
  ptr += _ACS_FRAME_COMMON_SZ;
  
  /* Insert game start time in whole seconds */
  *(unsigned int *)ptr = htonl(rpt->s_time);
  ptr += sizeof(rpt->s_time);
  
  /* Package fields for each sense record */
  struct _acs_frame_sense *sen = (struct _acs_frame_sense *)ptr;
  unsigned char i;
  for (i=0; i<rpt->s_cnt; i++) {
    sen[i].id		= rpt->sense[i].id;
    sen[i].flags_offset	= htons((rpt->sense[i].shootable << 15) |
    			        (rpt->sense[i].time_offset & 0x7fff));
    
    sen[i].pos.ns	= htonl(rpt->sense[i].pos.ns);
    sen[i].pos.ew	= htonl(rpt->sense[i].pos.ew);
    sen[i].pos.alt	= htonl(rpt->sense[i].pos.alt);
    sen[i].pos.roll	= htons(rpt->sense[i].pos.roll);
    sen[i].pos.pitch	= htons(rpt->sense[i].pos.pitch);
    sen[i].pos.yaw	= htons(rpt->sense[i].pos.yaw);
  }
  
  /* Compute and fill checksum */
  afc->cksum = htons(_acs_frame_cksum(buf, out_sz));
  
  return out_sz;
}



/*
*  Packet dissecting functions
*/

/* For a sensing report, return the number of sense records */
unsigned char acs_frame_getreccount(unsigned char *buf) {
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  unsigned int rt = ntohl(afc->recs_time);
  unsigned char rc = 0x000000ff & (rt >> 24);
  return rc;
}

/* Pre-process an unknown report type, returning its type value */
int acs_frame_process(unsigned char *buf, unsigned short sz) {
  if (!buf) return -1;
  
  /* Perform basic checks on message */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  if (afc->version != ACS_FRAME_VERSION) return -1;
  if ((afc->type != ACS_FRAME_TYPE_TELEM) &&
      (afc->type != ACS_FRAME_TYPE_SENSE) &&
      (afc->type != ACS_FRAME_TYPE_FIRES)) return -1;
  
  /* Make sure length makes sense with message type/size */
  if (sz < _ACS_FRAME_COMMON_SZ) return -1;
  if (afc->type == ACS_FRAME_TYPE_TELEM)
    if (sz < (_ACS_FRAME_COMMON_SZ + _ACS_FRAME_POS_SZ))
      return -1;
  if (afc->type == ACS_FRAME_TYPE_SENSE)
    if (sz < (_ACS_FRAME_COMMON_SZ + 4  /* 4 bytes for start time */
              + (_ACS_FRAME_SENSE_SZ * acs_frame_getreccount(buf))))
      return -1;
  if (afc->type == ACS_FRAME_TYPE_FIRES)
    if (sz < (_ACS_FRAME_COMMON_SZ + _ACS_FRAME_POS_SZ))
      return -1;
  if (afc->type == ACS_FRAME_TYPE_NACK)
    if (sz < _ACS_FRAME_COMMON_SZ)
      return -1;
  
  /* Compute checksum */
  unsigned short cksum_msg = afc->cksum;
  afc->cksum = 0x0000;
  unsigned short cksum_cpt = _acs_frame_cksum(buf, sz);
  if (ntohs(cksum_msg) != cksum_cpt) return -1;
  afc->cksum = cksum_msg;  /* Restore the bytes */
  
  /* All is well, return the message type */
  return afc->type;
}

/* Dissect a *processed* telemetry report into an allocated struct */
void acs_frame_dissect_telem(struct acs_report_telem *rpt,
			     unsigned char *buf) {
  /* Extract common fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  rpt->id	= ntohs(afc->id);
  rpt->g_time	= ntohl(afc->recs_time) & 0x00ffffff;
  rpt->seq	= ntohs(afc->seq);
  
  buf += _ACS_FRAME_COMMON_SZ;
  
  /* Extract telem fields */
  struct _acs_frame_pos *pos = (struct _acs_frame_pos *)buf;
  rpt->pos.ns		= ntohl(pos->ns);
  rpt->pos.ew		= ntohl(pos->ew);
  rpt->pos.alt		= ntohl(pos->alt);
  rpt->pos.roll		= ntohs(pos->roll);
  rpt->pos.pitch	= ntohs(pos->pitch);
  rpt->pos.yaw		= ntohs(pos->yaw);
}

/* Dissect a *processed* fires report into an allocated struct */
void acs_frame_dissect_fires(struct acs_report_fires *rpt,
			     unsigned char *buf) {
  /* Extract common fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  rpt->id	= ntohs(afc->id);
  rpt->g_time	= ntohl(afc->recs_time) & 0x00ffffff;
  rpt->seq	= ntohs(afc->seq);
  
  buf += _ACS_FRAME_COMMON_SZ;
  
  /* Extract telem fields */
  struct _acs_frame_pos *pos = (struct _acs_frame_pos *)buf;
  rpt->pos.ns		= ntohl(pos->ns);
  rpt->pos.ew		= ntohl(pos->ew);
  rpt->pos.alt		= ntohl(pos->alt);
  rpt->pos.roll		= ntohs(pos->roll);
  rpt->pos.pitch	= ntohs(pos->pitch);
  rpt->pos.yaw		= ntohs(pos->yaw);
}

/* Dissect a *processed* NACK into an allocated struct */
void acs_frame_dissect_nack(struct acs_report_nack *rpt,
			    unsigned char *buf) {
  /* Extract common fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  rpt->id	= ntohs(afc->id);
  rpt->g_time	= ntohl(afc->recs_time) & 0x00ffffff;
  rpt->seq	= ntohs(afc->seq);
}

/* Dissect a *processed* sensing report into an allocated struct,
   assuming *sense allocated with enough room already */
void acs_frame_dissect_sense(struct acs_report_sense *rpt,
			     unsigned char *buf) {
  /* Extract common fields */
  struct _acs_frame_common *afc = (struct _acs_frame_common *)buf;
  rpt->id	= ntohs(afc->id);
  rpt->g_time	= ntohl(afc->recs_time) & 0x00ffffff;
  rpt->s_cnt	= acs_frame_getreccount(buf);
  
  buf += _ACS_FRAME_COMMON_SZ;
  
  rpt->s_time	= ntohl(*(unsigned int *)buf);
  buf += sizeof(rpt->s_time);
  
  struct _acs_frame_sense *sen = (struct _acs_frame_sense *)buf;
  unsigned char i;
  for (i=0; i<rpt->s_cnt; i++) {
    rpt->sense[i].id		= sen[i].id;
    rpt->sense[i].shootable	= ntohs(sen[i].flags_offset) >> 15;
    rpt->sense[i].time_offset	= ntohs(sen[i].flags_offset) & 0x7fff;
    
    rpt->sense[i].pos.ns	= ntohl(sen[i].pos.ns);
    rpt->sense[i].pos.ew	= ntohl(sen[i].pos.ew);
    rpt->sense[i].pos.alt	= ntohl(sen[i].pos.alt);
    rpt->sense[i].pos.roll	= ntohs(sen[i].pos.roll);
    rpt->sense[i].pos.pitch	= ntohs(sen[i].pos.pitch);
    rpt->sense[i].pos.yaw	= ntohs(sen[i].pos.yaw);
  }
}
