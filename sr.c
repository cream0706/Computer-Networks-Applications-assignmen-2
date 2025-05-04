#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 8      /* Selective Repeat sequence space */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static int A_base;                      /* sequence number of oldest unacked pkt */
static int A_nextseqnum;                /* next sequence number to use */
static struct pkt buffer[SEQSPACE];     /* buffer for sent but unacked pkts */
static bool buf_valid[SEQSPACE];        /* marks valid buffered slots */
static bool buf_acked[SEQSPACE];        /* marks which buffered pkts have been ACKed */

/* called once before any other A routines are called */
void A_init(void)
{
  int i;
  A_base = 0;
  A_nextseqnum = 0;
  for (i = 0; i < SEQSPACE; i++) {
    buf_valid[i] = false;
    buf_acked[i] = false;
  }
}

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  int window_count = (A_nextseqnum - A_base + SEQSPACE) % SEQSPACE;
  int i;

  if ( windowcount < WINDOWSIZE) {
    struct pkt sendpkt;
    /* prepare packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = 0;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    tolayer3(A, sendpkt);
    
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    

        /* SR: buffer the packet for potential retransmission */
      buffer[A_nextseqnum] = sendpkt;
      buf_valid[A_nextseqnum] = true;
      buf_acked[A_nextseqnum] = false;

    /* start timer if first packet in window */
    if (A_base == A_nextseqnum)
      starttimer(A, 16.0);

   /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{  

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {

    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    int rel = (packet.acknum - A_base + SEQSPACE) % SEQSPACE;
    
    if (rel < WINDOWSIZE && buf_valid[packet.acknum]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n",packet.acknum);
      
      buf_acked[packet.acknum] = true;         /* SR: mark ACKed */
      new_ACKs++;


      while (buf_valid[A_base] && buf_acked[A_base]) {
        buf_valid[A_base] = false;
        buf_acked[A_base] = false;
        A_base = (A_base + 1) % SEQSPACE;
      }


      if (A_base != A_nextseqnum) {
        stoptimer(A);
        starttimer(A, 16.0);
      } else {
        stoptimer(A);
      }
        }
        else
          if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else{
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
  }
}
  

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  int count = (A_nextseqnum - A_base + SEQSPACE) % SEQSPACE;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");
  
  for (i = 0; i < count; i++) {
    int s = (A_base + i) % SEQSPACE;
    if (buf_valid[s] && !buf_acked[s]) {
      tolayer3(A, buffer[s]);
      packets_resent++;
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", (buffer[timer_index]).seqnum);

    }
  }
  /* SR: restart timer for oldest outstanding */
  starttimer(A, 16.0);
}


/********* Receiver (B)  variables and procedures ************/

static int B_base;                      /* lower edge of receiver window */
static int B_nextseqnum;                /* seqnum field for ACK packets */
static struct pkt B_buffer[SEQSPACE];   /* buffer for out-of-order pkts */
static bool B_bufreceived[SEQSPACE];    /* marks which slots have been received */

void B_init(void)
{
    int i;
    B_base = 0;
    B_nextseqnum = 0;
    for (i = 0; i < SEQSPACE; i++)
        B_bufreceived[i] = false;
}
/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int rel, i;

  /* if not corrupted and received packet is in order */
  if (!IsCorrupted(packet)){
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);

        rel = (packet.seqnum - B_base + SEQSPACE) % SEQSPACE;
        if (rel < WINDOWSIZE) {
          /* SR: buffer out-of-order */
          if (!B_bufreceived[packet.seqnum]) {
            B_buffer[packet.seqnum] = packet;
            B_bufreceived[packet.seqnum] = true;
          }
          /* SR: deliver all in-order */
        while (B_bufreceived[B_base]) {
          tolayer5(B, B_buffer[B_base].payload);
          B_bufreceived[B_base] = false;
          B_base = (B_base + 1) % SEQSPACE;
        }
      }
  }
  else {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }
  /* send ACK for last in-order */
  sendpkt.acknum = (B_base + SEQSPACE - 1) % SEQSPACE;
  sendpkt.seqnum = B_nextseqnum;
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '\0';
  sendpkt.checksum = ComputeChecksum(sendpkt);
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */


/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
