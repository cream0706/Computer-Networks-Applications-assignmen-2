#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
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

static struct pkt buffer[SEQSPACE];  /* cache all sent but unacknowledged packets */
static bool acked[SEQSPACE];         /* track whether each packet has been ACKed */
static int base = 0;                 /* current window starting point */
static int A_nextseqnum = 0;         /* next sequence number to be sent */
static int timer_index = -1;         /* the current timer monitors the packet sequence number */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( (A_nextseqnum + SEQSPACE - base) % SEQSPACE < WINDOWSIZE) {
 
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    buffer[A_nextseqnum] = sendpkt;
    acked[A_nextseqnum] = false;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (base == A_nextseqnum) {
      starttimer(A, RTT);
      timer_index = A_nextseqnum;
    }

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
  int ack;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    ack = packet.acknum;
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;
    
    if (!acked[ack]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n",packet.acknum);
      new_ACKs++;
      acked[ack] = true;

      while (acked[base]) {
        acked[base] = false;
        base = (base + 1) % SEQSPACE;
      }

      stoptimer(A);

      if (base != A_nextseqnum) {
        timer_index = base;
        starttimer(A, RTT);
      } else {
        timer_index = -1;
      }
        }
        else
          if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  
  if (TRACE > 0){
    printf("----A: time out,resend packets!\n");
  }
    if (timer_index != -1 && !acked[timer_index]) {
      printf ("---A: resending packet %d\n", (buffer[timer_index]).seqnum);
      tolayer3(A,buffer[timer_index]);
      packets_resent++;
      starttimer(A,RTT);
    }
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  A_nextseqnum = 0;
  base = 0;
  timer_index = -1;

  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = false;
  }
}



/********* Receiver (B)  variables and procedures ************/

static struct pkt recv_buffer[SEQSPACE];  /* buffer for out-of-order packets */
static bool received[SEQSPACE];           /* whether a packet is buffered */
static int expectedseqnum;                /* the sequence number expected next by the receiver */
static int B_nextseqnum;                  /* the sequence number for the next packets sent by B */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;

  /* if not corrupted and received packet is in order */
  if (!IsCorrupted(packet) &&
      ((packet.seqnum - expectedseqnum + SEQSPACE) % SEQSPACE < WINDOWSIZE)) {

    if (!received[packet.seqnum]) {
      recv_buffer[packet.seqnum] = packet;
      received[packet.seqnum] = true;
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      packets_received++;
    }
/*
  if (packet.seqnum == expectedseqnum && TRACE > 0) {
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
  }
*/
    while (received[expectedseqnum]) {
      
      tolayer5(B, recv_buffer[expectedseqnum].payload);
      received[expectedseqnum] = false;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }

    sendpkt.acknum = packet.seqnum;

  }
  else {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0){
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    }

    if (expectedseqnum == 0)
      sendpkt.acknum = SEQSPACE - 1;
    else
      sendpkt.acknum = expectedseqnum - 1;
  }

  /* create packet */
  sendpkt.seqnum = 0;

  /* we don't have any data to send.  fill payload with 0's */
  for ( i=0; i<20 ; i++ )
    sendpkt.payload[i] = '0';

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt);

  /* send out packet */
  tolayer3 (B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i, j;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = false;
    recv_buffer[i].seqnum = -1;
    recv_buffer[i].acknum = -1;
    recv_buffer[i].checksum = -1;
    for (j = 0; j < 20; j++) {
      recv_buffer[i].payload[j] = 0;
    }
  }
}

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
