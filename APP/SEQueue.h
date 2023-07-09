#ifndef __SEQUEUE_H_
#define __SEQUEUE_H_

#define FIFO_LEN	4000		//FIFO长度为4000

typedef struct{
	unsigned char data[FIFO_LEN];
	volatile int front;
	volatile int rear;
	volatile int len;
}SEQUEUE;

extern SEQUEUE g_Queue[1];

void SetSQNULL(SEQUEUE *sq);
int GetSQLength(SEQUEUE *sq);
int IsSQEmpty(SEQUEUE *sq);
int EnSEQueue(SEQUEUE *sq, unsigned char ch);
unsigned char DeSEQueue(SEQUEUE *sq, unsigned char *info);

#endif

