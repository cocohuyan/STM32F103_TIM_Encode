#include "SEQueue.h"

SEQUEUE g_Queue[1]={0};

void SetSQNULL(SEQUEUE *sq)
{
    int i=0;
    sq->front = 0;
    sq->rear  = 0;
    sq->len = 0;
    for(i=0;i<FIFO_LEN;i++)
    {
        sq->data[i]=0;
    }
}

int GetSQLength(SEQUEUE *sq)
{
    return sq->len;
}

//
int IsSQEmpty(SEQUEUE *sq)
{
    return (sq->len <=0) ? 1 : 0;
}

int EnSEQueue(SEQUEUE *sq, unsigned char ch)
{
    if( sq->len >= FIFO_LEN)
    {
        return 0;
    }

    sq->len++;
    sq->data[sq->rear++] = ch;
    sq->rear = ((sq->rear) >= FIFO_LEN) ? 0 : sq->rear;

    return 1;
}

unsigned char DeSEQueue(SEQUEUE *sq, unsigned char *info)
{
    if((IsSQEmpty(sq)) && (sq->front == sq->rear))
          return 0;    //

    *info = sq->data[sq->front++];
    sq->front = ((sq->front) >= FIFO_LEN) ? 0 : sq->front;
    sq->len--;
    sq->len = ((sq->len) <= 0) ? 0 : sq->len;            //
    return 1;
}

