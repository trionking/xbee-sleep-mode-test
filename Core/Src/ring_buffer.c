/*
 * ring_buffer.c
 *
 *  Created on: Jan 28, 2026
 *      Author: trion
 */

#include "main.h"
#include "ring_buffer.h"
#include "stdlib.h"

void InitQueue(Queue *queue, uint16_t q_size)
{
    queue->buf_size = q_size;
    queue->buf = malloc(q_size + 10);
    queue->front = queue->rear = 0;
}

void flush_queue(Queue *queue)
{
    queue->front = queue->rear = 0;
}

uint16_t next_q(Queue *queue, uint16_t q_cnt)
{
    return ((q_cnt + 1) % (queue->buf_size));
}

int IsFull(Queue *queue)
{
    return (next_q(queue, queue->rear) == queue->front);
}

int IsEmpty(Queue *queue)
{
    return (queue->front == queue->rear);
}

void Enqueue(Queue *queue, uint8_t data)
{
    uint8_t dummy;

    (void)dummy;

    if (IsFull(queue))
    {
        dummy = Dequeue(queue);
    }
    queue->buf[queue->rear] = data;
    queue->rear = next_q(queue, queue->rear);
}

void Enqueue_bytes(Queue *queue, uint8_t *data, uint32_t q_Len)
{
    for (uint32_t i = 0; q_Len > 0; q_Len--, i++)
    {
        Enqueue(queue, data[i]);
    }
}

uint8_t Dequeue(Queue *queue)
{
    uint8_t re = 0;
    if (IsEmpty(queue))
    {
        return re;
    }
    re = queue->buf[queue->front];
    queue->front = next_q(queue, queue->front);
    return re;
}

void Dequeue_bytes(Queue *src_queue, uint8_t *dst_buff, uint32_t q_Len)
{
    for (uint32_t i = 0; q_Len > 0; q_Len--)
    {
        dst_buff[i++] = Dequeue(src_queue);
    }
}

uint8_t Cuqueue(Queue *queue)
{
    uint8_t re = 0;
    if (IsEmpty(queue))
    {
        return re;
    }
    re = queue->buf[queue->front];
    return re;
}

uint16_t Len_queue(Queue *queue)
{
    return ((queue->buf_size - queue->front + queue->rear) % (queue->buf_size));
}
