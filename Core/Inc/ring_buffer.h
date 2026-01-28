/*
 * ring_buffer.h
 *
 *  Created on: Jan 28, 2026
 *      Author: trion
 */

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "stdint.h"

typedef struct Queue
{
    uint8_t *buf;
    uint16_t front;
    uint16_t rear;
    uint16_t buf_size;
} Queue;


#ifdef __cplusplus
extern "C" {
#endif

void InitQueue(Queue *queue, uint16_t q_size);
void flush_queue(Queue *queue);
uint16_t next_q(Queue *queue, uint16_t q_cnt);
int IsFull(Queue *queue);
int IsEmpty(Queue *queue);
void Enqueue(Queue *queue, uint8_t data);
void Enqueue_bytes(Queue *queue, uint8_t *data, uint32_t q_Len);
uint8_t Dequeue(Queue *queue);
void Dequeue_bytes(Queue *src_queue, uint8_t *dst_buff, uint32_t q_Len);
uint8_t Cuqueue(Queue *queue);
uint16_t Len_queue(Queue *queue);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H__ */
