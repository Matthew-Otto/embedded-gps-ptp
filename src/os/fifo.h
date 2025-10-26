#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct {
    uint32_t head;
    uint32_t tail;
    uint32_t size;
    uint8_t  data[];
} FIFO8_t;

FIFO8_t* fifo8_init(uint32_t size);
int fifo8_put(FIFO8_t *fifo, uint8_t data);
int fifo8_get(FIFO8_t *fifo, uint8_t *data);
int fifo8_get_blocking(FIFO8_t *fifo, uint8_t *data);
uint8_t fifo8_size(FIFO8_t *fifo);

#endif // FIFO_H