#include <stdlib.h>
#include "fifo.h"
#include "heap.h"

FIFO8_t *fifo8_init(uint32_t size) {
    size_t fifo_size = sizeof(FIFO8_t) + size * sizeof(uint8_t);

    FIFO8_t *fifo = (FIFO8_t*)malloc(fifo_size);
    if (fifo == NULL) return NULL;

    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
    return fifo;
}

int fifo8_put(FIFO8_t *fifo, uint8_t data) {
    if ((fifo->tail + 1) % fifo->size == fifo->head) {
        return -1; // FIFO full
    }
    fifo->data[fifo->tail] = data;
    fifo->tail = (fifo->tail + 1) % fifo->size;
    return 0;
}

int fifo8_get(FIFO8_t *fifo, uint8_t *data) {
    if (fifo->head == fifo->tail) // empty
        return -1; 

    *data = fifo->data[fifo->head];
    fifo->head = (fifo->head + 1) % fifo->size;

    return 0;
}

int fifo8_get_blocking(FIFO8_t *fifo, uint8_t *data) {
    while (fifo->head == fifo->tail); // empty

    *data = fifo->data[fifo->head];
    fifo->head = (fifo->head + 1) % fifo->size;

    return 0;
}

uint8_t fifo8_size(FIFO8_t *fifo) {
    if (fifo->tail >= fifo->head)
        return fifo->tail - fifo->head;
    else
        return fifo->size - (fifo->head - fifo->tail);
}
