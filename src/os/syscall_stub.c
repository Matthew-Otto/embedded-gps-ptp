#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>
#include "uart.h"  // optional, for printf redirection

// Provide heap boundaries (linker script must define _end or _heap_start/_heap_end)
extern uint8_t _end;  // symbol from linker script
static uint8_t *heap_end;

int _write(int file, char *ptr, int len) {
    return 0;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    return 0;
}

int _isatty(int file) {
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

void *_sbrk(ptrdiff_t incr) {
    return 0;
}

int _kill(int pid, int sig) {
    return -1;
}

int _getpid(void) {
    return 0;
}

void _exit(int status) {
    return;
}