#ifndef FILENAMES_H
#define FILENAMES_H
#include <stdint.h>
#define FILENAME_PREFIX 'r'
#define FILENAME_LENGTH 8
void filenames_init(void);
uint8_t filenames_next_read(char*);
uint8_t filenames_next_write(char*);
void filenames_delete(char*);
#endif
