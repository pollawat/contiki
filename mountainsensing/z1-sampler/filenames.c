#include "filenames.h"

#include "contiki.h"
#include "cfs/cfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>



#define FILENAME_DEBUG
#ifdef FILENAME_DEBUG
    #define FPRINT(...) printf(__VA_ARGS__)
#else
    #define FPRINT(...) 
#endif

static uint16_t number;

void 
filenames_init(void){
    FPRINT("Initing filename system\n");
    struct cfs_dirent dirent;
    struct cfs_dir dir;
    uint16_t file_num;
    uint16_t max_num;
    max_num = 0;
    if(cfs_opendir(&dir, "/") == 0) {
        FPRINT("\tOpened folder\n");
        while(cfs_readdir(&dir, &dirent) != -1) {
          if(strncmp(dirent.name, FILENAME_PREFIX, 1) == 0) {
            file_num = atoi(dirent.name + 1);
            FPRINT("Filename %d found\n", file_num);
            FPRINT("\tMax: %d Filenum: %d\n", max_num, file_num);
            if(file_num > max_num) {
              max_num = file_num;
            }
          }
        }
        if(max_num == 0) {
            FPRINT("\tNo previous files found\n");
            number = 0;
        }else{
            FPRINT("\tPrevious files found.  Highest number = %d\n", max_num);
            number = max_num;
        }
    }
}

/*
 * Returns 0 for no files
 * Returns 1 is files available
 */
uint8_t
filenames_next_read(char* filename){
    int fd, fd1;
    if(number == 0){
        //no files
        return 0;
    }
    filename[0] = FILENAME_PREFIX;
    itoa(number, filename + 1, 10);
    FPRINT("Next read filename = %s\n", filename);
    fd = cfs_open(filename, CFS_READ);
    fd1 = fd;
    cfs_close(fd);
    if(fd1 < 0){
        printf("File doesn't exist\n");
        printf("Reinitialising\n");
        filenames_init();
        filenames_next_read(filename);
    }
    return 1;
}


/*
 * Something needs doing with this return status
 */
uint8_t
filenames_next_write(char* filename){
    uint16_t next;
    next = ++number;
    filename[0] = FILENAME_PREFIX;
    itoa(next, filename +1, 10);
    FPRINT("Next write filename = %s\n", filename);
    return 1;
}

void
filenames_delete(char* filename){
    char fname[FILENAME_LENGTH];
    filenames_next_read(fname);
    if (strcmp(filename,fname) == 0){

        number--;
        FPRINT("Number decremented\n");
    }else{
        printf("Number mismatch not deleting\n");
    }
}
