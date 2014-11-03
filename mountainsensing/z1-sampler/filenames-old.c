
// General
#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cfs/cfs.h"

#include "filenames-old.h"

//#define FILEDEBUG
#ifdef FILEDEBUG
    #define FILEDEBUG(...) printf(__VA_ARGS__)
#else
    #define FILEDEBUG(...)
#endif


/*
 * Returns the filename that is to be read for POSTing
 * Returns NULL if no files can be POSTed
 */
uint8_t
get_next_read_filename(char *filename)
{

  struct cfs_dirent dirent;
  struct cfs_dir dir;

  if(cfs_opendir(&dir, "/") == 0) {
    while(cfs_readdir(&dir, &dirent) != -1) {
      if(strncmp(dirent.name, "r_", 2) == 0) {
        strcpy(filename, dirent.name);
        return 1;
      }
    }
  }
  FILEDEBUG("[NEXT] No file found. NULL\n");
  return 0;
}

/*
 * Returns the filename of the next file where it is safe to store `length` bytes of data
 */
uint8_t
get_next_write_filename(char *filename)
{
  FILEDEBUG("get_next_write_filename\n");
  struct cfs_dirent dirent;
  struct cfs_dir dir;
  uint16_t file_num;
  uint16_t max_num;
  file_num = 0;
  max_num = 0;

  filename[0] = 'r';
  filename[1] = '_';

  if(cfs_opendir(&dir, "/") == 0) {
    FILEDEBUG("\tOpened folder\n");
    while(cfs_readdir(&dir, &dirent) != -1) {
      if(strncmp(dirent.name, "r_", 2) == 0) {
        file_num = atoi(dirent.name + 2);
        if(file_num > max_num) {
          max_num = file_num;
          //file_size = (uint16_t)dirent.size;
          FILEDEBUG("Filename %d found\n", file_num);
        }
        FILEDEBUG("\tMax: %d Filenum: %d\n", max_num, file_num);
      }
    }
    if(max_num == 0) {
      FILEDEBUG("\tNo previous files found\n");
      filename[2] = '1';
      filename[3] = 0;
    }else{
      FILEDEBUG("\t Previous file %d\n", max_num);
      itoa(max_num + 1, filename + 2, 10);
    }

    FILEDEBUG("Returning %s\n", filename);
    return 1;
  }else{
    FILEDEBUG("[ERROR] UNABLE TO OPEN ROOT DIRECTORY!!!\n");
    return 0;
  }
}