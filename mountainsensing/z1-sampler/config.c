
#include "config.h"





#if SensorConfig_size > PostConfig_size
    #define CONFIG_BUF_SIZE (SensorConfig_size + 4)
#else
    #define CONFIG_BUF_SIZE (POSTConfig_size + 4)
#endif

#define CONFIG_DEBUG

#ifdef CONFIG_DEBUG 
    #define CPRINT(...) printf(__VA_ARGS__)
#else
    #define CPRINT(...)
#endif


void 
print_sensor_config(SensorConfig *conf)
{
  uint8_t i;
  CPRINT("\tInterval = %d\n", (unsigned int)conf->interval);
  CPRINT("\tADC1: ");
  if (conf->hasADC1 == 1){
    CPRINT("yes\n");
  }else{
    CPRINT("no\n");
  }
  CPRINT("\tADC2: ");
  if (conf-> hasADC2){
    CPRINT("yes\n");
  }else{
    CPRINT("no\n");
  }
  CPRINT("\tRain: ");
  if (conf-> hasRain){
    CPRINT("yes\n");
  }else{
    CPRINT("no\n");
  }
  CPRINT("\tAVRs: ");
  if(conf->avrIDs_count ==0 ){
    CPRINT("NONE\n");
  }else{
    for(i=0; i < conf->avrIDs_count; i++){
      CPRINT("%d", (int)conf->avrIDs[i] & 0xFF);
      if(i < conf->avrIDs_count -1){
        CPRINT(", ");
      }else{
        CPRINT("\n");
      }
    }
  }
}


/*
 * Sets the Sampler configuration (writes it to flash).
 * Returns 0 upon success, 1 on failure
 */
uint8_t 
set_config(void *pb, uint8_t config)
{
    uint8_t cfg_buf[CONFIG_BUF_SIZE];
    pb_ostream_t ostream;
    int write;
    bool encode_status;

    memset(cfg_buf, 0, CONFIG_BUF_SIZE);
    ostream = pb_ostream_from_buffer(cfg_buf, CONFIG_BUF_SIZE);
    if(config == SAMPLE_CONFIG) {
        encode_status = pb_encode_delimited(&ostream, SensorConfig_fields, (SensorConfig *)pb);
        cfs_remove("sampleconfig");
        write = cfs_open("sampleconfig", CFS_WRITE);
        CPRINT("Saving the following details to config file\n");
        print_sensor_config((SensorConfig *)pb);
        

    } else if (config == COMMS_CONFIG){
       
        CPRINT("Saving the following details to config file\n");
        CPRINT("\tInterval: %d\n", (unsigned int)((POSTConfig *)pb)->interval);
        CPRINT("\tPort: %d\n", (unsigned int)((POSTConfig *)pb)->port);
        CPRINT("\tPosting to %x:%x:%x:%x:%x:%x:%x:%x\n", 
            (unsigned int)((POSTConfig *)pb)->ip[0], (unsigned int)((POSTConfig *)pb)->ip[1], (unsigned int)((POSTConfig *)pb)->ip[2],
            (unsigned int)((POSTConfig *)pb)->ip[3], (unsigned int)((POSTConfig *)pb)->ip[4], (unsigned int)((POSTConfig *)pb)->ip[5],
            (unsigned int)((POSTConfig *)pb)->ip[6], (unsigned int)((POSTConfig *)pb)->ip[7]);
       encode_status = pb_encode_delimited(&ostream, POSTConfig_fields, (POSTConfig *)pb);
       cfs_remove("commsconfig");
       write = cfs_open("commsconfig", CFS_WRITE);
   }else{
      printf("UNKNOWN CONFIG TYPE. UNABLE TO WRITE FILE!\n");
      return 1;
   }
    if(encode_status){
      if(write != -1) {
        cfs_write(write, cfg_buf, ostream.bytes_written);
        cfs_close(write);
        CPRINT("[WCFG] Writing %d bytes to config file\n", ostream.bytes_written);
        return 0;
      } else {
         CPRINT("[WCFG] ERROR: could not write to disk\n");
         return 1;
      }
    }else{
      printf("Failed to encode file. Leaving config as is\n");
      return 1;
    }
    
}


/*
 * Get the Sampler config (reads it from flash).
 * Returns 0 upon success, 1 upon failure
 */
uint8_t 
get_config(void* pb, uint8_t config)
{
    int read;
    uint8_t cfg_buf[CONFIG_BUF_SIZE];
    pb_istream_t istream;

    memset(cfg_buf, 0, CONFIG_BUF_SIZE);
  
   if(config == SAMPLE_CONFIG) {
        CPRINT("[RCFG] Opening `sampleconfig`\n");
        read = cfs_open("sampleconfig", CFS_READ);
        CPRINT("[RCFG] Opened\n");
    } else {
        CPRINT("[RCFG] Opening `commsconfig`\n");
        read = cfs_open("commsconfig", CFS_READ);
    }
    CPRINT("[RCFG] Attmepting to read\n");
    if(read != -1) {
      CPRINT("[RCFG] Reading...\n");
      cfs_read(read, cfg_buf, CONFIG_BUF_SIZE);
      cfs_close(read);

      istream = pb_istream_from_buffer(cfg_buf, CONFIG_BUF_SIZE);

      CPRINT("[RCFG] Bytes left = %d\n", istream.bytes_left);

      if(config == SAMPLE_CONFIG) {
          pb_decode_delimited(&istream, SensorConfig_fields, (SensorConfig *)pb);
          CPRINT("Read sensor config:\n");
          print_sensor_config((SensorConfig *)pb);
      } else {
          pb_decode_delimited(&istream, POSTConfig_fields, (POSTConfig *)pb);
      }
      return 0;
    } else {
        CPRINT("[RCFG] ERROR: could not read from disk\n");
        return 1;
    }
}