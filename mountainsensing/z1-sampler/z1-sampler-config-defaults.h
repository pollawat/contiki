/*#define SENSOR_INTERVAL 120
#define SENSOR_AVRIDS_COUNT 0
#define SENSOR_HASADC1 0
#define SENSOR_HASADC2 0
#define SENSOR_HASRAIN 0

#define POST_INTERVAL 180
#define POST_IP_COUNT 8
#define POST_IP0 0x2a01
#define POST_IP1 0x348
#define POST_IP2 0x24b
#define POST_IP3 0x2
#define POST_IP4 0x0
#define POST_IP5 0x0
#define POST_IP6 0x0
#define POST_IP7 0x1
#define POST_PORT 8081*/

//TODO: Move these into the sampler code/header now that they are externally settable.


/* **************************************************************************** */
/* ------------------------- Sensors Default Config --------------------------- */
/* **************************************************************************** */
#ifdef SAMPLER_CONFIG_SENSOR_INTERVAL
#define SENSOR_INTERVAL SAMPLER_CONFIG_SENSOR_INTERVAL
#else
#define SENSOR_INTERVAL 120
#endif

#ifdef SAMPLER_CONFIG_SENSOR_AVRIDS_COUNT
#define SENSOR_AVRIDS_COUNT SAMPLER_CONFIG_SENSOR_AVRIDS_COUNT
#else
#define SENSOR_AVRIDS_COUNT 0
#endif

#ifdef SAMPLER_CONFIG_SENSOR_HASADC1
#define SENSOR_HASADC1 SAMPLER_CONFIG_SENSOR_HASADC1
#else
#define SENSOR_HASADC1 0
#endif

#ifdef SAMPLER_CONFIG_SENSOR_HASADC2
#define SENSOR_HASADC2 SAMPLER_CONFIG_SENSOR_HASADC2
#else
#define SENSOR_HASADC2 0
#endif

#ifdef SAMPLER_CONFIG_SENSOR_HASRAIN
#define SENSOR_HASRAIN SAMPLER_CONFIG_SENSOR_HASRAIN
#else
#define SENSOR_HASRAIN 0
#endif


/* **************************************************************************** */
/* --------------------------- POST Default Config ---------------------------- */
/* **************************************************************************** */
#ifdef POST_CONFIG_INTERVAL
#define POST_INTERVAL POST_CONFIG_INTERVAL
#else
#define POST_INTERVAL 3600
#endif

#ifdef POST_CONFIG_IP_COUNT
#define POST_IP_COUNT POST_CONFIG_IP_COUNT
#else
#define POST_IP_COUNT 8
#endif

#ifdef POST_CONFIG_IP0
#define POST_IP0 POST_CONFIG_IP0
#else
#define POST_IP0 0xaaaa
#endif

#ifdef POST_CONFIG_IP1
#define POST_IP1 POST_CONFIG_IP1
#else
#define POST_IP1 0x0
#endif

#ifdef POST_CONFIG_IP2
#define POST_IP2 POST_CONFIG_IP2
#else
#define POST_IP2 0x0
#endif

#ifdef POST_CONFIG_IP3
#define POST_IP3 POST_CONFIG_IP3
#else
#define POST_IP3 0x0
#endif

#ifdef POST_CONFIG_IP4
#define POST_IP4 POST_CONFIG_IP4
#else
#define POST_IP4 0xc30c
#endif

#ifdef POST_CONFIG_IP5
#define POST_IP5 POST_CONFIG_IP5
#else
#define POST_IP5 0x0
#endif

#ifdef POST_CONFIG_IP6
#define POST_IP6 POST_CONFIG_IP6
#else
#define POST_IP6 0x0
#endif

#ifdef POST_CONFIG_IP7
#define POST_IP7 POST_CONFIG_IP7
#else
#define POST_IP7 0x1
#endif

#ifdef POST_CONFIG_PORT
#define POST_PORT POST_CONFIG_PORT
#else
#define POST_PORT 8081
#endif

