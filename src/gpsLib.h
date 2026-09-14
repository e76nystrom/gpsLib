#if !defined(GPS_LIB_H)
#define GPS_LIB_H

#if defined(ARDUINO)

#include "dbgPin.h"

void dbgInit();
#include "soc/gpio_reg.h"

inline void dbg0Set()
{
 // GPIO.out_w1ts.val = (1 << DBG0_PIN));
 REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG0_PIN));
}

inline void dbg0Clr()
{
 // GPIO.out_w1tc.val = (1 << DBG0_PIN));
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG0_PIN));
}

inline void dbg1Set()
{
 // GPIO.out_w1ts.val = (1 << DBG1_PIN));
 REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG1_PIN));
}

inline void dbg1Clr()
{
 // GPIO.out_w1tc.val = (1 << DBG1_PIN));
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG1_PIN));
}

#else

#if defined(ESP_PLATFORM)

#include "soc/gpio_reg.h"

#define DBG0_PIN 2
#define DBG1_PIN 4

inline void dbg0Set()
{
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG0_PIN));
}

inline void dbg0Clr()
{
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG0_PIN));
}

inline void dbg1Set()
{
 REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG1_PIN));
}

inline void dbg1Clr()
{
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG1_PIN));
}

#endif	/* ESP_PLATFORM */

#if defined(PICO_BUILD)

#include "hardware/structs/sio.h"

#define sio_hw ((sio_hw_t *)SIO_BASE)

inline void dbg0Set()
{
 sio_hw->gpio_set = (1 << DBG0_PIN);
}

inline void dbg0Clr()
{
 sio_hw->gpio_clr = (1 << DBG0_PIN);
}

inline void dbg1Set()
{
 sio_hw->gpio_set = (1 << DBG1_PIN);
}

inline void dbg1Clr()
{
 sio_hw->gpio_clr = (1 << DBG1_PIN);
}

inline uint32_t usTime()
{
 return timer_hw->timerawl;
}

#endif	/* PICO_BUILD */

#endif	/* ARDUINO */

enum RCV_STATE {RCV_IDLE, RCV_GET_LEN, RCV_GET_DATA, RCV_TEXT};

constexpr size_t RTK_BUF_SIZE = 1024;

typedef struct S_RTK_DATA
{
 RCV_STATE state;
 unsigned int t0;
 uint64_t startTime;
 uint32_t crc;
 int count;
 int len;
 int fil;
 char buf[RTK_BUF_SIZE];
 unsigned int t0Accum;
 int rxAccum;
 int rxCount;
 unsigned int tData;
 int numSv;
 unsigned int svTmr;
 int svCount[4];
} T_RTK_DATA, *P_RTK_DATA;

#define MAX_SIG 4
#define MAX_SAT 100

typedef struct S_FREQ_INFO
{
 char freq;
 char cno;
} T_FREQ_INFO, *P_FREQ_INFO;

typedef struct S_SAT_DATA
{
 int cons;
 char sVid;
 char elv;
 char az;
 int freqs;
 T_FREQ_INFO sig[MAX_SIG];
} T_SAT_DATA, *P_SAT_DATA;

#define TIME_LEN 12

typedef struct S_GPS_INFO
{
 char timeBuf[TIME_LEN];
 int gpsTime;
 double lat;
 double lon;
 char fix;
 char sats;
 bool update;
} T_GPS_INFO, *P_GPS_INFO;

inline char cons[5] = "PLBA";
inline const char *names[] = {"GPS", "GLO", "BDS", "GAL"};

inline uint32_t crcBuf[1024];

#if defined(DBG_PRT)
inline int prt;
#endif

inline T_RTK_DATA rtk;

inline T_GPS_INFO gpsInfo;

inline S_SAT_DATA satData[MAX_SAT];
inline int satIndex;

void printHex(const uint8_t *data, size_t len);

char* nextArg(char* p0);
char *getNum(char *p0, int n, int *result);
int getNum(char **p0, int n);
int getNum(char **p0);
int getHex(char **p0);

inline uint32_t crc24qTable[256];

void buildCRC24qTable();
inline uint32_t crc24(uint32_t crc, unsigned char c)
{
 return ((crc << 8) ^ crc24qTable[((crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
}

void pollSerial();

#if defined(ARDUINO)
#define PROCESS_SERIAL processSerial()
#else

#if defined(ESP_PLATFORM)
#define PROCESS_SERIAL processSerial(int sock, char *buf, size_t len)
#endif	/* ESP_PLATFORM */

#if defined(PICO_BUILD)
#define PROCESS_SERIAL processSerial(int sock)
#endif	/* PICO_BUILD */

#endif  /* ARDUINO */

void PROCESS_SERIAL;
void processRemData(void *data, size_t len);
void gpsLoc();
void gpsSat();

#if defined(RTK_SEND)
bool sendBinary(const uint8_t *data, size_t len);
#endif	/* RTK_SEND */

#endif	/* GPS_LIB_H */
