#if !defined(EXTERN)
#define EXTERN 1
#endif	/* EXT */

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

enum RCV_STATE {RCV_IDLE, RCV_GET_LEN, RCV_GET_DATA, RCV_TEXT};

typedef struct S_RTK_DATA
{
 RCV_STATE state;
 unsigned int t0;
 uint64_t startTime;
 uint32_t crc;
 int count;
 int len;
 int fil;
 char buf[1024];
 unsigned int t0Accum;
 int rxAccum;
 int rxCount;
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
 char cons;
 char sVid;
 char elv;
 char az;
 char freqs;
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
 boolean update;
} T_GPS_INFO, *P_GPS_INFO;

#if EXTERN

#define EXT extern

extern char cons[5];
extern const char *names[4];

#else

#define EXT

char cons[5] = "PLBA";
const char *names[] = {"GPS", "GLO", "BDS", "GAL"};

#endif

EXT uint32_t crcBuf[1024];

EXT T_RTK_DATA rtk;

EXT T_GPS_INFO gpsInfo;

EXT S_SAT_DATA satData[MAX_SAT];
EXT int satIndex;

void printHex(const uint8_t *data, size_t len);

char* nextArg(char* p0);
char *getNum(char *p0, int n, int *result);
int getNum(char **p0, int n);
int getNum(char **p0);
int getHex(char **p0);

void buildCRC24qTable();
inline uint32_t crc24(uint32_t crc, unsigned char c);
EXT uint32_t crc24qTable[256];

void processRemData(void *data, size_t len);
void processSerial();
void gpsLoc();
void gpsSat();

#if defined(RTK_SEND)
bool sendBinary(const uint8_t *data, size_t len);
#endif	/* RTK_SEND */
