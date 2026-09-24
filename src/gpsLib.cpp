#if defined(ARDUINO)

#include "cfg.h"

#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "dbgPin.h"

#else

#if defined(ESP_PLATFORM)

#include <cstdio>
#include <cstdint>
#include <cstring>
typedef uint8_t u_int8_t;

#include "esp_timer.h"
#include "driver/uart.h"

uint32_t millis()
{
 return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

inline uint32_t micros()
{
 return static_cast<uint32_t>(esp_timer_get_time());
}

#endif	/* ESP_PLATFORM */

#if defined(PICO_BUILD)

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
typedef uint8_t u_int8_t;

#include "hardware/timer.h"
#include "hardware/uart.h"

uint32_t millis()
{
 return static_cast<uint32_t>(timer_time_us_64(timer_hw) / 1000ULL);
}

inline uint32_t micros()
{
 return static_cast<uint32_t>(timer_time_us_64(timer_hw));
}

#endif	/* PICO_BUILD */

#endif	/* ARDUINO */

#include "gpsLib.h"

#if defined(ARDUINO)

void dbgInit()
{
 pinMode(DBG0_PIN, OUTPUT);
 pinMode(DBG1_PIN, OUTPUT);
}

#else

#if defined(ESP_PLATFORM)
#endif	/* ESP_PLATFORM */

#if defined(PICO_BUILD)
#endif	/* PICO_BUILD */

#endif	/* ARDUINO */

void pollSerial()
{
 if (rtk.state != RCV_IDLE)
 {
  if ((millis() - rtk.t0) > 100)
  {
   rtk.state = RCV_IDLE;
   printf("receive timeout\n");
  }
 }

 if (const unsigned int t0 = millis();
     (rtk.state == RCV_IDLE) && (rtk.t0Accum != 0) && (t0 - rtk.t0Accum) > 100)
 {
  rtk.t0Accum = 0;
  rtk.rxCount = rtk.rxAccum;
  rtk.rxAccum = 0;
 }

 if (rtk.svTmr != 0)
 {
  if ((millis() - rtk.svTmr) > 500)
  {
   printf("***svTmr\n");
   rtk.svTmr = 0;

   P_SAT_DATA data = satData;
   for (int i = 0; i < satIndex; i++)
   {
    printf("%2d %3s sVid %2d elv %2d az %3d n %d ",
           i, names[data->cons], data->sVid, data->elv, data->az, data->freqs);
    auto f = data->sig;
    for (int j = 0; j < data->freqs; j++)
    {
     printf("freq %d cno %d ", f->freq, f->cno);
     f += 1;
    }
    printf("\n");
    data += 1;
   }

   int total = 0;
   int *p = rtk.svCount;
   for (int i = 0; i < 4; i++)
   {
    total +=  *p;
    printf("%2d ", *p);
    *p++ = 0;
   }
   printf("%2d\n", total);
  }
 }
}

#if defined(ARDUINO)
#define GET_COUNT
#define AVAILABLE() Serial2.available()
#define READ() Serial2.read()
#define SEND_BINARY(buf, len) sendBinary(buf, len)
#else

#if defined(ESP_PLATFORM)

#include "lwip/sockets.h"

#define GET_COUNT
#define AVAILABLE() len
#define READ() *buf++; len -= 1
#define SEND_BINARY(buf, len) send(sock, buf, len, MSG_DONTWAIT)
#endif	/* ESP_PLATFORM */

#if defined(PICO_BUILD)

#include "socket.h"

#define SOCKET_TCP_SERVER 0
#define SOCKET_TCP_CLIENT 1

#if defined(MULTI_CORE)

//#define AVAILABLE() rtk.iCount

inline int AVAILABLE()
{
 if (rtk.iCount > 0)
  rtk.availCount += 1;
 return rtk.iCount > 0;
}

inline char READ()
{
 int emp = rtk.iEmp;
 char c = rtk.iBuf[emp++];
 if (emp >= ISR_BUF_SIZE)
  emp = 0;
 rtk.iEmp = emp;
 __atomic_fetch_sub(&rtk.iCount, 1, __ATOMIC_SEQ_CST);
 rtk.readByteCount += 1;
 return c;
}

#else

#if defined(UART1_ISR)

#define AVAILABLE() rtk.iCount

inline char READ()
{
 int emp = rtk.iEmp;
 char c = rtk.iBuf[emp++];
 if (emp > ISR_BUF_SIZE)
  emp = 0;
 rtk.iEmp = emp;
 __atomic_fetch_sub(&rtk.iCount, 1, __ATOMIC_SEQ_CST);
 return c;
}

#else

#define AVAILABLE() uart_is_readable(uart1)
#define READ() uart_getc(uart1)

#endif	/* UART1_ISR */
#endif	/* MULTI_CORE */

#if defined(TCP_SERVER)
#define SEND_BINARY(buf, len) send(sock, buf, len)
#endif	/* SERVER */

#if defined(TCP_CLIENT)
#define SEND_BINARY(buf, len) send(sock, buf, len)
#endif	/* CLIENT */

#endif	/* PICO_BUILD */

#endif	/* ARDUINO */

void PROCESS_SERIAL
{
 if (rtk.state != RCV_IDLE)
 {
  if ((millis() - rtk.t0) > 100)
  {
   rtk.state = RCV_IDLE;
   printf("receive timeout\n");
  }
 }

#if defined(PICO_BUILD) || defined(MULTI_CORE)

#endif	/* PICO_BUILD */

 while (AVAILABLE() > 0)
 {
  dbg1Set();
  const unsigned char c = READ();
  switch (rtk.state)
  {
  case RCV_IDLE:
   if (c == 0xd3)
   {
    dbg0Set();
    rtk.count = 2;
    rtk.buf[0] = c;
    rtk.crc = crc24qTable[c];
    crcBuf[0] = rtk.crc;
    rtk.fil = 1;
    rtk.t0 = millis();
    rtk.state = RCV_GET_LEN;
    rtk.startTime = micros();
   }
   else if (c == '$')
   {
    rtk.t0 = millis();
    rtk.state = RCV_TEXT;
    rtk.buf[0] = c;
    rtk.fil = 1;
   }
   break;

  case RCV_GET_LEN:
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.len = (rtk.len << 8) + c;
   rtk.buf[rtk.fil] = c;
   rtk.fil += 1;
   rtk.count -= 1;
   if (rtk.count == 0)
   {
    rtk.state = RCV_GET_DATA;
    rtk.len &= 0x3ff;
    // printf("rtkLen %d\n", rtk.len);
#if defined(DBG_PRT)
    // if ((prt == 0) && (rtk.len == 19))
    if (rtk.len == 19)
    {
     prt = 1;
    }
#endif	/* DBG_PRT */
    rtk.len += 3;
   }
   break;

  case RCV_GET_DATA:
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.buf[rtk.fil] = c;
   if (rtk.fil < RTK_BUF_SIZE)	/* if room in bufffer */
   {
    rtk.fil += 1;
    rtk.len -= 1;
    if (rtk.len == 0)
    {
     const auto msgT = static_cast<uint32_t>(micros() - rtk.startTime);
     int type = (rtk.buf[3] << 4) | (rtk.buf[4] >> 4);
     rtk.rxAccum += rtk.fil;
     printf("rtkLen %4d type %4d rtkCRC %08x %5d %u\n",
	    rtk.fil, type, static_cast<unsigned int>(rtk.crc), rtk.rxAccum,
	    static_cast<unsigned int>(msgT));
     rtk.t0Accum = millis();
#if 1
     const int32_t err = SEND_BINARY(reinterpret_cast<uint8_t *>(rtk.buf), rtk.fil);
     if (err < 0)
      printf("err %ld\n", err);
#endif

#if defined(DBG_PRT)
     if (prt == 1)
     {
      printHex(reinterpret_cast<const u_int8_t *>(rtk.buf), rtk.fil);
      printHex(reinterpret_cast<const u_int8_t *>(crcBuf), rtk.fil << 2);
      prt = 0;
     }
#endif	/* DDBG_PRT */
     dbg0Clr();
     rtk.state = RCV_IDLE;
    }
   }
   else				/* buffer overflow */
   {
    rtk.fil = 0;
    rtk.state = RCV_IDLE;	/* return to idle state */
   }
   break;

  case RCV_TEXT:
   if (c == '\n')
   {
    if (rtk.buf[rtk.fil - 1] == '\r')
     rtk.fil -= 1;
    rtk.buf[rtk.fil] = 0;

    puts(rtk.buf);
    if (rtk.buf[0] == '$')
    {
     char *p2 = &rtk.buf[1];
     char chk = 0;
     char rcvChk = 0xff;
     for (int i = 0; i < (rtk.fil - 1); i++)
     {
      const char c0 = *p2++;
      if (c0 == '*')
      {
       rcvChk = getHex(&p2);
       // printf("chk %02x tmp %02x\n", chk, rcvChk);
       break;
      }
      chk ^= c0;
     }
     if (chk != rcvChk)
     {
      //printHex(reinterpret_cast<const u_int8_t *>(rtk.buf), rtk.fil);
      printf("checksum error\n");
      break;
     }
     printf("%s\n", static_cast<const char *>(rtk.buf));

     if (strncmp(rtk.buf, "$GNGGA", 6) == 0)
     {
      gpsLoc();
     }
     else if (rtk.buf[1] == 'G' && strncmp(&rtk.buf[3], "GSV", 3) == 0)
     {
      gpsSat();
     }
    }
    rtk.state = RCV_IDLE;
   }
   else
   {
    rtk.buf[rtk.fil] = c;
    if (rtk.fil < RTK_BUF_SIZE)
     rtk.fil += 1;
    else
    {
     rtk.fil = 0;
     rtk.state = RCV_IDLE;
    }
   }
   break;
  }  // end switch (state)

  dbg1Clr();
 }  // end while (AVAILABLE())

#if defined(PICO_BUILD) && defined(MULTI_CORE)
// __atomic_fetch_sub(&rtk.iCount, total, __ATOMIC_SEQ_CST);
#endif	/* PICO_BUILD */

}  // end PROCESS_SERIAL

/* $GNGGA, 091628.00, 3844.78718183,N, 07755.96337656,W, 7,28,0.5,135.9670,M,-33.6653,M,,*44 */

void gpsLoc()
{
 char *p = nextArg(rtk.buf);

 char *p0 = p;
 char *p1 = gpsInfo.timeBuf;
 *p1++ = *p0++;		/* 0 */
 *p1++ = *p0++;		/* 1 */
 *p1++ = ':';		/* 2 */
 *p1++ = *p0++;		/* 3 */
 *p1++ = *p0++;		/* 4 */
 *p1++ = ':';		/* 5 */
 *p1++ = *p0++;		/* 6 */
 *p1++ = *p0;		/* 7 */
 *p1++ = ' ';		/* 8 */
 *p1++ = ' ';		/* 9 */
 *p1 = 0;

 int gpsTime = getNum(&p, 2) * 60;
 gpsTime += getNum(&p, 2);
 gpsTime *= 60;
 gpsTime += getNum(&p, 2);
 gpsInfo.gpsTime = gpsTime;

 p = nextArg(p);
 int tmp = getNum(&p, 2);
 gpsInfo.lat = static_cast<double>(tmp) + strtod(p, &p) / 60.0;
 p = nextArg(p);
 p = nextArg(p);
 tmp = getNum(&p, 3);
 gpsInfo.lon = (static_cast<double>(tmp) + strtod(p, &p) / 60.0);
 p = nextArg(p);
 if (*p == 'W')
  gpsInfo.lon = -gpsInfo.lon;
 p = nextArg(p);
 gpsInfo.fix = static_cast<char>(getNum(&p));
 gpsInfo.sats = static_cast<char>(getNum(&p));
 printf("gpsTime %s lat %-14.10f lon %-14.10f fix %d sats %2d\n",
	gpsInfo.timeBuf, gpsInfo.lat, gpsInfo.lon, gpsInfo.fix, gpsInfo.sats);

 gpsInfo.update = true;
}

void gpsSat()
{
 if (rtk.svTmr == 0)
 {
  printf("***start svTmr\n");
  satIndex = 0;
 }
 rtk.svTmr = millis();

 char c0 = rtk.buf[2];
 int satCons = -1;
 for (int i = 0; i < sizeof(cons) - 1; i++)
 {
  if (c0 == cons[i])
  {
   satCons = i;
   break;
  }
 }

 if (satCons >= 0)
 {
  const char* txtEnd = &rtk.buf[rtk.fil];
  int freq = -1;
  for (int i = 0; i < 3; i++)
  {
   if (const char c2 = *--txtEnd; c2 == '*')
   {
    txtEnd -= 1;
    puts(txtEnd);
    freq = *txtEnd - '0';
    break;
   }
  }
  printf("constellation %d %s freq %d\n", satCons, names[satCons], freq);

  char* p = nextArg(rtk.buf); /* skip name */
  const int numMsg = getNum(&p);
  const int msgNum = getNum(&p);
  const int numSv =  getNum(&p);
  if (msgNum == 1)
   rtk.numSv = numSv;
  printf("numMsg %d msgNum %d numSv %d\n", numMsg, msgNum, numSv);
  const int n = rtk.numSv > 4 ? 4 : rtk.numSv;
  for (int i = 0; i < n; i++)
  {
   const int sVid = getNum(&p);
   const int elv = getNum(&p);
   const int az = getNum(&p);
   const int cno = getNum(&p);
   printf("%2d %2d sVid %2d elv %2d az %3d ", satIndex, rtk.numSv, sVid, elv, az);

   int j;
   for (j = 0; j <= satIndex; j++)
   {
    P_SAT_DATA rec = &satData[j];
    if (satCons == rec->cons && sVid == rec->sVid)
    {
     if (rec->freqs < MAX_SIG)
     {
      P_FREQ_INFO f = &rec->sig[rec->freqs];
      rec->freqs += 1;
      f->freq = freq;
      f->cno = cno;
      printf("n %d freq %d cno %2d\n", rec->freqs, f->freq, f->cno);
      break;
     }
    }
   }

   if (j > satIndex)
   {
    rtk.svCount[satCons] += 1;
    P_SAT_DATA rec = &satData[satIndex];
    rec->cons = satCons;
    rec->sVid = sVid;
    rec->elv = elv;
    rec->az = az;
    rec->freqs = 1;
    P_FREQ_INFO f = rec->sig;
    memset(f, 0, sizeof(rec->sig));
    f->freq = freq;
    f->cno = cno;
    if (satIndex < MAX_SAT)
     satIndex += 1;
    printf("n %d freq %d cno %2d\n", rec->freqs, f->freq, f->cno);
   }

   rtk.numSv -= 1;
  }
  printf("satCons %d count %d\n", satCons, rtk.svCount[satCons]);

  int total = 0;
  int *pC = rtk.svCount;
  for (int i = 0; i < 4; i++)
  {
   total += *pC;
   printf("%2d ", *pC);
   pC++;
  }
  printf("%2d\n", total);

 }
 printf("\n");
}

/*
GP: GPS satellites
GL: GLONASS satellites
GA: Galileo satellites
GB or BD: BeiDou satellites

Total Messages: The total number of GSV sentences in the current cycle.

Message Number: The current sentence number (1 to Total).
 
Satellites in View: The total number of satellites currently visible to the receiver.
 
Satellite Data Blocks: Up to four sets of four values each:
PRN: The satellite's PRN (Pseudo-Random Noise) number.
Elevation: The satellite's elevation in degrees (00–90).
Azimuth: The satellite's azimuth in degrees from true north (000–359).
SNR: The Signal-to-Noise Ratio in dB (00–99); this field may be empty
     if the satellite is not currently tracked. 


$GPGSV,2,1, 05, 29,05,194,31, 15,49,046,30, 18,75,200,44, 05,14,095,31, 1*6D
$GPGSV,2,2, 05, 24,52,122,46, 1*53

$GPGSV,1,1, 03, 29,05,194,33,18,75,200,42,24,52,122,39,4*55

$GPGSV,1,1,03,18,75,200,49,23,56,321,31,24,52,122,47,8*59

$GLGSV,1,1,04,71,51,018,26,86,59,197,46,72,49,281,32,73,00,000,29,1*71

$GLGSV,1,1,03,86,59,197,42,72,49,281,31,73,00,000,30,3*44

$GBGSV,2,1,06,11,14,172,38,20,42,051,32,29,30,189,49,22,24,265,32,1*7B
$GBGSV,2,2,06,19,68,321,24,35,49,262,37,1*79

$GBGSV,1,1,04,20,42,051,26,29,30,189,46,22,24,265,29,35,49,262,33,3*7F

$GBGSV,2,1,06,11,14,172,35,20,42,051,25,29,30,189,42,12,60,033,27,8*74
$GBGSV,2,2,06,22,24,265,35,35,49,262,36,8*70

$GAGSV,2,1,06,19,89,120,30,28,57,222,47,33,49,140,47,04,32,225,42,1*7D
$GAGSV,2,2,06,06,07,225,29,18,,,42,1*43

$GAGSV,2,1,06,19,89,120,29,28,57,222,46,33,49,140,43,04,32,225,43,2*72
$GAGSV,2,2,06,29,37,046,27,18,,,36,2*44

$GAGSV,1,1,04,19,89,120,29,28,57,222,44,33,49,140,42,04,32,225,43,5*77

$GAGSV,2,1,06,19,89,120,25,28,57,222,42,33,49,140,43,04,32,225,43,7*7F
$GAGSV,2,2,06,06,07,225,22,18,,,42,7*4E

GPS              GLONASS               Galileo          BeiDou (BDS)       
ID Signal        ID Signal             ID Signal        ID Signal     
0  All signals   0  All signals        0  All signals   0  All signals
1  L1 C/A        1  G1 C/A             1  E5a           1  B1I        
2  L1 P(Y)       2  G1 P               2  E5b           2  B1Q        
3  L1 M          3  G2 C/A             3  E5 (a+b)      3  B1C        
4  L2 P(Y)       4  G2 P (GLONASS-M)   4  E6-A          4  B1A        
5  L2C-M         5–F Reserved          5  E6-BC         5  B2-a       
6  L2C-L                               6  L1-A          6  B2-b       
7  L5-I                                7  L1-BC         7  B2 (a+b)   
8  L5-Q                                8–F Reserved     8  B3I         
9–F Reserved                                            9  B3Q        
                                                        A  B3A        
                                                        B  B2I        
                                                        C  B2Q        
                                                        D–F Reserved   
*/


#if defined(ARDUINO)
#define WRITE() Serial2.write(ch)
#else

#if defined(ESP_PLATFORM)
#define WRITE() uart_write_bytes(UART_NUM_1, ptr, 1)
#endif  /* ESP_PLATFORM */

#if defined(PICO_BUILD)
#define WRITE() uart_putc(uart1, ch)
#endif  /* PICO_BUILD */

#endif  /* ARDUINO */

void processRemData(void *data, size_t len)
{
 const auto *ptr = static_cast<char *>(data);
 while (len > 0)
 {
  len -= 1;
  const char ch = *ptr;
  switch (rtk.state)
  {
  case RCV_IDLE:
   if (ch == 0xd3)
   {
    rtk.count = 2;
    WRITE();
    rtk.buf[0] = ch;
    rtk.crc = crc24qTable[static_cast<int>(ch)];
    crcBuf[0] = rtk.crc;
    rtk.fil = 1;
    rtk.t0 = millis();
    rtk.state = RCV_GET_LEN;
   }
   else if (ch == '$')
   {
    rtk.t0 = millis();
    rtk.state = RCV_TEXT;
   }
   break;
    
  case RCV_GET_LEN:
   WRITE();
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ ch) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.len = (rtk.len << 8) + ch;
   rtk.buf[rtk.fil] = ch;
   rtk.fil += 1;
   rtk.count -= 1;
   if (rtk.count == 0)
   {
    rtk.state = RCV_GET_DATA;
    rtk.len &= 0x3ff;
    // printf("dLen %d\n", dLen);
#if defined(DBG_PRT)
    //if ((prt == 0) && (rtk.len == 19))
    if (rtk.len == 19)
    {
     prt = 1;
    }
#endif	/* DBG_PRT */
    rtk.len += 3;
   }
   break;

  case RCV_GET_DATA:
   WRITE();
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ ch) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.buf[rtk.fil] = ch;
   rtk.fil += 1;
   rtk.len -= 1;
   if (rtk.len == 0)
   {
    const int type = (rtk.buf[3] << 4) | (rtk.buf[4] >> 4);
    printf("len %4d type %4d CRC %08x\n", rtk.fil, type, static_cast<unsigned int>(rtk.crc));
#if defined(DBG_PRT)
    if (prt == 1)
    {
     printHex(reinterpret_cast<const uint8_t *>(rtk.buf), rtk.fil);
     printHex(reinterpret_cast<const uint8_t *>(crcBuf), rtk.fil << 2);
     prt = 0;
    }
#endif	/* DBG_PRT */
    rtk.state = RCV_IDLE;
   }
   break;

  case RCV_TEXT:
   if (ch == '\n')
   {
    rtk.state = RCV_IDLE;
   }
   else
   {

   }
   break;
  }
  ptr += 1;
 }
}

void printHex(const uint8_t *data, size_t len)
{
 int col = 0;
 for (size_t i = 0; i < len; i++)
 {
  if (col == 0)
  {
   printf("  %04X: ", static_cast<unsigned int>(i));
  }
  printf("%02X ", data[i]);
  col += 1;
  if (col == 16)
  {
   col = 0;
   printf("\n");
  }
 }
 if (col != 0)
  printf("\n");
}

/* ── CRC-24Q constants ───────────────────────────────────────────────────── */

#define CRC24Q_POLY      0x1864CFBu  /* Generator polynomial                 */
#define RTCM3_PREAMBLE   0xD3u       /* Mandatory first byte of every frame  */
#define RTCM3_HDR_LEN    3           /* Preamble + 2 length/reserved bytes   */
#define RTCM3_CRC_LEN    3           /* 24-bit CRC appended at end           */
#define RTCM3_MIN_FRAME  (RTCM3_HDR_LEN + RTCM3_CRC_LEN)

/* ── CRC-24Q lookup table (generated once on first use) ─────────────────── */

void buildCRC24qTable()
{
 for (uint32_t i = 0; i < 256; i++)
 {
  uint32_t crc = i << 16;
  for (int j = 0; j < 8; j++)
  {
   crc <<= 1;
   if (crc & 0x1000000u)
    crc ^= CRC24Q_POLY;
  }
  crc24qTable[i] = crc & 0xFFFFFFu;
 }
}

char* nextArg(char* p0)
{
 while (true)
 {
  const char c0 = *p0;
  if (c0 == 0)
   break;
  p0 += 1;
  if (c0 == ',')
  {
   break;
  }
 }
 return p0;
}

char *getNum(char *p0, int n, int *result)
{
 int val = 0;
 while (n > 0)
 {
  const char c1 = *p0++;
  val *= 10;
  val += c1 - '0';
  n -= 1;
 }
 *result = val;
 return p0;
}

int getNum(char **p0, int n)
{
 char *p1 = *p0;
 int val = 0;
 while (n > 0)
 {
  const char c1 = *p1++;
  val *= 10;
  val += c1 - '0';
  n -= 1;
 }
 *p0 = p1;
 return val;
}

int getNum(char **p0)
{
 char *p1 = *p0;
 int val = 0;
 while (true)
 {
  const char c1 = *p1++;
  if (c1 == ',' || c1 == 0)
   break;
  val *= 10;
  val += c1 - '0';
 }
 *p0 = p1;
 return val;
}

int getHex(char **p0)
{
 char *p1 = *p0;
 int val = 0;
 while (true)
 {
  char c1 = *p1++;
  if (c1 <= ' ')
   break;
  val <<= 4;
  c1 -= '0';
  if (c1 > 9)
   c1 -= 'A' - ('9' + 1);
  val += c1;
 }
 *p0 = p1;
 return val;
}
