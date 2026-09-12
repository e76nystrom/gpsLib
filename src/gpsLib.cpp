#include "cfg.h"

#if defined(ARDUINO)

#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "dbgPin.h"

#endif	/* ARDUINO */

#if defined(ESP_PLATFORM)

#define dbg0Set()
#define dbg0Clr()
#define dbg1Set()
#define dbg1Clr()

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

#define dbg0Set()
#define dbg0Clr()
#define dbg1Set()
#define dbg1Clr()

uint32_t millis()
{
 return static_cast<uint32_t>(timer_time_us_64(timer_hw) / 1000ULL);
}

inline uint32_t micros()
{
 return static_cast<uint32_t>(timer_time_us_64(timer_hw));
}

#endif	/* PICO_BUILD */

#include "gpsLib.h"

#if defined(ARDUINO)
void dbgInit()
{
 pinMode(DBG0_PIN, OUTPUT);
 pinMode(DBG1_PIN, OUTPUT);
}
#endif

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

 unsigned int t0 = millis();
 if ((rtk.state == RCV_IDLE) && (rtk.t0Accum != 0) && (t0 - rtk.t0Accum) > 100)
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
#define AVAILABLE() Serial2.available()
#define READ() Serial2.read()
#define SEND_BINARY(buf, len) sendBinary(buf, len)
#else
#define AVAILABLE() len
#define READ() *buf++; len -= 1
#define SEND_BINARY(buf, len) send(sock, buf, len, MSG_DONTWAIT)
#endif

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

 while (AVAILABLE() > 0)
 {
  dbg1Set();
  unsigned char c = READ();
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
#if defined(RTK_SEND)
    sendBinary(reinterpret_cast<const uint8_t *>(rtk.buf), (ssize_t) rtk.fil);
#endif	/* RTK_SEND */

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
      printHex(reinterpret_cast<const u_int8_t *>(rtk.buf), rtk.fil);
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
    rtk.fil += 1;
   }
   break;
  }
  dbg1Clr();
 }
}

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
   const char c2 = *--txtEnd;
   if (c2 == '*')
   {
    txtEnd -= 1;
    puts(txtEnd);
    freq = *txtEnd - '0';
    break;
   }
  }
  printf("constellation %d %s freq %d\n", satCons, names[satCons], freq);

  char* p = nextArg(rtk.buf); /* skip name */
  int numMsg = getNum(&p);
  int msgNum = getNum(&p);
  int numSv =  getNum(&p);
  if (msgNum == 1)
   rtk.numSv = numSv;
  printf("numMsg %d msgNum %d numSv %d\n", numMsg, msgNum, numSv);
  int n = rtk.numSv > 4 ? 4 : rtk.numSv;
  for (int i = 0; i < n; i++)
  {
   int sVid = getNum(&p);
   int elv = getNum(&p);
   int az = getNum(&p);
   int cno = getNum(&p);
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

#if defined(ARDUINO)
#define WRITE() Serial2.write(ch)
#endif  /* ARDUINO */
#if defined(ESP_PLATFORM)
#define WRITE() uart_write_bytes(UART_NUM_1, ptr, 1)
#endif  /* ESP_PLATFORM */
#if defined(PICO_BUILD)
#define WRITE() uart_putc(uart1, ch)
#endif  /* PICO_BUILD */

//#if defined(RTK_RECV)

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

//#endif	/* RTK_RECV */

// void printHex(const uint8_t *data, size_t len)
// {
//  int col = 0;
//  for (size_t i = 0; i < len; i++)
//  {
//   if (col == 0)
//   {
//    printf("  %04X: ", static_cast<unsigned int>(i));
//   }
//   printf("%02X ", data[i]);
//   col += 1;
//   if (col == 16)
//   {
//    col = 0;
//    printf("\n");
//   }
//  }
//  if (col != 0)
//   printf("\n");
// }

// /* ── CRC-24Q constants ───────────────────────────────────────────────────── */
//
// #define CRC24Q_POLY      0x1864CFBu  /* Generator polynomial                 */
// #define RTCM3_PREAMBLE   0xD3u       /* Mandatory first byte of every frame  */
// #define RTCM3_HDR_LEN    3           /* Preamble + 2 length/reserved bytes   */
// #define RTCM3_CRC_LEN    3           /* 24-bit CRC appended at end           */
// #define RTCM3_MIN_FRAME  (RTCM3_HDR_LEN + RTCM3_CRC_LEN)
//
// /* ── CRC-24Q lookup table (generated once on first use) ─────────────────── */
//
// void buildCRC24qTable()
// {
//  for (uint32_t i = 0; i < 256; i++)
//  {
//   uint32_t crc = i << 16;
//   for (int j = 0; j < 8; j++)
//   {
//    crc <<= 1;
//    if (crc & 0x1000000u)
//     crc ^= CRC24Q_POLY;
//   }
//   crc24qTable[i] = crc & 0xFFFFFFu;
//  }
// }

// char* nextArg(char* p0)
// {
//  while (true)
//  {
//   const char c0 = *p0;
//   if (c0 == 0)
//    break;
//   p0 += 1;
//   if (c0 == ',')
//   {
//    break;
//   }
//  }
//  return p0;
// }
//
// char *getNum(char *p0, int n, int *result)
// {
//  int val = 0;
//  while (n > 0)
//  {
//   const char c1 = *p0++;
//   val *= 10;
//   val += c1 - '0';
//   n -= 1;
//  }
//  *result = val;
//  return p0;
// }
//
// int getNum(char **p0, int n)
// {
//  char *p1 = *p0;
//  int val = 0;
//  while (n > 0)
//  {
//   const char c1 = *p1++;
//   val *= 10;
//   val += c1 - '0';
//   n -= 1;
//  }
//  *p0 = p1;
//  return val;
// }
//
// int getNum(char **p0)
// {
//  char *p1 = *p0;
//  int val = 0;
//  while (true)
//  {
//   const char c1 = *p1++;
//   if (c1 == ',' || c1 == 0)
//    break;
//   val *= 10;
//   val += c1 - '0';
//  }
//  *p0 = p1;
//  return val;
// }
//
// int getHex(char **p0)
// {
//  char *p1 = *p0;
//  int val = 0;
//  while (true)
//  {
//   char c1 = *p1++;
//   if (c1 <= ' ')
//    break;
//   val <<= 4;
//   c1 -= '0';
//   if (c1 > 9)
//    c1 -= 'A' - ('9' + 1);
//   val += c1;
//  }
//  *p0 = p1;
//  return val;
// }
