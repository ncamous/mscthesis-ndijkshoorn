#include <utils/ardrone_gen_ids.h>
#include <utils/ardrone_crc_32.h>
#include <string.h>
#include <stdio.h>

#include <stdlib.h>
#include <time.h>

#ifdef _MSC_VER


inline int c99_vsnprintf(char* str, size_t size, const char* format, va_list ap)
{
    int count = -1;

    if (size != 0)
        count = _vsnprintf_s(str, size, _TRUNCATE, format, ap);
    if (count == -1)
        count = _vscprintf(format, ap);

    return count;
}

inline int c99_snprintf(char* str, size_t size, const char* format, ...)
{
    int count;
    va_list ap;

    va_start(ap, format);
    count = c99_vsnprintf(str, size, format, ap);
    va_end(ap);

    return count;
}

#define snprintf c99_snprintf

#endif // _MSC_VER

void
ardrone_gen_appid (const char *appName, const char *sdkVersion, char appId [9], char *appDesc, int descLen)
{
#define _BUFFER_SIZE 512
  char appNamePlusSdk [_BUFFER_SIZE] = {0};
  snprintf (appNamePlusSdk, _BUFFER_SIZE, "%s:%s", appName, sdkVersion);
#undef _BUFFER_SIZE
  { // block for definition
  uint32_t binaryId = ardrone_crc_32 ((uint8_t *)appNamePlusSdk, strlen (appNamePlusSdk));
  snprintf (appId, 9, "%08x", binaryId);
  appId [8] = '\0';
  strncpy (appDesc, appName, descLen);
  }
}

void
ardrone_gen_usrid (const char *usrName, char usrId [9], char *usrDesc, int descLen)
{
  uint32_t binaryId = ardrone_crc_32 ((uint8_t *)usrName, strlen (usrName));
  snprintf (usrId, 9, "%08x", binaryId);
  usrId [8] = '\0';
  strncpy (usrDesc, usrName, descLen);
}

void
ardrone_gen_sessionid (char sessId [9], char *sessDesc, int descLen)
{
  static int runOnce = 1;
  if (1 == runOnce)
    {
      srand (time (NULL));
      runOnce = 0;
    }
  { // block for definitoin
  uint32_t binaryId = (uint32_t)rand ();
  binaryId = (0 != binaryId) ? binaryId : 1u;
  snprintf (sessId, 9, "%08x", binaryId);
  sessId [8] = '\0';
  snprintf (sessDesc, descLen, "Session %s", sessId);
  }
}
