#include <VP_Os/vp_os_malloc.h>
#include <ardrone_tool/ardrone_version.h>
#include <utils/ardrone_ftp.h>
#include <config.h>

#include <stdio.h>

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

int
compareVersions (ardrone_version_t *v1, ardrone_version_t *v2)
{
  int retVal =  0;
  if (NULL == v1 || NULL == v2)
    {
      return 0;
    }
  retVal = v1->majorVersion - v2->majorVersion;
  if (0 == retVal)
    {
      retVal = v1->minorVersion - v2->minorVersion;
      if (0 == retVal)
        {
          retVal = v1->revision - v2->revision;
        }
    }
  return retVal;
}

int
getDroneVersion (const char *tempPath, const char *droneIp, ardrone_version_t *version)
{
	_ftp_status status;
	_ftp_t *ftp = NULL;
	size_t lNameSize;
	char *localName;

  if (NULL == tempPath || NULL == droneIp || NULL == version)
    {
      return -1;
    }
  
  ftp = ftpConnect (droneIp, FTP_PORT, "anonymous", "", &status);
  if (FTP_FAILED (status) || NULL == ftp)
    {
      ftpClose (&ftp);
      return -2;
    }

  lNameSize = strlen (tempPath) + strlen ("/__version.txt") + 1;
  localName = (char *)vp_os_calloc (lNameSize, 1);
  if (NULL == localName)
    {
      ftpClose (&ftp);
      return -3;
    }

  snprintf (localName, lNameSize, "%s/__version.txt", tempPath);
  status = ftpGet (ftp, "version.txt", localName, 0, NULL);
  if (FTP_FAILED (status))
    {
      vp_os_free (localName);
      ftpClose (&ftp);
      return -4;
    }
  
  ftpClose (&ftp);
  
  { // block for definition
  FILE *versionFile = fopen (localName, "r");
  uint32_t maj, min, rev;

  if (NULL == versionFile)
    {
      remove (localName);
      vp_os_free (localName);
      return -5;
    }

  
  if (3 != fscanf (versionFile, "%u.%u.%u", &maj, &min, &rev))
    {
      fclose (versionFile);
      remove (localName);
      vp_os_free (localName);
      return -6;
    }
  
  fclose (versionFile);
  remove (localName);
  vp_os_free (localName);
  
  version->majorVersion = maj;
  version->minorVersion = min;
  version->revision = rev;
  } // end of versionFile block
  return 0;
}
