#include <VLIB/video_mem32.h>
#include <VLIB/Platform/video_utils.h>

#include <VP_Os/vp_os_types.h>

#ifndef bswap
// from http://www.klocwork.com/products/documentation/current/Checkers:PORTING.BSWAP.MACRO
#if BIG_ENDIAN
#define SWAPBYTES(x) ( ((x) & 0xff) << 24 ) | ( (((x) >> 8) & 0xff) << 16 ) | ( (((x) >> 16) & 0xff) << 8 ) | ( (((x) >> 24) & 0xff) )
#else
#define SWAPBYTES(x) ( x )
#endif
#define bswap(x) (SWAPBYTES(x))
#endif


// Default implementation

C_RESULT video_zeromem32( uint32_t* dst, uint32_t length )
{
  while( length )
  {
    *dst = 0;

    dst ++;
    length--;
  }

  return C_OK;
}

C_RESULT video_copy32(uint32_t* dst, uint32_t* src, uint32_t nb)
{
  uint32_t i;

  for( i = 0; i < nb; i++ )
  {
    dst[i] = src[i];
  }

  return C_OK;
}

C_RESULT video_copy32_swap(uint32_t* dst, uint32_t* src, uint32_t nb)
{
  uint32_t i;

  for( i = 0; i < nb; i++ )
  {
    dst[i] = bswap( src[i] );
  }

  return C_OK;
}
