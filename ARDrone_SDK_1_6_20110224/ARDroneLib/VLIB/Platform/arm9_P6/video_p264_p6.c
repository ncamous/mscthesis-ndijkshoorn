#include <stdio.h>
#include "libuiomap.h"
#include "dma_malloc.h"
#include "video_config.h"
#include "video_p264_p6.h"
#include "video_utils_p6.h"
#include <VP_Os/vp_os_malloc.h>
#include "P6_h264_reg.h"
#include <VLIB/P264/p264_common.h>
#include <VLIB/P264/p264_zigzag.h>
#include <stdio.h>
#include <VP_Os/vp_os_print.h>

//#define H264_P6_DEBUG
//#define H264_P6_PFRAME_DEBUG


// h264 registers
static struct uiomap ui_h264_reg;
static struct uiomap ui_sysc_reg;

// h264 picture parameters
static uint8_t* current_Y;
static uint8_t* current_Cb;
static uint8_t* current_Cr;
static uint32_t current_linesize;
static uint32_t current_width;
static uint32_t current_height;
static uint32_t current_i_MB;
static uint32_t current_j_MB;
static bool_t I_encoding;

static uint8_t* h264_ref_frame;  // frame decoded by encoder as a reference
static uint8_t* h264_deb_frame;  // frame decoded by encoder as a reference

C_RESULT video_p264_p6_init(void)
{
  int32_t err;
  err= uiomap_ioremap(&ui_h264_reg, 0xd0060000, 0x10000);
  if (err)
  {
    PRINT("error remapping h264 regs: %s\n", strerror(-err));
    return C_FAIL;
  }
  err= uiomap_ioremap(&ui_sysc_reg, 0xd0000000, 0x10);
  if (err)
  {
    PRINT("error activating h264 clock : %s\n", strerror(-err));
    return C_FAIL;
  }

 // active h264 clock
  uint32_t value;
  value = uiomap_readl(&ui_sysc_reg, 0x0C);
  uiomap_writel(&ui_sysc_reg,value | (1<<4) ,0x0C);

  // reset IP
  uiomap_writel(&ui_h264_reg, 0x1, H264_RESET);

  // config h264
  uiomap_writel (&ui_h264_reg, 0, H264_ITEN ); // no interrupt
  uiomap_writel(&ui_h264_reg, 0x33, H264_DMA);
  uiomap_writel(&ui_h264_reg, 0x33, H264_DMAINT);

  uiomap_writel(&ui_h264_reg, (1<<18)|(1<<13)|(1<<11)|(1<<9)|(1<<7), H264_CONFIG); // ME_noDEB | MEtoDEB_EN | MEtoDCT_EN | IS_H264 | MEuseMCforCC

  // configure MB split for P frame
  uiomap_writel(&ui_h264_reg,ME_ANAL_16x16,ME_ANALYSIS); // Authorize only 16x16 block motion


  // configure search patterns for P frames
  //uiomap_writel(&ui_h264_reg,(ME_PAT_SMALL_DIAMOND<<28)|(ME_PAT_SMALL_DIAMOND<<24)|(ME_PAT_SMALL_DIAMOND<<20)|(ME_PAT_SMALL_DIAMOND<<16)|(ME_PAT_SMALL_DIAMOND<<12)|(ME_PAT_SMALL_DIAMOND<<8)|(ME_PAT_SMALL_DIAMOND<<4)|(ME_PAT_SMALL_DIAMOND<<0),ME_PAT_LIST); // set small diamond
  uiomap_writel(&ui_h264_reg,(ME_PAT_BIG_SQUARE<<28)|(ME_PAT_BIG_SQUARE<<24)|(ME_PAT_BIG_SQUARE<<20)|(ME_PAT_BIG_SQUARE<<16)|(ME_PAT_BIG_SQUARE<<12)|(ME_PAT_BIG_SQUARE<<8)|(ME_PAT_BIG_SQUARE<<4)|(ME_PAT_BIG_SQUARE<<0),ME_PAT_LIST); // set big square

  uiomap_writel(&ui_h264_reg,0x00000022,ME_PAT_RESIZE); // no subpixelix mv on luma

  //test
   uiomap_writel(&ui_h264_reg,0x80007000,DEB_ME_TMP_ADDR ); // MUST BE INTRAM


  // init local variable
  // h264 picture parameters
  current_Y  = NULL;
  current_Cb = NULL;
  current_Cr = NULL;
  current_linesize = 0;
  current_width = 0;
  current_height = 0;
  h264_deb_frame = NULL;
  h264_ref_frame = NULL;

  PRINT("H264 IP : initialized\n");
  return C_OK;
}

#ifdef HAS_P264_FTRANSFORM
C_RESULT video_p264_prepare_slice ( video_controller_t* controller, const vp_api_picture_t* blockline)
{
  // picture dimensions change ?
  if (controller->width != current_width || controller->height != current_height)
  {
    // realloc a YUV 4:2:0 reference frame
    h264_ref_frame = (uint8_t*)dma_realloc(h264_ref_frame,controller->width*controller->height*3/2);
    h264_deb_frame = (uint8_t*)dma_realloc(h264_deb_frame,controller->width*controller->height*3/2);
    current_width = controller->width;
    current_height = controller->height;

    if (current_width > CIF_WIDTH || current_height > CIF_HEIGHT)
      PRINT("resolutions above CIF are not supported by P6 h264 ip\n");
    // set search window size
    // ... use default value

    // set new dimensions
    uiomap_writel(&ui_h264_reg,(current_width<<16)|current_width,H264_LINESIZE); // set ME&MC frame line size
    uiomap_writel(&ui_h264_reg,((current_height>>4)<<24)|((current_width>>4)<<16)|((current_height>>4)<<8)|((current_width>>4)<<0),H264_FRAMESIZE); // set ME&MC frame dimension
#ifdef H264_P6_DEBUG
    PRINT ("H264 IP : new frame dimensions %dx%d\n",current_width,current_height);
#endif
  }

  // swap DEB and ref frames
  uint8_t* p_tmp;
  p_tmp = h264_deb_frame;
  h264_deb_frame = h264_ref_frame;
  h264_ref_frame = p_tmp;

  // set DEB new frame
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(h264_deb_frame),DEB_ME_FRAME_ADDRY);
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(h264_deb_frame+current_width*current_height),DEB_ME_FRAME_ADDRCC);
  dma_flush_inv( (uint32_t)h264_deb_frame, current_width*current_height*3/2);

  // set reference frame (same as DEB)
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(h264_ref_frame),ME_SW_FRAME_ADDRY);
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(h264_ref_frame+current_width*current_height),ME_SW_FRAME_ADDRCC);
  dma_flush_inv( (uint32_t)h264_ref_frame, current_width*current_height*3/2);

  // configure P/I frame type
  if (controller->picture_type == VIDEO_PICTURE_INTRA)
  {
#if defined(H264_P6_DEBUG) || defined(H264_P6_PFRAME_DEBUG)
    PRINT ("H264 IP : NEW I FRAME\n");
#endif
    I_encoding = TRUE;
    //uiomap_writel(&ui_h264_reg,ME_ALGO_I_FRAME|ME_ALGO_DIS_MVP_INTRA_PRED,ME_ALGORITHM); // configure ME to I-frame, don't use pred intra prediction
    uiomap_writel(&ui_h264_reg,ME_ALGO_I_FRAME,ME_ALGORITHM); // configure ME to I-frame, use pred intra prediction
  }
  else
  {
#if defined(H264_P6_DEBUG) || defined(H264_P6_PFRAME_DEBUG)
    PRINT ("H264 IP : NEW P FRAME\n");
#endif
    I_encoding = FALSE;
    //uiomap_writel(&ui_h264_reg,0x70000|ME_ALGO_DIS_MVP_INTRA_PRED|ME_ALGO_P_FRAME|0x07,ME_ALGORITHM); // configure ME to P-frame, don't use MV prediction
    uiomap_writel(&ui_h264_reg,0x70000|ME_ALGO_P_FRAME|0x07,ME_ALGORITHM); // configure ME to P-frame, use MV prediction
  }

  // retrieve picture pointers
  current_Y = blockline->y_buf;
  current_Cb = blockline->cb_buf;
  current_Cr = blockline->cr_buf;

  // set input frame addr
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(current_Y),ME_CMB_FRAME_ADDRY);
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(current_Cb),ME_CMB_FRAME_ADDRCU);
  uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(current_Cr),ME_CMB_FRAME_ADDRCV);

  // flush input frame
  dma_flush_inv( (uint32_t)current_Y, current_width*current_height);
  dma_flush_inv( (uint32_t)current_Cb, (current_width*current_height>>2));
  dma_flush_inv( (uint32_t)current_Cr, (current_width*current_height>>2));

  // flush output data
  dma_flush_inv( (uint32_t)controller->cache, (controller->width>>4) * (controller->height>>4) * sizeof(MB_p264_t));

  // reset current macroblock index
  uiomap_writel(&ui_h264_reg,0,H264_MB_ADDR);
  current_i_MB = 0;
  current_j_MB = 0;

  return C_OK;
}

static void intra_pred_4x4_p6_to_list (uint32_t intra_pred_4x4_0, uint32_t intra_pred_4x4_1, intra_4x4_mode_t* out)
{
  uint32_t i;
  for (i=0;i<8;i++)
  {
    *out++ = (intra_4x4_mode_t) (intra_pred_4x4_0 & 0x0F);
    intra_pred_4x4_0 = intra_pred_4x4_0>>4;
  }
  for (i=0;i<8;i++)
  {
    *out++ = (intra_4x4_mode_t) (intra_pred_4x4_1 & 0x0F);
    intra_pred_4x4_1 = intra_pred_4x4_1>>4;
  }
}

void P6_get_MV (MV_XY_t *mv,uint32_t num_mv)
{
  uint32_t mv_2k_result;
  mv_2k_result = uiomap_readl(&ui_h264_reg,ME_PRED_BASE_REG+((num_mv&0x0E)<<1));
  if ((num_mv&0x01) == 0)
  {
    mv->x = (mv_2k_result&0x000000FF)>>0;
    mv->x >>= 2;
    mv->y = (mv_2k_result&0x0000FF00)>>8;
    mv->y >>= 2;
  }
  else
  {
    mv->x = (mv_2k_result&0x00FF0000)>>16;
    mv->x >>= 2;
    mv->y = (mv_2k_result&0xFF000000)>>24;
    mv->y >>= 2;
  }
}

C_RESULT video_p264_encode_MB(video_macroblock_t* prev_macroblock,video_macroblock_t* next_macroblock ,int32_t qp)
{
  if (prev_macroblock != NULL)
  {
    // there is already a macroblock processing, wait for its completion and finalize it
    MB_p264_t* MB_P6 = (MB_p264_t*)prev_macroblock->data;

    // wait hardware completion
    uint32_t nb_cycle=0;
    uint32_t completion_flag,completion_mask;
    if (I_encoding == TRUE)
    {
      completion_flag = I_MB_COMPLETION_FLAG;
      completion_mask = I_MB_COMPLETION_MASK;
    }
    else
    {
      completion_flag = P_MB_COMPLETION_FLAG;
      completion_mask = P_MB_COMPLETION_MASK;
    }

    while ((uiomap_readl(&ui_h264_reg,H264_STATUS)&completion_mask) != completion_flag)
        nb_cycle++;
    //PRINT("nb cycle %d\n",nb_cycle);

    // read me results
    uint32_t me_result;
    me_result =  uiomap_readl(&ui_h264_reg,ME_RESULT);
    //PRINT("me result 0x%x\n",me_result);

    if (I_encoding == FALSE)
    {

      // save MB partition and MV
      if (INTER_16_8_PARTITION(me_result) == 0)
      {
        // 16x16 partition
        // save partition mode
        prev_macroblock->inter_partition_mode[0] = INTER_PART_16x16;
        // save corresponding MV
        P6_get_MV(&prev_macroblock->inter_MV[0],0);
        prev_macroblock->nb_partition = 1;
      }
      else if (INTER_16_8_PARTITION(me_result) == 1)
      {
        // 16x8 partition
        // save partition mode
        prev_macroblock->inter_partition_mode[0] = INTER_PART_16x8;
        prev_macroblock->inter_partition_mode[1] = INTER_PART_16x8;
        // save corresponding MV
        P6_get_MV(&prev_macroblock->inter_MV[0],0);
        P6_get_MV(&prev_macroblock->inter_MV[1],8);
        prev_macroblock->nb_partition = 2;
      }
      else if (INTER_16_8_PARTITION(me_result) == 2)
      {
        // 16x8 partition
        // save partition mode
        prev_macroblock->inter_partition_mode[0] = INTER_PART_8x16;
        prev_macroblock->inter_partition_mode[1] = INTER_PART_8x16;
        // save corresponding MV
        P6_get_MV(&prev_macroblock->inter_MV[0],0);
        P6_get_MV(&prev_macroblock->inter_MV[1],4);
        prev_macroblock->nb_partition = 2;
      }
      // 8x8 8x4 4x8 4x4 cases missing
      else
        PRINT ("wrong partition (or not supported) result %d\n",INTER_16_8_PARTITION(me_result));

      // patch DC chroma coeff
      // P6 h264 IP performs a zigzag on DC coeff. It should not.
      MB_P6->inter.DC_U[3] = MB_P6->inter.dummy_DC_U[0];
      MB_P6->inter.DC_V[3] = MB_P6->inter.dummy_DC_V[0];

#ifdef H264_P6_PFRAME_DEBUG
      uint32_t i;
      for (i=0;i<prev_macroblock->nb_partition;i++)
      {
        PRINT ("partition[%d]=%d - MV (%d,%d)\n",i,prev_macroblock->inter_partition_mode[i],prev_macroblock->inter_MV[i].x,prev_macroblock->inter_MV[i].y);
      }
#endif
    }
    else
    {
      #ifdef H264_P6_DEBUG
      PRINT ("me result 0x%x\n",me_result);
      PRINT ("--> Y intra mode %d\n",(me_result&0x1F));
      PRINT ("--> Chroma intra mode %d\n",CHROMA_MODE(me_result));
      #endif

      if (IS_INTRA_4x4(me_result))
      {
        // patch DC chroma coeff
        // P6 h264 IP performs a zigzag on DC coeff. It should not.
        MB_P6->intra_4x4.DC_U[3] = MB_P6->intra_4x4.dummy_DC_U[0];
        MB_P6->intra_4x4.DC_V[3] = MB_P6->intra_4x4.dummy_DC_V[0];
        // set macroblock type
        prev_macroblock->intra_type = INTRA_4x4;
        // save 4x4 intra luma result
        uint32_t intra_pred_4x4_0,intra_pred_4x4_1;
        intra_pred_4x4_0 =  uiomap_readl(&ui_h264_reg,ME_IPRED0);
        intra_pred_4x4_1 =  uiomap_readl(&ui_h264_reg,ME_IPRED1);
        intra_pred_4x4_p6_to_list (intra_pred_4x4_0,intra_pred_4x4_1,prev_macroblock->intra_4x4_mode);
    #ifdef H264_P6_DEBUG
        PRINT ("intra 4x4 pred result 0x%x\n",intra_pred_4x4_0);
        PRINT ("intra 4x4 pred result 0x%x\n",intra_pred_4x4_1);
    #endif
      }
      else
      {
        // patch DC chroma coeff
        // P6 h264 IP performs a zigzag on DC coeff. It should not.
        MB_P6->intra_16x16.DC_U[3] = MB_P6->intra_16x16.dummy_DC_U[0];
        MB_P6->intra_16x16.DC_V[3] = MB_P6->intra_16x16.dummy_DC_V[0];
        // set macroblock type
        prev_macroblock->intra_type = INTRA_16x16;
        // save 16x16 luma result
        prev_macroblock->intra_luma_16x16_mode = INTRA_16x16_MODE(me_result);
    #ifdef H264_P6_DEBUG
        printf("luma 16x16 pred %d\n",INTRA_16x16_MODE(me_result));
    #endif

        // zagzig DC luma (P6 bug fix)
        int16_t tmp_zagzig[16];
        zagzig_4x4 (MB_P6->intra_16x16.DC_Y, tmp_zagzig);
        vp_os_memcpy(MB_P6->intra_16x16.DC_Y,tmp_zagzig,16*sizeof(int16_t));
      }

      // save chroma mode
      prev_macroblock->intra_chroma_8x8_mode = CHROMA_MODE(me_result);
    }

    // configure IP for next MB
#ifdef H264_P6_DEBUG
    PRINT("encoding mb (%d,%d)\n",current_i_MB,current_j_MB);
#endif
    current_i_MB++;
    if (current_i_MB >= (current_width>>4))
    {
      current_i_MB = 0;
      current_j_MB++;
    }
    uiomap_writel(&ui_h264_reg,(current_j_MB<<24)|(current_i_MB<<16)|(current_j_MB<<8)|current_i_MB,H264_MB_ADDR);
  }

  if (next_macroblock != NULL)
  {
    MB_p264_t* MB_P6 = (MB_p264_t*)next_macroblock->data;

    if (current_i_MB == 0)
    {
      // dirty bug fix : flush next output h264 IP buffer
      dma_flush_inv( MB_P6, (current_width>>4) * sizeof(MB_p264_t));
    }

    // set quantization parameter, the same QP is used for chroma and luma block
    uiomap_writel(&ui_h264_reg,(qp<<24)|(qp<<16)|(qp<<8)|qp,H264_QP);

    // set output data addr
    //PRINT ("dma_virt2phy on addr %d - dma addr %d\n",MB_P6,dma_virt2phy(MB_P6));
    uiomap_writel(&ui_h264_reg,(uint32_t)dma_virt2phy(MB_P6),DCT_DEST_Y_ADDR);

    // launch
    uiomap_writel(&ui_h264_reg, 0, H264_START);
  }

  return C_OK;
}
#endif


C_RESULT video_p264_p6_close(void)
{
  uiomap_iounmap(&ui_h264_reg);
  uiomap_iounmap(&ui_sysc_reg);
  if (h264_ref_frame != NULL)
    dma_free (h264_ref_frame);
  h264_ref_frame = NULL;
  if (h264_deb_frame != NULL)
    dma_free (h264_deb_frame);
  h264_deb_frame = NULL;
  current_Y  = NULL;
  current_Cb = NULL;
  current_Cr = NULL;
  current_linesize = 0;
  current_width = 0;
  current_height = 0;
  current_i_MB = 0;
  current_j_MB = 0;
  return C_OK;
}
