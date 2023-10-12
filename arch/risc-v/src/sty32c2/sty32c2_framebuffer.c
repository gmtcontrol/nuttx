/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_framebuffer.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/sty32c2_video.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Window positions and sizes */

#define CONFIG_STY32C2_VID_XRES   (getreg32(STY32C2_VTG_HRES_REG))
#define CONFIG_STY32C2_VID_YRES   (getreg32(STY32C2_VTG_VRES_REG))

/* Bits per pixel */

#define STY32C2_VID_BPP           (32)

/* The width of a line in bytes */

#define STY32C2_VID_STRIDE        ((CONFIG_STY32C2_VID_XRES * STY32C2_VID_BPP + 7) / 8)

/* The area of the screen in bytes */

#define STY32C2_VID_FBLEN         (STY32C2_VID_STRIDE * CONFIG_STY32C2_VID_YRES)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Initialization */

static int  sty32c2_allocvideomemory(void);
static void sty32c2_freevideomemory(void);
static void sty32c2_hwinitialize(void);

/* Framebuffer interface methods */

static int sty32c2_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo);
static int sty32c2_getplaneinfo(struct fb_vtable_s *vtable,
                                int planeno,
                                struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware signals vertical
 * synchronisation
 */

#ifdef CONFIG_FB_SYNC
static int sty32c2_waitforvsync(struct fb_vtable_s *vtable);
#endif

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

#ifdef CONFIG_FB_HWCURSORIMAGE
#  error "Cursor image not supported by this driver"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are the addresses of allocated framebuffer memory regions */

static void *g_vid_base = 0;

static struct fb_vtable_s g_vid_vtable =
{
  .getvideoinfo = sty32c2_getvideoinfo,
  .getplaneinfo = sty32c2_getplaneinfo,
#ifdef CONFIG_FB_SYNC
  .waitforvsync = sty32c2_waitforvsync
#endif
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * sty32c2_blankscreen
 ****************************************************************************/

static inline void sty32c2_blankscreen(uint8_t *buffer, int len)
{
  memset(buffer, 0x00, len);
}

/****************************************************************************
 * sty32c2_allocvideomemory
 ****************************************************************************/

static int sty32c2_allocvideomemory(void)
{
  g_vid_base = (void *)kmm_malloc(STY32C2_VID_FBLEN);

  if (!g_vid_base)
    {
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: sty32c2_freevideomemory
 ****************************************************************************/

static void sty32c2_freevideomemory(void)
{
  if (g_vid_base)
    {
      kmm_free(g_vid_base);
      g_vid_base = NULL;
    }
}

/****************************************************************************
 * Name: sty32c2_disable_vtg
 ****************************************************************************/

static void sty32c2_disable_vtg(void)
{
  /* Disable all planes */
  
  putreg32(0, STY32C2_VTG_ENAB_REG);
}

/****************************************************************************
 * Name: sty32c2_enable_vtg
 ****************************************************************************/

static void sty32c2_enable_vtg(void)
{
  /* Enable all planes */
  
  putreg32(1, STY32C2_VTG_ENAB_REG);
}

/****************************************************************************
 * Name: sty32c2_disable_dma
 ****************************************************************************/

static void sty32c2_disable_dma(void)
{
  /* Disable DMA transfer */
  
    putreg32(0, STY32C2_FBDMA_ENAB_REG);
}

/****************************************************************************
 * Name: sty32c2_enable_dma
 ****************************************************************************/

static void sty32c2_enable_dma(void)
{
  /* Enable DMA transfer */
  
  putreg32(1, STY32C2_FBDMA_ENAB_REG);
}

/****************************************************************************
 * Name: sty32c2_putpixel
 ****************************************************************************/

static void sty32c2_putpixel(uint16_t x, uint16_t y, uint32_t col)
{
  putreg32(col, ((uint32_t)g_vid_base) + (y * STY32C2_VID_STRIDE) + (x * STY32C2_VID_BPP));
}

/****************************************************************************
 * Name: sty32c2_hwinitialize
 ****************************************************************************/

static void sty32c2_hwinitialize(void)
{
  /* Disable all planes */

  sty32c2_disable_vtg();

  /* Set up video */

  vinfo("Initialize video controller\n");

  putreg32((uint32_t)g_vid_base, STY32C2_FBDMA_BASE_REG);
  putreg32(STY32C2_VID_FBLEN,    STY32C2_FBDMA_LENG_REG);
  putreg32(1,                    STY32C2_FBDMA_LOOP_REG);
  putreg32(0,                    STY32C2_FBDMA_OFFS_REG);

  /* Clear the screen */

  sty32c2_blankscreen((uint8_t *)g_vid_base, STY32C2_VID_FBLEN);

  /* Enable the frame buffer DMA */

  sty32c2_enable_dma();

  /* Enable all planes */

  sty32c2_enable_vtg();
}

/****************************************************************************
 * Name: sty32c2_getvideoinfo
 ****************************************************************************/

static int sty32c2_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo)
{
  if (!vtable || !vinfo)
    {
      return -EINVAL;
    }

  vinfo->fmt     = FB_FMT_RGB32;
  vinfo->xres    = CONFIG_STY32C2_VID_XRES;
  vinfo->yres    = CONFIG_STY32C2_VID_YRES;
  vinfo->nplanes = 1;
#ifdef CONFIG_FB_OVERLAY
  vinfo->noverlays = 0;
#endif

  return OK;
}

/****************************************************************************
 * Name: sty32c2_getplaneinfo
 ****************************************************************************/

static int sty32c2_getplaneinfo(struct fb_vtable_s *vtable,
                                int planeno,
                                struct fb_planeinfo_s *pinfo)
{
  if (!vtable || !pinfo)
    {
      return -EINVAL;
    }

  pinfo->fbmem   = g_vid_base;
  pinfo->fblen   = STY32C2_VID_FBLEN;
  pinfo->stride  = STY32C2_VID_STRIDE;
  pinfo->bpp     = STY32C2_VID_BPP;
  pinfo->display = 0;

  return OK;
}

/****************************************************************************
 * Name: sty32c2_waitforvsync
 ****************************************************************************/

#ifdef CONFIG_FB_SYNC
static int sty32c2_waitforvsync(struct fb_vtable_s *vtable)
{
  if (!vtable)
    {
      return -EINVAL;
    }

  /* Wait until register reload hase been done */

  while ((getreg32(STY32C2_FBDMA_DONE_REG) & 1) == 0) {

  }

  return OK;
}
#endif /* CONFIG_FB_SYNC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  int ret;

  vinfo("Allocating framebuffers\n");
  ret = sty32c2_allocvideomemory();
  if (ret != 0)
    {
      verr("ERROR: Failed to allocate video buffers\n");
      return ret;
    }

  /* Initialize the hardware */

  vinfo("Initializing hardware\n");
  sty32c2_hwinitialize();

  return OK;
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  /* Disable the hardware */

  sty32c2_disable_vtg();
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.
 *   Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  vinfo("vplane: %d\n", vplane);
  if (vplane == 0)
    {
      return &g_vid_vtable;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: up_fbteardown
 ****************************************************************************/

void fb_teardown(void)
{
  /* Disable the hardware */

  sty32c2_disable_vtg();

  /* Free the video buffers */

  sty32c2_freevideomemory();
}
