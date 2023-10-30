/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_rtc.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_RTC_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_RTC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_RTC_R64CNT_OFFSET     0x0000      /* 64-Hz Counter */
#define RA6M5_RTC_RSECCNT_OFFSET    0x0002      /* Binary Counter 0 (Second Counter) */
#define RA6M5_RTC_RMINCNT_OFFSET    0x0004      /* Binary Counter 1 (Minute Counter) */
#define RA6M5_RTC_RHRCNT_OFFSET     0x0006      /* Binary Counter 2 (Hour Counter) */
#define RA6M5_RTC_RWKCNT_OFFSET     0x0008      /* Binary Counter 3 (Day-of-Week Counter) */
#define RA6M5_RTC_RDAYCNT_OFFSET    0x000A      /* Day Counter */
#define RA6M5_RTC_RMONCNT_OFFSET    0x000C      /* Month Counter */
#define RA6M5_RTC_RYRCNT_OFFSET     0x000E      /* Year Counter */
#define RA6M5_RTC_RSECAR_OFFSET     0x0010      /* Second Alarm Register */
#define RA6M5_RTC_RMINAR_OFFSET     0x0012      /* Minute Alarm Register */
#define RA6M5_RTC_RHRAR_OFFSET      0x0014      /* Hour Alarm Register */
#define RA6M5_RTC_RWKAR_OFFSET      0x0016      /* Day-of-Week Alarm Register */
#define RA6M5_RTC_RDAYAR_OFFSET     0x0018      /* Date Alarm Register */
#define RA6M5_RTC_RMONAR_OFFSET     0x001A      /* Month Alarm Register */
#define RA6M5_RTC_RYRAR_OFFSET      0x001C      /* Year Alarm Register */
#define RA6M5_RTC_RYRAREN_OFFSET    0x001E      /* Year Alarm Enable Register */
#define RA6M5_RTC_RCR1_OFFSET       0x0022      /* RTC Control Register 1 */
#define RA6M5_RTC_RCR2_OFFSET       0x0024      /* RTC Control Register 2 */
#define RA6M5_RTC_RCR4_OFFSET       0x0028      /* RTC Control Register 4 */
#define RA6M5_RTC_RFRH_OFFSET       0x002A      /* Frequency Register H */
#define RA6M5_RTC_RFRL_OFFSET       0x002C      /* Frequency Register L */
#define RA6M5_RTC_RADJ_OFFSET       0x002E      /* Time Error Adjustment Register */
#define RA6M5_RTC_RTCCR0_OFFSET     0x0040      /* Time Capture Control Register 0 */
#define RA6M5_RTC_RTCCR1_OFFSET     0x0042      /* Time Capture Control Register 1 */
#define RA6M5_RTC_RTCCR2_OFFSET     0x0044      /* Time Capture Control Register 2 */
#define RA6M5_RTC_CP0_OFFSET        0x0050      /* Capture registers 0 */
#define RA6M5_RTC_RSEC0_OFFSET      0x0052      /* Second Capture Register 0 */
#define RA6M5_RTC_RMIN0_OFFSET      0x0054      /* Minute Capture Register 0 */
#define RA6M5_RTC_RHR0_OFFSET       0x0056      /* Hour Capture Register 0 */
#define RA6M5_RTC_RDAY0_OFFSET      0x005A      /* Date Capture Register 0 */
#define RA6M5_RTC_RMON0_OFFSET      0x005C      /* Month Capture Register 0 */
#define RA6M5_RTC_CP1_OFFSET        0x0060      /* Capture registers 1 */
#define RA6M5_RTC_RSEC1_OFFSET      0x0062      /* Second Capture Register 1 */
#define RA6M5_RTC_RMIN1_OFFSET      0x0064      /* Minute Capture Register 1 */
#define RA6M5_RTC_RHR1_OFFSET       0x0066      /* Hour Capture Register 1 */
#define RA6M5_RTC_RDAY1_OFFSET      0x006A      /* Date Capture Register 1 */
#define RA6M5_RTC_RMON1_OFFSET      0x006C      /* Month Capture Register 1 */
#define RA6M5_RTC_CP2_OFFSET        0x0070      /* Capture registers 2 */
#define RA6M5_RTC_RSEC2_OFFSET      0x0072      /* Second Capture Register 2 */
#define RA6M5_RTC_RMIN2_OFFSET      0x0074      /* Minute Capture Register 2 */
#define RA6M5_RTC_RHR2_OFFSET       0x0076      /* Hour Capture Register 2 */
#define RA6M5_RTC_RDAY2_OFFSET      0x007A      /* Date Capture Register 2 */
#define RA6M5_RTC_RMON2_OFFSET      0x007C      /* Month Capture Register 2 */

/* Register Bitfield Definitions ********************************************/

/* RTC Control Register 1 */

#define RTC_RCR1_AIE            (1 << 0)    /* Bit 0: Alarm Interrupt Enable */
#define RTC_RCR1_CIE            (1 << 1)    /* Bit 1: Carry Interrupt Enable */
#define RTC_RCR1_PIE            (1 << 2)    /* Bit 2: Periodic Interrupt Enable */
#define RTC_RCR1_RTCOS          (1 << 3)    /* Bit 3: RTCOUT Output Select */
#define RTC_RCR1_PES_SHIFT      (4)         /* Bits[7:4]: Periodic Interrupt Select */
#define RTC_RCR1_PES_MASK       (15 << RTC_RCR1_PES_SHIFT)
#  define RTC_RCR1_PES(n)       (((n) << RTC_RCR1_PES_SHIFT) & RTC_RCR1_PES_MASK)

/* RTC Control Register 2 */

#define RTC_RCR2_START          (1 << 0)    /* Bit 0: Start */
#define RTC_RCR2_RESET          (1 << 1)    /* Bit 1: RTC Software Reset */
#define RTC_RCR2_ADJ30          (1 << 2)    /* Bit 2: 30-Second Adjustment */
#define RTC_RCR2_RTCOE          (1 << 3)    /* Bit 3: RTCOUT Output Enable */
#define RTC_RCR2_AADJE          (1 << 4)    /* Bit 4: Automatic Adjustment Enable */
#define RTC_RCR2_AADJP          (1 << 5)    /* Bit 5: Automatic Adjustment Period Select */
#define RTC_RCR2_HR24           (1 << 6)    /* Bit 6: Hours Mode */
#define RTC_RCR2_CNTMD          (1 << 7)    /* Bit 7: Count Mode Select */

/* RTC Control Register 4 */

#define RTC_RCR4_RCKSEL         (1 << 0)    /* Bit 0: Count Source Select */


#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_RTC_H */
