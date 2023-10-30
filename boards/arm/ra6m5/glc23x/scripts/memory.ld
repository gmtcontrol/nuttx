/****************************************************************************
 * boards/arm/ra6m5/glc23x/scripts/memory.ld
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

/* TODO: Add description for memory organisation */

MEMORY
{
  /* 2048Kb FLASH */

  kflash (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  uflash (rx) : ORIGIN = 0x00040000, LENGTH = 256K
  xflash (rx) : ORIGIN = 0x00080000, LENGTH = 2048K - 512K

  /* 512Kb SRAM */

  ksram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
  usram (rwx) : ORIGIN = 0x20004000, LENGTH = 16K
  xsram (rwx) : ORIGIN = 0x20008000, LENGTH = 512K - 32K
}
