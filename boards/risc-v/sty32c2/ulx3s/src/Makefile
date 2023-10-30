############################################################################
# boards/risc-v/sty32c2/ulx3s/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS  = ulx3s_bringup.c ulx3s_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += ulx3s_appinit.c
endif

ifeq ($(CONFIG_STY32C2_SDIO),y)
CSRCS += ulx3s_sdio.c
endif

ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += ulx3s_automount.c
endif

ifeq ($(CONFIG_STY32C2_PWM),y)
CSRCS += ulx3s_pwm.c
endif

ifeq ($(CONFIG_STY32C2_APPLICATION_RAMDISK),y)
CSRCS += ulx3s_ramdisk.c
endif

include $(TOPDIR)/boards/Board.mk
