/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_uart.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_UART_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RXTX_OFFSET        0x00
#define UART_TXFULL_OFFSET      0x04
#define UART_RXEMPTY_OFFSET     0x08
#define UART_EV_STATUS_OFFSET   0x0c
#define UART_EV_PENDING_OFFSET  0x10
#define UART_EV_ENABLE_OFFSET   0x14

#define UART_EV_TX	            0x01
#define UART_EV_RX	            0x02

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(HAVE_SERIAL_CONSOLE)
/****************************************************************************
 * Name: rv32m1_console_uart_setup
 ****************************************************************************/

EXTERN void sty32c2_console_uart_setup(void);

/****************************************************************************
 * Name: rv32m1_console_uart_putc
 ****************************************************************************/

EXTERN void sty32c2_console_uart_putc(char);
#endif /* HAVE_SERIAL_CONSOLE */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* _ARCH_RISCV_SRC_STY32C2_CHIP_STY32C2_UART_H */
