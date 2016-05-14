/* Copyright (c) 2012 LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_LGE_HANDLE_PANIC_H
#define __MACH_LGE_HANDLE_PANIC_H

/*                                     */
#define LGE_RB_MAGIC    	0x6D630000
#define LGE_ERR_KERN    	0x0100
#define LGE_HIDDEN_RESET	0x0200
#define LGE_MODEM_CRASH		0x0300

#define LAF_DLOAD_MODE   0x6C616664 /* lafd */

void lge_set_panic_reason(void);
void lge_set_restart_reason(unsigned int);

#endif
