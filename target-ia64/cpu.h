/*
 * IA64 virtual CPU header
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 *  Copyright (c) 2007 Intel Corporation
 *  Zhang xiantao <xiantao.zhang@intel.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef CPU_IA64_H
#define CPU_IA64_H
#include "config.h"
#include "ia64intrin.h"

#include<string.h>
#include<stdio.h>

#define TARGET_LONG_BITS 64


#define TARGET_PAGE_BITS 14

#define ELF_MACHINE	EM_IA_64

#define CPU_PAL_HALT 1
#define HF_HALTED_MASK       (1 << CPU_PAL_HALT)

#include "cpu-defs.h"

#include "softfloat.h"
typedef struct CPUIA64State {
    CPU_COMMON;
    /* exception/interrupt handling */
   jmp_buf jmp_env;
   int exception_index;

   int interrupt_request;
   int user_mode_only;
   uint32_t hflags;

#ifdef USE_KVM
      uint8_t ready_for_interrupt_injection;
#endif

} CPUIA64State;

#define CPUState CPUIA64State
#define cpu_gen_code cpu_ia64_gen_code
#define cpu_init cpu_ia64_init
#define cpu_signal_handler cpu_ia64_signal_handler

struct CPUIA64State *env;
int cpu_get_pic_interrupt(CPUIA64State *s);
int cpu_exec(CPUState *env1);
void cpu_dump_state(CPUState *env, FILE *f,
                    int (*cpu_fprintf)(FILE *f, const char *fmt, ...),
                    int flags);

CPUState *cpu_ia64_init(void);

#include "cpu-all.h"

/* IA64 has seperate I/D cache, with coherence maintained by DMA controller.
 * So to emulate right behavior that guest OS is assumed, we need to flush
 * I/D cache here.
 */
void kvm_sync_icache(unsigned long address, int len);
#endif