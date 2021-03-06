// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2009
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <timer.h>
#include <watchdog.h>
#include <div64.h>
#include <asm/io.h>

#ifndef CONFIG_WD_PERIOD
# define CONFIG_WD_PERIOD	(10 * 1000 * 1000)	/* 10 seconds default */
#endif

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SYS_TIMER_RATE
/* Returns tick rate in ticks per second */
ulong notrace get_tbclk(void)
{
	return CONFIG_SYS_TIMER_RATE;
}
#endif

#ifdef CONFIG_SYS_TIMER_COUNTER
unsigned long notrace timer_read_counter(void)
{
#ifdef CONFIG_SYS_TIMER_COUNTS_DOWN
	return ~readl(CONFIG_SYS_TIMER_COUNTER);
#else
	return readl(CONFIG_SYS_TIMER_COUNTER);
#endif
}

ulong timer_get_boot_us(void)
{
	ulong count = timer_read_counter();

#if CONFIG_SYS_TIMER_RATE == 1000000
	return count;
#elif CONFIG_SYS_TIMER_RATE > 1000000
	return lldiv(count, CONFIG_SYS_TIMER_RATE / 1000000);
#elif defined(CONFIG_SYS_TIMER_RATE)
	return (unsigned long long)count * 1000000 / CONFIG_SYS_TIMER_RATE;
#else
	/* Assume the counter is in microseconds */
	return count;
#endif
}

#else
extern unsigned long __weak timer_read_counter(void);
#endif

//#endif /* CONFIG_TIMER */

/* Returns time in milliseconds */
static uint64_t notrace tick_to_time(uint64_t tick)
{
	ulong div = get_tbclk();

	tick *= CONFIG_SYS_HZ;
	do_div(tick, div);
	return tick;
}

unsigned long __weak notrace timer_get_us(void)
{
	return tick_to_time(get_ticks() * 1000);
}

static uint64_t usec_to_tick(unsigned long usec)
{
	uint64_t tick = usec;
	tick *= get_tbclk();
	do_div(tick, 1000000);
	return tick;
}

void __weak __udelay(unsigned long usec)
{
	uint64_t tmp;

	tmp = get_ticks() + usec_to_tick(usec);	/* get current timestamp */

	while (get_ticks() < tmp+1)	/* loop till event */
		 /*NOP*/;
}

/* ------------------------------------------------------------------------- */

void udelay(unsigned long usec)
{
	ulong kv;

	do {
		WATCHDOG_RESET();
		kv = usec > CONFIG_WD_PERIOD ? CONFIG_WD_PERIOD : usec;
		__udelay (kv);
		usec -= kv;
	} while(usec);
}
