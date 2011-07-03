/*
 *  include/linux/mmc/core/mmc_simple.h
 *
 * Simple MMC framework implementation that does not depend on the kernel MMC
 * framework.  The purpose is to be able to access an MMC/SD/SDIO device after
 * the kernel has panicked (no scheduling).
 *
 * Copyright (c) 2010, Motorola.
 *
 * Authors:
 *	Russ W. Knize	<russ.knize@motorola.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef __MMC_SIMPLE_H__
#define __MMC_SIMPLE_H__

#include <linux/mmc/host.h>

extern int mmc_simple_init(int, sector_t, sector_t, size_t);
extern int mmc_simple_add_host(struct mmc_host *);
extern size_t mmc_simple_read(char *, size_t, size_t);
extern size_t mmc_simple_write(char *, size_t, size_t);

/* The externel platform driver needs to implement this. */
extern int mmc_simple_platform_init(int);

#endif /* __MMC_SIMPLE_H__ */
