/* linux/arch/arm/mach-s5pv210/include/mach/media.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Samsung Media device descriptions for smdkv210
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _S5PV210_MEDIA_H
#define _S5PV210_MEDIA_H

/* 3 fimc indexes should be fixed as n, n+1 and n+2 */
#define S5P_MDEV_MFC        0
#define S5P_MDEV_FIMC0      1
#define S5P_MDEV_FIMC1      2
#define S5P_MDEV_FIMC2      3
#define S5P_MDEV_FIMD       4
#define S5P_MDEV_JPEG       5
#define S5P_MDEV_TEXSTREAM  6

#ifdef CONFIG_ANDROID_PMEM
#define S5P_MDEV_PMEM       7
#define S5P_MDEV_PMEM_GPU1  8
#endif

#endif

