/*
 * Copyright (C) 2009-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __LINUX_NVODMCAM_H__
#define __LINUX_NVODMCAM_H__

#define NVODMCAM_MAGIC 0xF1

/* List of ioctl commands */
#define NVODMCAM_IOCTL_GET_NUM_CAM          _IOR(NVODMCAM_MAGIC, 100, int)

#define NVODMCAM_IOCTL_CAMERA_OFF           _IO(NVODMCAM_MAGIC, 200)
#define NVODMCAM_IOCTL_CAMERA_ON            _IOW(NVODMCAM_MAGIC, 201, int)

#endif

