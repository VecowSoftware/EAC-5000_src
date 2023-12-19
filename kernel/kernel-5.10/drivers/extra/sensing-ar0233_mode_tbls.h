/*
 * sensing-ar0233_mode_tbls.h - ar0233 sensor mode tables
 *
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __AR0233_I2C_TABLES__
#define __AR0233_I2C_TABLES__

#include <media/camera_common.h>

#define AR0233_TABLE_WAIT_MS 0xff00
#define AR0233_TABLE_END 0xff01
#define AR0233_MAX_RETRIES 3
#define AR0233_WAIT_MS_STOP 1
#define AR0233_WAIT_MS_START 30
#define AR0233_WAIT_MS_STREAM 210
#define AR0233_GAIN_TABLE_SIZE 255

#define ar0233_reg struct reg_16

static ar0233_reg ar0233_start[] = {{AR0233_TABLE_END, 0x00}};

static ar0233_reg ar0233_stop[] = {{AR0233_TABLE_END, 0x00}};
static ar0233_reg ar0233_1920x1080_crop_30fps_hdr[] = {
    {AR0233_TABLE_END, 0x00},
};
enum {
  AR0233_MODE_1920X1080_CROP_30FPS_HDR,
  AR0233_MODE_START_STREAM,
  AR0233_MODE_STOP_STREAM,
};

static ar0233_reg *mode_table[] = {
    [AR0233_MODE_1920X1080_CROP_30FPS_HDR] = ar0233_1920x1080_crop_30fps_hdr,
    [AR0233_MODE_START_STREAM] = ar0233_start,
    [AR0233_MODE_STOP_STREAM] = ar0233_stop,
};

static const int ar0233_30fps[] = {
    30,
};

static const struct camera_common_frmfmt ar0233_frmfmt[] = {
    {{1920, 1080}, ar0233_30fps, 1, 0, AR0233_MODE_1920X1080_CROP_30FPS_HDR},
};
#endif /* __AR0233_I2C_TABLES__ */
