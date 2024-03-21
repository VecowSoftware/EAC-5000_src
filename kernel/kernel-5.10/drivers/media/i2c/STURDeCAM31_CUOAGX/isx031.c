/*[20:24] Koil Arul Raj  S
static uint8_t num_cam = 0;

 * isx031.c - isx031 sensor driver
 *
 * Copyright (c) 2022-2023, Leopard CORPORATION.  All rights reserved.
 * Base on Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/firmware.h>

#include <media/tegracam_core.h>
#include "isx031_mode_tbls.h"
#include <media/camera_common.h>
#include "../../../../nvidia/drivers/media/platform/tegra/camera/camera_gpio.h"

#include "isx031.h"
static uint8_t num_cam = 0;

const struct firmware *mcu_fw;
static uint8_t is_fw_loaded = 0;
unsigned char *mcu_fw_buf = NULL;
const char *mcu_fw_name = NULL;

static uint8_t is_fw_loaded_ISP = 0;
const struct firmware *mcu_fw_ISP;
unsigned char *mcu_fw_buf_ISP = NULL;
const char *mcu_fw_name_ISP = NULL;

#define VERSION_SIZE			32
#define VERSION_FILE_OFFSET			99
#define VERSION_FILE_OFFSET_SPI			( VERSION_FILE_OFFSET + 92 )
#define MCU_RST_REG 0x02CA

#define SET_SER_GPIO 0x98
#define RST_SER_GPIO 0x40

#define PWM_30HZ_PERIOD  33333333
#define PWM_30HZ_DUTY    3333333

#include "serdes.h"
#include "mcu_firmware.h"

#include <linux/pwm.h>


#define DEBUG_PRINTK
#ifndef DEBUG_PRINTK
#define debug_printk(s , ... )
#else
#define debug_printk(s, ... ) printk( s, ##__VA_ARGS__)
#endif


/* */

#define ISX031_MIN_GAIN         (0)
#define ISX031_MAX_GAIN         (30)
#define ISX031_MAX_GAIN_REG     ((ISX031_MAX_GAIN - ISX031_MIN_GAIN) * 10 / 3)
#define ISX031_DEFAULT_FRAME_LENGTH    (1125)
#define ISX031_FRAME_LENGTH_ADDR_MSB    0x200A
#define ISX031_FRAME_LENGTH_ADDR_MID    0x2009
#define ISX031_FRAME_LENGTH_ADDR_LSB    0x2008
#define ISX031_COARSE_TIME_SHS1_ADDR_MSB    0x000E
#define ISX031_COARSE_TIME_SHS1_ADDR_MID    0x000D
#define ISX031_COARSE_TIME_SHS1_ADDR_LSB    0x000C
#define ISX031_COARSE_TIME_SHS2_ADDR_MSB    0x0012
#define ISX031_COARSE_TIME_SHS2_ADDR_MID    0x0011
#define ISX031_COARSE_TIME_SHS2_ADDR_LSB    0x0010
#define ISX031_GROUP_HOLD_ADDR			0x0008
#define ISX031_ANALOG_GAIN_SP1H_ADDR	0x0018
#define ISX031_ANALOG_GAIN_SP1L_ADDR	0x001A


static struct of_device_id cam_of_match[] = {
	{.compatible = "sony,isx031",},
	{},
};


static const struct of_device_id isx031_of_match[] = {
	{ .compatible = "sony,isx031",},
	{ },
};
MODULE_DEVICE_TABLE(of, isx031_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_HDR_EN,
};

struct isx031 {

	struct tegracam_device          *tc_dev;
	struct camera_common_power_rail power;
	int numctrls;
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	struct media_pad pad;

	int reg_offset;

	s32 group_hold_prev;
	bool group_hold_en;
	struct regmap *b_regmap;
	struct regmap *w_regmap;
	struct regmap *dw_regmap;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;
	int ident;
	u16 chip_id;
	u8 revision;

	uint16_t frate_index;
	uint32_t format_fourcc;
	int frmfmt_mode;
	int num_ctrls;
	ISP_STREAM_INFO *stream_info;
	ISP_CTRL_INFO *mcu_ctrl_info;
	/* Total formats */
	int *streamdb;
	uint32_t *ctrldb;
	/* Array of Camera framesizes */
	struct camera_common_frmfmt *mcu_cam_frmfmt;
	uint16_t prev_index;
	uint16_t mipi_lane_config;
	uint32_t mipi_clk_config;
	uint8_t last_sync_mode;
	uint8_t phy;
	uint8_t ser_addr;
	uint8_t des_addr;
	uint8_t tb_id;
	uint8_t ser_status;
	uint8_t cam_index;
	uint8_t des_link_status;
	uint8_t cam_order_num;
	struct mutex mcu_i2c_mutex;
	
	uint8_t trig_val;
	int trigger_gpio;
	struct pwm_device *pwm_dev;
	bool got_pwm_handle;


	struct v4l2_ctrl *ctrls[];	


};
static const struct v4l2_ctrl_ops cam_ctrl_ops = {
	.g_volatile_ctrl = cam_g_volatile_ctrl,
	.s_ctrl = cam_s_ctrl,
};

static int cam_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx031 *priv =
		container_of(ctrl->handler, struct isx031, ctrl_handler);
	struct i2c_client *client = priv->i2c_client;
	int err = 0;

	uint8_t ctrl_type = 0;
	int ctrl_val = 0;
	if (!priv || !priv->pdata)
		return -EINVAL;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	if ((err = mcu_get_ctrl(client, ctrl->id, &ctrl_type, &ctrl_val)) < 0) {
		return err;
	}

	if (ctrl_type == CTRL_STANDARD) {
		ctrl->val = ctrl_val;
	} else {
		/* Not Implemented */
		return -EINVAL;
	}

	return err;
}

static int mcu_cam_stream_off(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	/* call ISP init command */
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	uint8_t mc_data[512], mc_ret_data[512];

	/*lock semaphore*/
	mutex_lock(&priv->mcu_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) CAM Stream OFF Write Error - %d \n", __func__,
				__LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		yield();

		cmd_id = CMD_ID_STREAM_OFF;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
				0) {
			dev_err(&client->dev," %s(%d) CAM Get CMD Stream Off Error \n",
				       	__func__,__LINE__);
			err = -1;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			debug_printk(" %s %d CAM Get CMD Stream off Success !! \n",
				       	__func__, __LINE__ );
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d CAM Get CMD Stream off"
				       	"Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			err = -1;
			goto exit;
		}
		mdelay(1);
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);
	return err;
}

static int cam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx031 *priv =
		container_of(ctrl->handler, struct isx031, ctrl_handler);
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	int err = 0, mode = 0, retry = 5;

	mode = s_data->mode;


	if (!priv || !priv->pdata)
		return -EINVAL;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	if(ctrl->id == V4L2_CID_CUSTOM_TRIGGER){
		if(ctrl->val == 1 && ctrl->val != priv->trig_val){
				priv->trig_val = 1;
				toggle_gpio(priv->trigger_gpio, 1);
		} else {
				toggle_gpio(priv->trigger_gpio, 0);
				priv->trig_val = 0;
		}
		return 0;
	}
	
	if(ctrl->id == V4L2_CID_FRAME_SYNC) {
		if(ctrl->val == 1){
		if(priv->got_pwm_handle) {
	                // Set PWM Trigger to default
        	        pwm_disable(priv->pwm_dev);
               		pwm_config(priv->pwm_dev, PWM_30HZ_DUTY, PWM_30HZ_PERIOD);
               		pwm_enable(priv->pwm_dev);
               		priv->last_sync_mode = 1;
               		dev_info(&client->dev, "PWM 30Hz set\n");
		        }

			priv->last_sync_mode = 1;
		}
	}

	while(retry -- > 0) {
		if ((err =
			mcu_set_ctrl(client, ctrl->id, CTRL_STANDARD, ctrl->val)) < 0) {
			dev_info(&client->dev," %s (%d ) retry \n", __func__, __LINE__);
			if(retry <= 0) {
				dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
				break;
			} else {
				continue;
			}
		}
		break;
	}

	return err;
}


static int mcu_cam_stream_on(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 5,status_retry=1000, err = 0;
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	uint8_t mc_data[512], mc_ret_data[512];

	/*lock semaphore*/
	mutex_lock(&priv->mcu_i2c_mutex);

	while(retry-- < 0) {
		/* First Txn Payload length = 0 */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err= cam_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU Stream On Write Error - %d \n",
				       	__func__,__LINE__, err);
			continue;
		}

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU Stream On Write Error - %d \n",
				       	__func__,__LINE__, err);
			continue;
		}

		while (status_retry-- > 0) {
			/* Some Sleep for init to process */
			yield();

			cmd_id = CMD_ID_STREAM_ON;
			if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
					0) {
				dev_err(&client->dev," %s(%d) MCU Get CMD Stream On Error \n",
					       __func__,__LINE__);
				err = -1;
				goto exit;
			}

			if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
					(retcode == ERRCODE_SUCCESS)) {
				debug_printk(" %s %d MCU Stream On Success !! \n",
					       	__func__, __LINE__);
				err = 0;
				goto exit;
			}

			if ((retcode != ERRCODE_BUSY) &&
					((cmd_status != MCU_CMD_STATUS_PENDING))) {
				dev_err(&client->dev,
						"(%s) %d MCU Get CMD Stream On Error"
					       	"STATUS = 0x%04x RET = 0x%02x\n",
						__func__, __LINE__, cmd_status, retcode);
				err = -1;
				goto exit;
			}
			mdelay(1);
		}
		if(retry == 0) 
			err = -1;
		break;
	}
	msleep(10);
exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);
	return err;

}

static int cam_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	int err = 0;

	printk("%s is called , enable=%d\n",__func__,enable);
	if (!priv || !priv->pdata)
		return -EINVAL;

	// Increment the refs count when streaming 
	// and decrement when streaming is disabled
	if (enable) {
		if (!try_module_get(s_data->owner))
			return -ENODEV;
	} else {
		module_put(s_data->owner);
	}


	if (!enable) {
		/* Perform Stream Off Sequence - if any */
		err = mcu_cam_stream_off(client);
			/* Reset Frame rate index */
		priv->frate_index = 0;

		return err;
	}
	/* Perform Stream On Sequence - if any  */

	err = mcu_cam_stream_on(client);{
		if(err!= 0){
			dev_err(&client->dev,"%s (%d) Stream_On \n",
					__func__, __LINE__);
			return err;
		}
	}
	mdelay(10);
	return 0;
}

static int cam_g_input_status(struct v4l2_subdev *sd, u32 * status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	if (!priv || !priv->pdata)
		return -EINVAL;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static int gen_mcu_stream_config(struct i2c_client *client,struct isx031 *priv)
{
	int  err = 0, retry = 5;
	uint16_t data = 0;

	while (retry-- > 0) {
		
		err = mcu_stream_config(client, priv->format_fourcc, priv->frmfmt_mode,
					priv->frate_index);
		if (err < 0) {
			dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
			if(retry != 0)
				continue;
			if(err < 0){
				dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
				return err;
			}
		}

		break;
	}

	if(retry <= 0) {
		dev_err(&client->dev, "%s(%d): Failed \n", 						
				__func__, __LINE__);
		return err;
	}

	return 0;

}
static int cam_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;

	if (!priv || !priv->pdata) {
		return -ENOTTY;
	}

	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;

	param->parm.capture.timeperframe.denominator =
		priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frate_index];
	param->parm.capture.timeperframe.numerator = 1;

	return 0;
}


static int cam_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	int ret = 0, err = 0, retry = 3;
	uint16_t data = 0;

	if (!priv || !priv->pdata) {
		return -EINVAL;
	}

	for (ret = 0; ret < priv->mcu_cam_frmfmt[priv->frmfmt_mode].num_framerates;
			ret++) {
		if ((priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[ret] ==
					param->parm.capture.timeperframe.denominator)) {
			priv->frate_index = ret;

			param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
			param->parm.capture.timeperframe.denominator = 
				priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frate_index]; 
			param->parm.capture.timeperframe.numerator = 1;

			err = gen_mcu_stream_config(client, priv);

			if(err < 0){
				dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
				return err;
			}

		}
	}

	/* if S_PARM is called with invalid parameters, 
	 * set the right parameters and return success */
	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.timeperframe.denominator =
		priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frate_index]; 
	param->parm.capture.timeperframe.numerator = 1;	

	return 0;
}
static struct v4l2_subdev_video_ops cam_subdev_video_ops = {
	.s_stream = cam_s_stream,
	//.g_mbus_config = camera_common_g_mbus_config,
	.g_input_status = cam_g_input_status,
	.g_parm = cam_g_parm,
	.s_parm = cam_s_parm,
};

static struct v4l2_subdev_core_ops cam_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

static int cam_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int cam_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	int flag = 0, err = 0, retry = 3;
	uint16_t data = 0;
	if (!priv || !priv->pdata)
		return -EINVAL;
	switch (format->format.code) {
		case MEDIA_BUS_FMT_UYVY8_1X16:
			priv->format_fourcc = V4L2_PIX_FMT_UYVY;
			break;

		default:
			/* Not Implemented */
			if (format->which != V4L2_SUBDEV_FORMAT_TRY) {		
				return -EINVAL;
			}
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ret = camera_common_try_fmt(sd, &format->format);
	printk("Try_fmt called\n");		
	} else {

	printk("Set_fmt called\n");		
		for (ret = 0; ret < s_data->numfmts; ret++) {
			if ((priv->mcu_cam_frmfmt[ret].size.width == format->format.width)
					&& (priv->mcu_cam_frmfmt[ret].size.height ==
						format->format.height)) {
				priv->frmfmt_mode = priv->mcu_cam_frmfmt[ret].mode;
				flag = 1;
				break;
			}
		}

		if(flag == 0) {
			return -EINVAL;
		}

		err = gen_mcu_stream_config(client, priv);
		if(err < 0){
			dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
			return err;
		}

		ret = camera_common_s_fmt(sd, &format->format);
	}
	//conditional deserializer configuration
	if(num_cam==1)
	{
		if((serdes_write_16b_reg(client, priv->des_addr, 0x0320, 0x6d)) < 0) {
                        dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
                        return -EIO;
                }
                msleep(10);
		if((serdes_write_16b_reg(client, priv->des_addr, 0x0323, 0x6d)) < 0) {
                        dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
                        return -EIO;
                }
                msleep(10);
	}
	else
	{
                if((serdes_write_16b_reg(client, priv->des_addr, 0x0320, 0x72)) < 0) {
                        dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
                        return -EIO;
                }
                msleep(10);
                if((serdes_write_16b_reg(client, priv->des_addr, 0x0323, 0x72)) < 0) {
                        dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
                        return -EIO;
                }
                msleep(10);
	}
	
	if (priv->phy == PHY_A) {
		/* Resetting PIPE-Y for SER1 */
		if((serdes_write_16b_reg(client, priv->des_addr, 0x0002, 0x43)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
            		return -EIO;
		}
        	msleep(10);
    		if((serdes_write_16b_reg(client, priv->des_addr, 0x0002, 0x63)) < 0) {
            		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
        		return -EIO;
    		}
    		msleep(100);
    	} else if(priv->phy == PHY_B) {
		/* Resetting PIPE-Z for SER2 */
        	if((serdes_write_16b_reg(client, priv->des_addr, 0x0002, 0x23)) < 0) {
            		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
            		return -EIO;
		}
        	msleep(10);
        	if((serdes_write_16b_reg(client, priv->des_addr, 0x0002, 0x63)) < 0) {
        		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
        		return -EIO;
        	}
        	msleep(100);
        }

	return ret;
}

static struct v4l2_subdev_pad_ops cam_subdev_pad_ops = {
	.enum_mbus_code = camera_common_enum_mbus_code,
	.set_fmt = cam_set_fmt,
	.get_fmt = cam_get_fmt,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops cam_subdev_ops = {
	.core = &cam_subdev_core_ops,
	.video = &cam_subdev_video_ops,
	.pad = &cam_subdev_pad_ops,
};

static int cam_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops cam_subdev_internal_ops = {
	.open = cam_open,
};

static const struct media_entity_operations cam_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static inline void isx031_get_frame_length_regs(isx031_reg *regs,
				u32 frame_length)
{
	regs->addr = ISX031_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x01;

	(regs + 1)->addr = ISX031_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xff;

	(regs + 2)->addr = ISX031_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void isx031_get_coarse_time_regs_shs1(isx031_reg *regs,
				u32 coarse_time)
{
	regs->addr = ISX031_COARSE_TIME_SHS1_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x0f;

	(regs + 1)->addr = ISX031_COARSE_TIME_SHS1_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = ISX031_COARSE_TIME_SHS1_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;
}

static inline void isx031_get_coarse_time_regs_shs2(isx031_reg *regs,
				u32 coarse_time)
{
	regs->addr = ISX031_COARSE_TIME_SHS2_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x0f;

	(regs + 1)->addr = ISX031_COARSE_TIME_SHS2_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = ISX031_COARSE_TIME_SHS2_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;
}

static inline void isx031_get_gain_reg(isx031_reg *regs,
				u16 gain)
{
	regs->addr = ISX031_ANALOG_GAIN_SP1H_ADDR;
	regs->val = (gain) & 0xff;

	(regs + 1)->addr = ISX031_ANALOG_GAIN_SP1H_ADDR + 1;
	(regs + 1)->val = (gain >> 8) & 0xff;

	(regs + 2)->addr = ISX031_ANALOG_GAIN_SP1L_ADDR;
	(regs + 2)->val = (gain) & 0xff;

	(regs + 3)->addr = ISX031_ANALOG_GAIN_SP1L_ADDR + 1;
	(regs + 3)->val = (gain >> 8) & 0xff;
}


static int test_mode;
module_param(test_mode, int, 0644);

static int cam_read(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}
unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
	}

	return crc;
}
static int cam_write(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register ret = %d!\n",
				ret);
		return ret;
	}

	return 0;
}

int mcu_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9') {
		return (ascii - '0');
	} else if ((ascii >= 'a') && (ascii <= 'f')) {
		return (0xA + (ascii - 'a'));
	} else if ((ascii >= 'A') && (ascii <= 'F')) {
		return (0xA + (ascii - 'A'));
	}
	return -1;
}


static int mcu_jump_bload(struct i2c_client *client)
{
	uint32_t payload_len = 0;
	int err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/*lock semaphore */
	mutex_lock(&g_i2c_mutex);
	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);
	if (err !=0 ) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
				__func__, __LINE__, err);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
				__func__, __LINE__, err);
		goto exit;
	} 

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	return err;

}

static inline int isx031_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int isx031_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int isx031_write_table(struct isx031 *priv,
				const isx031_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 ISX031_TABLE_WAIT_MS,
					 ISX031_TABLE_END);
}

static int isx031_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	if (!priv || !priv->pdata)
		return -EINVAL;
	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			dev_err(&priv->i2c_client->dev,"%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto cam_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto cam_iovdd_fail;

	usleep_range(1350, 1360);

	pw->state = SWITCH_ON;
	return 0;

cam_iovdd_fail:
	regulator_disable(pw->avdd);

cam_avdd_fail:
	dev_err(&priv->i2c_client->dev,"%s failed.\n", __func__);
	return -ENODEV;
}

static int isx031_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

        if (priv->pdata && priv->pdata->power_off) {
                err = priv->pdata->power_off(pw);
                if (err)
                        dev_err(&priv->i2c_client->dev, "%s failed.\n", __func__);
                else
                        goto power_off_done;
        }

        if (pw->avdd) {
                regulator_disable(pw->avdd);
        }

        if (pw->iovdd) {
                regulator_disable(pw->iovdd);
        }

        return err;

power_off_done:
        pw->state = SWITCH_OFF;
        return 0;
}

static int isx031_power_get(struct isx031 *priv)
{

	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;

	mclk_name =
		priv->pdata->mclk_name ? priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev, "unable to get clock %s\n",
				mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent))
			dev_err(&priv->i2c_client->dev,
					"unable to get parent clcok %s",
					parentclk_name);
		else
			clk_set_parent(pw->mclk, parent);
	}


	err |=
		camera_common_regulator_get(&priv->i2c_client->dev, &pw->avdd,
				pdata->regulators.avdd);

	err |=
		camera_common_regulator_get(&priv->i2c_client->dev, &pw->iovdd,
				pdata->regulators.iovdd);

	pw->state = SWITCH_OFF;
	return err;
}

static int isx031_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static int isx031_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	return 0;
}

static int isx031_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int isx031_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int isx031_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static struct tegracam_ctrl_ops isx031_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = isx031_set_gain,
	.set_exposure = isx031_set_exposure,
	.set_exposure_short = isx031_set_exposure,
	.set_frame_rate = isx031_set_frame_rate,
	.set_group_hold = isx031_set_group_hold,
};

static struct camera_common_pdata
*isx031_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(isx031_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	return board_priv_pdata;
}

static int isx031_set_mode(struct tegracam_device *tc_dev)
{
	struct isx031 *priv = (struct isx031 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	int err;

	match = of_match_device(isx031_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	if (s_data->mode_prop_idx < 0)
		return -EINVAL;

	err = isx031_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int isx031_start_streaming(struct tegracam_device *tc_dev)
{
	struct isx031 *priv = (struct isx031 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	/* enable serdes streaming */


	err = isx031_write_table(priv,
		mode_table[ISX031_MODE_START_STREAM]);
	if (err)
		return err;

	msleep(20);

	return 0;

//exit:
	dev_err(dev, "%s: error setting stream\n", __func__);

	return err;
}

static int isx031_stop_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct isx031 *priv = (struct isx031 *)tegracam_get_privdata(tc_dev);
	int err;

	/* disable serdes streaming */

	err = isx031_write_table(priv, mode_table[ISX031_MODE_STOP_STREAM]);
	if (err)
		return err;

	return 0;
}

static struct camera_common_sensor_ops isx031_common_ops = {
	.numfrmfmts = ARRAY_SIZE(isx031_frmfmt),
	.frmfmt_table = isx031_frmfmt,
	.power_on = isx031_power_on,
	.power_off = isx031_power_off,
	.write_reg = isx031_write_reg,
	.read_reg = isx031_read_reg,
	.parse_dt = isx031_parse_dt,
	//.power_get = isx031_power_get,
	.power_put = isx031_power_put,
	.set_mode = isx031_set_mode,
	.start_streaming = isx031_start_streaming,
	.stop_streaming = isx031_stop_streaming,
};

static int isx031_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops isx031_subdev_internal_ops = {
	.open = isx031_open,
};

#if 0
static int cam_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;

	pw->state = SWITCH_OFF;

	return 0;
}
#endif

static s32 serdes_parse_regdata(struct i2c_client *client, SERDES_PARSE * regdata,
		u32 reg_cnt,u8 serdes_id)
{
	int i = 0;

	for (i = 0; i < reg_cnt; i++) {
		if (regdata[i].reg == 0xFFFF) {
			mdelay(100);
			continue;
		}

		if ((serdes_write_16b_reg(client, serdes_id, regdata[i].reg, regdata[i].val)) <
				0) {
			dev_err(&client->dev, "%s(%d): Failed \n",
					__func__, __LINE__);
			return -EIO;
		}

	}

	return 0;
}


static s32 serdes_config_init(struct i2c_client *client,struct isx031 *priv)
{
	uint8_t slave_addr=0;

	if (priv->phy == PHY_A)
	{	 
		dev_info(&client->dev, " Issuing CHIP reset for Deserializer ... \n");
		if((serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x80)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);

		if((serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x21)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);

		if((serdes_write_16b_reg(client, SER1_ADDR, 0x0010, 0x21)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);
		if((serdes_read_16b_reg(client, SER1_ADDR, 0x0000, &slave_addr)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		if(SER1_ADDR != (slave_addr>>1)){
			/* Enabling Only LINKB */
			serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x22);
			msleep(100);
			debug_printk("serializer slave address read is=%x\n",slave_addr>>1);
			dev_err(&client->dev," No serializer found on SIOA\n");
			dev_err(&client->dev," Exiting probe\n");
			return -ENODEV;
		}
		priv->ser_addr = SER1_ADDR;

		/* SIOA port I2C address translation */
		if(serdes_parse_regdata(client, SER1_I2C_CONF, ARRAY_SIZE(SER1_I2C_CONF),
					priv->ser_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOA Serializer" 
					"I2C translation\n",__func__);
			return -EIO;
		}
		dev_info(&client->dev,"SIOA Port I2C translated successfully\n");

		/* Setting Boot pin low */
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02BE, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

			
/* SENSOR RESET CHECK */
		// SER:GPIO8 ---> SEN:GPIO0
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02D6, 0x98)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		// SER:GPIO7 ---> SEN:GPIO1
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02D3, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}


		// SER:GPIO3 ---> SEN:GPIO2
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02C7, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		/* Updating Serializer availabilty */
		ser_status = 0x21; 
	}
	else if(priv->phy == PHY_B)
	{
		if((serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x22)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);

		/* Checking Whether SIOB serializer I2C Reassignment is Already Done*/
		serdes_read_16b_reg(client, SER2_ADDR, 0x0000, &slave_addr);
		if(slave_addr == SER2_ADDR << 1)
		{
			dev_info(&client->dev,"I2C translate detected.. Skip i2c translate... \n");
			goto skip_translate;
		}	
		if((serdes_write_16b_reg(client, SER1_ADDR, 0x0010, 0x21)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			/* Enabling Only LINKA */
			serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x21);
			msleep(100);			
			return -EIO;
		}
		msleep(100);
		if((serdes_read_16b_reg(client, SER1_ADDR, 0x0000, &slave_addr)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		if(SER1_ADDR != (slave_addr>>1)){
			/* Enabling Only LINKA */
			serdes_write_16b_reg(client, priv->des_addr, 0x0010, 0x21);
			msleep(100);
			debug_printk("serializer slave address read is=%x\n",slave_addr>>1);
			dev_err(&client->dev," No serializer found on SIOB\n");
			dev_err(&client->dev," Exiting probe\n");
			return -ENODEV;
		}

		/*I2C Reassignement for SIOB port Serializer*/
		if((serdes_write_16b_reg(client, SER1_ADDR, 0x0000, SER2_ADDR<<1)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);

skip_translate:		
		dev_info(&client->dev,"SIOB Port I2C Reassignment successful\n");	
		priv->ser_addr = SER2_ADDR;
		msleep(100);

		/*Change GMSL2 Packet header*/
		if(serdes_parse_regdata(client,	SER2_PKT_HEADER_CHANGE, 
					ARRAY_SIZE(SER2_PKT_HEADER_CHANGE),
					priv->ser_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOA Serializer" 
					"I2C translation\n",__func__);
			return -EIO;
		}

		/* SIOB I2C Translation */
		if(serdes_parse_regdata(client, SER2_I2C_CONF, ARRAY_SIZE(SER2_I2C_CONF),
					priv->ser_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOA Serializer" 
					"I2C translation\n",__func__);
			return -EIO;
		}

		msleep(100);

		/*Set Boot pin low*/		
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02BE, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		debug_printk("SIOB Port I2C translated successfully\n");

		/* SENSOR RESET CHECK */
		// SER:GPIO8 ---> SEN:GPIO0
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02D6, 0x98)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		// SER:GPIO7 ---> SEN:GPIO1
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02D3, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}


		// SER:GPIO3 ---> SEN:GPIO2
		if((serdes_write_16b_reg(client, priv->ser_addr, 0x02C7, 0x40)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

		/* Updating Serializer availabilty */
		if (ser_status == 0x21)
			ser_status = 0x23;
		else
			ser_status = 0x22;	
	}
	else{
		dev_err(&client->dev,"Device tree SIOA ports Parse Unsuccessful\n");
		return -EINVAL;
	}
	return 0;
}


static struct camera_common_pdata *cam_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(cam_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata =
		devm_kzalloc(&client->dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(&client->dev, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}
	
	board_priv_pdata->use_cam_gpio =
		of_property_read_bool(node, "cam,use-cam-gpio");

	err =
		of_property_read_string(node, "avdd-reg",
				&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(&client->dev, "avdd-reg not in DT\n");
		goto error;
	}
	err =
		of_property_read_string(node, "iovdd-reg",
				&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(&client->dev, "iovdd-reg not in DT\n");
		goto error;
	}

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}


int ecam_specific_dt_parse(struct i2c_client *client, struct isx031 *priv)
{
	struct device_node *node = client->dev.of_node;	
	uint32_t mipi_lanes=0, mipi_clk = 0;
	int ret_val = 0;
	const char *str;

	/* get mcu firmware name to load the firmware */
	//printk("Going to get mcu firmware name from device tree\n");
	ret_val = of_property_read_string(node, "mcu_fw_name",&mcu_fw_name);
	if(!ret_val)
		printk("Firmware name from the device tree is > %s\n",
				mcu_fw_name);
	else
		printk("Unable to get mcu firmware name form Device tree\n");



	ret_val = of_property_read_string(node, "spi_mcu_fw_name",&mcu_fw_name_ISP);
	if(!ret_val)
		printk("ISP Firmware name from the device tree is > %s\n",
				mcu_fw_name_ISP);
	else
		printk("Unable to get ISP firmware name form Device tree\n");
	
	/*Identifying Deserializer SIO port for 
	  I2C Address Reassignment and Translation
	  */
	ret_val = of_property_read_string(node, "sio-port", &str);
	if (!ret_val) {
		if (!strcmp(str, "A")){
			priv->phy = PHY_A;
			debug_printk("Current SIO ports is %c\n",priv->phy);
			/* RESET status if PHYA */
			ser_status = 0;
		}
		else{
			priv->phy = PHY_B;
			debug_printk("Current SIO ports is %c\n",priv->phy);
		}	
	} else {
		dev_err(&client->dev,"No SIO port mentioned in device tree\n");
		return -EINVAL;	
	}
	ret_val= of_property_read_u32(node, "camera_mipi_lanes", &mipi_lanes);
	if (!ret_val) {
		debug_printk("Device No of MIPI lane configuration is %u\n",mipi_lanes);
	} else {
		dev_err(&client->dev,"No of MIPI lanes not mentioned in device tree\n");
		return -EINVAL;	
	}
	
	priv->mipi_lane_config = 4;

	return 0;

}

static int ecam_mcu_firmware_load(struct i2c_client *client)
{
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};
	int i= 0,loop = 0, ret = 0;
	unsigned long txt_fw_pos = 0;
	/* Request firmware from the rootfs */
	ret = request_firmware(&mcu_fw, mcu_fw_name, &client->dev);
	if(ret < 0)
		return -ENOENT; 
	txt_fw_pos = mcu_fw->size -VERSION_FILE_OFFSET;
	mcu_fw_buf = kmalloc(mcu_fw->size+1, GFP_KERNEL);
	mcu_fw_buf[mcu_fw->size] = '\0';
	memcpy(mcu_fw_buf, mcu_fw->data, mcu_fw->size); 

	return 0;
}

static int mcu_get_fw_key(struct i2c_client *client, unsigned char *fw_key)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0, i, j;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_FW_KEY;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	mdelay(50);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_FW_KEY;
	msleep(10);
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	msleep(10);
	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;


	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], (payload_len - HEADER_FOOTER_SIZE));
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	for (i=2,j=0 ; i<payload_len-1; i++,j++)
		fw_key[j] = (unsigned char)mc_ret_data[i];

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}


static int ecam_mcu_firmware_load_ISP(struct i2c_client *client)
{
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};
	int i= 0,key_size = 0,loop = 0, ret = 0;
	unsigned long txt_fw_pos = 0;
	unsigned char key[16];
	/* Request firmware from the rootfs */
	ret = request_firmware(&mcu_fw_ISP, mcu_fw_name_ISP, &client->dev);
	if(ret < 0)
		return -ENOENT;
        mcu_get_fw_key(client, key);	
	txt_fw_pos = mcu_fw->size;
	mcu_fw_buf_ISP = kmalloc(mcu_fw_ISP->size+1, GFP_KERNEL);
	mcu_fw_buf_ISP[mcu_fw_ISP->size] = '\0';
	memcpy(mcu_fw_buf_ISP, mcu_fw_ISP->data, mcu_fw_ISP->size);
       	for (i=0;i<mcu_fw_ISP->size;i++)
	{
		mcu_fw_buf_ISP[i] ^= key[key_size++];
		key_size = (key_size == 16) ? 0 : key_size;
	}	

	return 0;
}
int toggle_mcu_reset_pin(struct i2c_client *client, struct isx031 *priv)
{

	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02CA , 0x40 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
	msleep(1);
	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02CA , 0x98 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
	msleep(1);
	return 0;	
}

int toggle_mcu_boot_pin(struct i2c_client *client, struct isx031 *priv)
{
	uint16_t rst_reg_addr = 0;
	uint8_t boot_conf_reg_val = 0;
	
	if(serdes_write_16b_reg(client, priv->ser_addr, 0x02BE , 0x20) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}

	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02BE , 0x98 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
	msleep(1);

	if(toggle_mcu_reset_pin(client, priv) <  0)
		return -EIO;
return 0;
}

static int mcu_get_fw_version(struct i2c_client *client,
	       	unsigned char *fw_version, unsigned char *txt_fw_version)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop, i=0, retry = 5;
	unsigned long txt_fw_pos = strlen(mcu_fw_buf)-VERSION_FILE_OFFSET;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);
	/* Get Text Firmware version*/
	for(loop = txt_fw_pos; loop < (txt_fw_pos+64); loop=loop+2) {
		*(txt_fw_version+i) = (mcu_bload_ascii2hex(mcu_fw_buf[loop]) << 4 |
				mcu_bload_ascii2hex(mcu_fw_buf[loop+1]));
		i++;
	}

	while (retry-- > 0) {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;

		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n",
				       	__func__,__LINE__, ret);
			ret = -EIO;
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		//msleep(100);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n",
				       	__func__,__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}

		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}

		/* Read the actual version from MCU*/
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev," %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		if(ret == ERRCODE_SUCCESS) 
			break; 
	}

	if (retry < 0 && ret != ERRCODE_SUCCESS) {
		pr_info(" %s with exit code = %d %d\n", __func__, ret,__LINE__);
		goto exit;
	}

	for (loop = 0 ; loop < VERSION_SIZE ; loop++ )
		*(fw_version+loop) = mc_ret_data[2+loop];

	/* Check for forced/always update field in the text firmware version*/
	if(txt_fw_version[17] == '1') {
		dev_err(&client->dev, "Forced Update Enabled - Firmware Version - (%.32s) \n",
				fw_version);
		ret = 2;
		goto exit;
	}			

	for(i = 0; i < VERSION_SIZE; i++) {

	
		if(txt_fw_version[i] != fw_version[i]) {
			dev_dbg(&client->dev, "Previous Firmware Version - (%.32s)\n", fw_version);
			dev_dbg(&client->dev, "Current Firmware Version - (%.32s)\n", txt_fw_version);
			ret = 1;
			goto exit;
		}
	}

	ret = ERRCODE_SUCCESS;

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int spi_get_fw_version(struct i2c_client *client,
	       	unsigned char *fw_version, unsigned char *txt_fw_version)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop, i=0, retry = 5;
	unsigned long txt_fw_pos = strlen(mcu_fw_buf)-VERSION_FILE_OFFSET_SPI;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);
	/* Get Text Firmware version*/
	for(loop = txt_fw_pos; loop < (txt_fw_pos+64); loop=loop+2) {
		*(txt_fw_version+i) = (mcu_bload_ascii2hex(mcu_fw_buf[loop]) << 4 |
				mcu_bload_ascii2hex(mcu_fw_buf[loop+1]));
		i++;
	}


	while (retry-- > 0) {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION_SPI;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION_SPI;

		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n",
				       	__func__,__LINE__, ret);
			ret = -EIO;
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);

		//msleep(100);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n",
				       	__func__,__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		/* Read the actual version from MCU*/
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev," %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}


		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		if(ret == ERRCODE_SUCCESS) 
			break; 
	}


	if (retry < 0 && ret != ERRCODE_SUCCESS) {
		pr_info(" %s with exit code = %d %d\n", __func__, ret,__LINE__);
		goto exit;
	}
	for (loop = 0 ; loop < VERSION_SIZE ; loop++ )
		*(fw_version+loop) = mc_ret_data[2+loop];

	ret = ERRCODE_SUCCESS;

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

int mcu_bload_go(struct i2c_client *client)
{
	int ret = 0;

	g_bload_buf[0] = BL_GO;
	g_bload_buf[1] = ~(BL_GO);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	/*   Start Address */
	g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf[4] =
		g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	return 0;
}

static int Sensor_flash_wp_enable(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	mutex_lock(&g_i2c_mutex);

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_FLASH_WP;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_FLASH_WP;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		goto exit_flash_en;
	}

	msleep(200);
	mutex_unlock(&g_i2c_mutex);
	return 0;

exit_flash_en:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;
}


static int Sensor_flash_wp_disable(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];


	mutex_lock(&g_i2c_mutex);

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_FLASH_WP_DISABLE;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_FLASH_WP_DISABLE;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		goto exit_flash_de;
	}

	msleep(200);
	mutex_unlock(&g_i2c_mutex);
	return 0;

exit_flash_de:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;
}

static int mcu_isp_init_ISP_ERASE_FLASH(struct i2c_client *client)

{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];
	int LoopIndex = 0;
	

	pr_info("ERASING SPI FLASH\n");


		payload_len = 0;
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_INIT_CAM_ISP_ERASE;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
	
		err = cam_write(client, mc_data, TX_LEN_PKT);
		msleep(2);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
					__LINE__, err);
			goto exit_isp_erase;
		}
	
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_INIT_CAM_ISP_ERASE;
		mc_data[2] = ( LoopIndex & 0xFF );
		
	
		err = cam_write(client, mc_data, 3);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
					__LINE__, err);
			goto exit_isp_erase;
		}
		msleep(18000);
	
	
			cmd_id = CMD_ID_INIT_CAM_ISP_ERASE;
			if (mcu_get_cmd_status
					(client, &cmd_id, &cmd_status, &retcode) < 0) {
				dev_err(&client->dev," %s(%d) Error \n",
						__func__, __LINE__);
				goto exit_isp_erase;
			}
	
			if ( retcode == ERRCODE_SUCCESS) {

				mutex_unlock(&g_i2c_mutex);
				return 0;
			}
	
			if ( retcode != ERRCODE_SUCCESS ) {
				dev_err(&client->dev, "(%s) %d Write Bootdata Error RET = 0x%02x\n",__func__, __LINE__, retcode);
				goto exit_isp_erase;
			}
	
exit_isp_erase:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;
}

static int mcu_isp_init_ISP(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];
	
	
	uint32_t isxBootDataSize = 512*1024;
	uint16_t transactionCount = ((uint32_t)isxBootDataSize/ISX_BUFFER_SIZE);
	int buffIndex = 0;
	
	msleep(100);

	pr_info("SENSOR BOOT DATA mcu_isp_init\n");

	mutex_lock(&g_i2c_mutex);

	for(buffIndex=0;transactionCount;--transactionCount,buffIndex++){

		payload_len = 256;
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_INIT_CAM_ISP;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
	
		cam_write(client, mc_data, TX_LEN_PKT);
	
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_INIT_CAM_ISP;
	

	/* ISX031 BOOT DATA WRITE IMPLEMENATTAION*/
		
		memcpy( &mc_data[2] , &mcu_fw_buf_ISP[ buffIndex*ISX_BUFFER_SIZE], 256);
		
		mc_data[258] = errorcheck(&mc_data[2], 256);
	
	
	/* -------------------------------------------------------- */

		err = cam_write(client, mc_data, 259);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
					__LINE__, err);
			goto exit_isp_write;
		}
		msleep(100);
	
		cmd_id = CMD_ID_INIT_CAM_ISP;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
					__func__, __LINE__);
			goto exit_isp_write;
		}
	
		if ( retcode == ERRCODE_SUCCESS) {
			continue;
		}

		if ( retcode != ERRCODE_SUCCESS ) {
			dev_err(&client->dev, "(%s) %d Write Bootdata Error RET = 0x%02x\n",__func__, __LINE__, retcode);
			goto exit_isp_write;
		}
	
	}
		mutex_unlock(&g_i2c_mutex);
		return 0;

exit_isp_write:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;

}


static int mcu_isp_init(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	pr_info("mcu_isp_init\n");

	mutex_lock(&g_i2c_mutex);

	/* call ISP init command */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		goto exit_isp_init;
		//return -EIO;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		msleep(400);

		cmd_id = CMD_ID_INIT_CAM;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
					__func__, __LINE__);
			goto exit_isp_init;
			//return -EIO;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				((retcode == ERRCODE_SUCCESS) || (retcode == ERRCODE_ALREADY))) {
			dev_err(&client->dev,"ISP Initialized !! \n");
			mutex_unlock(&g_i2c_mutex);
			return 0;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d Init Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			goto exit_isp_init;
			//return -EIO;
		}
	}
	dev_err(&client->dev,"ETIMEDOUT Error\n");
	return -ETIMEDOUT;

exit_isp_init:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;
}
unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len)
{
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		crc ^= buf[i];
	}

	return crc;
}
unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len)
{
	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		checksum = (checksum + buf[i]);
	}

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

int mcu_bload_get_version(struct i2c_client *client)
{
	int ret = 0;

	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf[0] = BL_GET_VERSION;
	g_bload_buf[1] = ~(BL_GET_VERSION);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed\n");
		return -1;
	}

	/* ---------------- GET VERSION END ------------------- */

	return 0;
}
int mcu_bload_erase_flash(struct i2c_client *client)
{
	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;

	/* --------------- ERASE FLASH --------------------- */

	for (i = 0; i < NUM_ERASE_CYCLES; i++) {

		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_ERASE_MEM_NS;
		g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
		g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
		g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

		ret = cam_write(client, g_bload_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
			g_bload_buf[(2 * pagenum)] =
				(pagenum + (i * MAX_PAGES)) >> 8;
			g_bload_buf[(2 * pagenum) + 1] =
				(pagenum + (i * MAX_PAGES)) & 0xFF;
			checksum =
				checksum ^ g_bload_buf[(2 * pagenum)] ^
				g_bload_buf[(2 * pagenum) + 1];
		}
		g_bload_buf[2 * MAX_PAGES] = checksum;

		ret = cam_write(client, g_bload_buf, (2 * MAX_PAGES) + 1);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		debug_printk(" ERASE Sector %d success !! \n", i + 1);
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}
int mcu_bload_parse_send_cmd(struct i2c_client *client,
		unsigned char *bytearray, int rec_len)
{
	IHEX_RECORD *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (IHEX_RECORD *) bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = mcu_bload_inv_checksum(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		dev_err(&client->dev," Invalid Checksum 0x%02x != 0x%02x !! \n",
				checksum, calc_checksum);
		return -1;
	}

	if ((ihex_rec->rectype == REC_TYPE_ELA)
			&& (ihex_rec->addr == 0x0000)
			&& (ihex_rec->datasize = 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr = (ihex_rec->recdata[0] <<
				24) | (ihex_rec->recdata[1]
					<< 16);

		debug_printk("Updated Flash Addr = 0x%08x \n",
				g_bload_flashaddr);

	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr =
			(g_bload_flashaddr & 0xFFFF0000) | (ihex_rec->addr);
		g_bload_crc16 ^=
			mcu_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_WRITE_MEM_NS;
		g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
		g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
		g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
		g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
		g_bload_buf[4] =
			g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
			g_bload_buf[3];

		ret = cam_write(client, g_bload_buf, 5);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf[i + 1];
		}

		g_bload_buf[i + 1] = checksum;

		ret = cam_write(client, g_bload_buf, i + 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
		return 0;
	} else {

		/*   Unhandled Type */
		dev_err(&client->dev,"Unhandled Command Type \n");
		return -1;
	}

	return 0;
}

int mcu_bload_update_fw(struct i2c_client *client)
{
	unsigned long hex_file_size = strlen(mcu_fw_buf); //- 1;
	unsigned char wbuf[MAX_BUF_LEN];
	int i = 0, recindex = 0, ret = 0;

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (mcu_fw_buf[i] == ':')) {
			/*  debug_printk("Start of a Record \n"); */
		} else if (mcu_fw_buf[i] == CR) {
			/*   No Implementation */
		} else if (mcu_fw_buf[i] == LF) {
			if (recindex == 0) {
				/*   Parsing Complete */
				break;
			}

			/*   Analyze Packet and Send Commands */
			ret = mcu_bload_parse_send_cmd(client, wbuf, recindex);
			if (ret < 0) {
				dev_err(&client->dev,"Error in Processing Commands \n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			if ((ret = mcu_bload_ascii2hex(mcu_fw_buf[i])) < 0) {
				dev_err(&client->dev,
						"Invalid Character - 0x%02x !! \n",
						mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			if ((ret = mcu_bload_ascii2hex(mcu_fw_buf[i])) < 0) {
				dev_err(&client->dev,
						"Invalid Character - 0x%02x !!!! \n",
						mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}

	debug_printk("Program FLASH Success !! - CRC = 0x%04x \n",
			g_bload_crc16);

	/* ------------ PROGRAM FLASH END ----------------------- */

	return ret;
}

int mcu_bload_read(struct i2c_client *client,
		unsigned int g_bload_flashaddr, char *bytearray,
		unsigned int len)
{
	int ret = 0;

	g_bload_buf[0] = BL_READ_MEM;
	g_bload_buf[1] = ~(BL_READ_MEM);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] =
		g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = len - 1;
	g_bload_buf[1] = ~(len - 1);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = cam_read(client, bytearray, len);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	return 0;
}

int mcu_bload_verify_flash(struct i2c_client *client,
		unsigned short int orig_crc)
{
	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
				(client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
				(client, flash_addr + i, bytearray, (FLASH_SIZE - i))
				< 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
		dev_err(&client->dev," CRC verification fail !! 0x%04x != 0x%04x \n",
				orig_crc, calc_crc);
		return -1;
	}

	debug_printk(" CRC Verification Success 0x%04x == 0x%04x \n",
			orig_crc, calc_crc);

	return 0;
}

static int mcu_fw_update(struct i2c_client *client, unsigned char *mcu_fw_version)
{
	int ret = 0;
	g_bload_crc16 = 0;

	/* Read Firmware version from bootloader MCU */
	ret = mcu_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Get Version \n");
		goto exit;
	}

	debug_printk(" Get Version SUCCESS !! \n");

	/* Erase firmware present in the MCU and flash new firmware*/
	ret = mcu_bload_erase_flash(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Erase Flash \n");
		goto exit;
	}

	debug_printk("Erase Flash Success !! \n");

	/* Read the firmware present in the text file */
	if ((ret = mcu_bload_update_fw(client)) < 0) {
		dev_err(&client->dev," Write Flash FAIL !! \n");
		goto exit;
	}

	/* Verify the checksum for the update firmware */
	if ((ret = mcu_bload_verify_flash(client, g_bload_crc16)) < 0) {
		dev_err(&client->dev," verify_flash FAIL !! \n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	/* I2C GO Command */
	if ((ret = mcu_bload_go(client)) < 0) {
		dev_err(&client->dev," i2c_bload_go FAIL !! \n");
		goto exit;
	}

	if(mcu_fw_version) {
		dev_dbg(&client->dev, "(%s) - Firmware Updated - (%.32s)\n",
				__func__, mcu_fw_version);
	}
exit:
	return ret;
}

int check_ecam_mcu_fw_status(struct i2c_client *client, struct isx031 *priv)
{
	uint8_t ret, loop, retry, err;
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};
	uint16_t boot_reg_addr = 0;

	ret = mcu_get_fw_version(client, fw_version, txt_fw_version);
	if (ret != 0) {

		if(ret > 0) {
			if((err = mcu_jump_bload(client)) < 0) {
				dev_err(&client->dev," Cannot go into bootloader mode\n");
				return -EIO;
			}			
			msleep(1000);
		} else {
			dev_info(&client->dev,"Using Boot pin for firmware update\n");

			retry = 10;
			while(retry -- > 0) {		
				if(toggle_mcu_boot_pin(client, priv) < 0) {
					msleep(100);
					dev_info(&client->dev,"Retry Boot pin toggle \n");
					continue;
				}
				break;
			}
			if(retry <= 0) {
				dev_err(&client->dev," Cannot go into bootloader mode\n");
				return -EIO;
			}

		}
		dev_err(&client->dev," Trying to Detect Bootloader mode\n");

		for(loop = 0;loop < 10; loop++) {
			err = mcu_bload_get_version(client);
			if (err != 0) {
				/* Trial and Error for 1 second (100ms * 10) */
				if(toggle_mcu_boot_pin(client, priv) < 0) {
					msleep(1000);
					dev_info(&client->dev,"Retry Boot pin toggle \n");
				}
				continue;
			} else {
				dev_err(&client->dev," Get Bload Version Success\n");
				break;
			}
		}

		if(loop == 10) {
			dev_err(&client->dev, "Error updating firmware \n");
			return -EINVAL;
		}				

		for( loop = 0; loop < 10; loop++) {
			err = mcu_fw_update(client, NULL);
			if(err < 0) {
				dev_err(&client->dev,
						"%s(%d) Error updating firmware.. Retry.. \n",
						__func__, __LINE__);

				continue;
			} else {
				dev_err (&client->dev, "Firmware Updated Successfully\n");
				break;	
			}

		}
		if( loop == 10) {
			dev_err( &client->dev, "Error Updating Firmware\n");
			return -EFAULT;
		}
	
		boot_reg_addr = MCU_BOOT_REG;
		
		if((serdes_write_16b_reg(client, priv->ser_addr,
					       	boot_reg_addr, RST_SER_GPIO)) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",__func__, __LINE__);
			return -EIO;
		}

		/* Allow FW Updated Driver to reboot */
		msleep(3000);
		/*Maintaining GMSL1 firmware update compatability*/
		for(loop = 0;loop < 10; loop++) {
			err = mcu_get_fw_version(client, fw_version, txt_fw_version);
			if (err < 0) {
				msleep(1000);

				/* See if it is a empty MCU */
				err = mcu_bload_get_version(client);
				if (err < 0) {
					dev_err(&client->dev," Get Bload Version Fail\n");
				} else {
					dev_err(&client->dev," Get Bload Version Success\n");

					/* Re-issue GO command to get into user mode */
					if (mcu_bload_go(client) < 0) {
						dev_err(&client->dev," i2c_bload_go FAIL !! \n");
					}					
					msleep(1000);
				}						

				continue;
			} else {
				dev_err(&client->dev," Get FW Version Success\n");
				break;
			}
		}
		if(loop == 10) {
			dev_err(&client->dev, "Error updating firmware \n");
			return -EINVAL;
		}						

		debug_printk("Current Firmware Version - (%.32s).",
				fw_version);

	} else {
		/* Same firmware version in MCU and Text File */
		debug_printk("Current Firmware Version - (%.32s)",fw_version);
	}
	return 0;
}

static int mcu_stream_config(struct i2c_client *client, uint32_t format,
		int mode, int frate_index)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);
	for (loop = 0;(&priv->streamdb[loop]) != NULL; loop++) {
		if (priv->streamdb[loop] == mode) {
			index = loop + frate_index;
			break;
		}
	}

	debug_printk(" Index = 0x%04x , format = 0x%08x, width = %hu,"
			" height = %hu, frate num = %hu \n", index, format,
			priv->mcu_cam_frmfmt[mode].size.width,
			priv->mcu_cam_frmfmt[mode].size.height,
			priv->mcu_cam_frmfmt[mode].framerates[frate_index]);

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if(priv->prev_index == index) {
		debug_printk("Not Skipping Previous mode set ... \n");
		//ret = 0;
		//goto exit;
	}


issue_cmd:
	/* First Txn Payload length = 0 */
	payload_len = 14;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Format Fourcc - currently only UYVY */
	mc_data[4] = format >> 24;
	mc_data[5] = format >> 16;
	mc_data[6] = format >> 8;
	mc_data[7] = format & 0xFF;

	/* width */
	mc_data[8] = priv->mcu_cam_frmfmt[mode].size.width >> 8;
	mc_data[9] = priv->mcu_cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data[10] = priv->mcu_cam_frmfmt[mode].size.height >> 8;
	mc_data[11] = priv->mcu_cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data[12] = priv->mcu_cam_frmfmt[mode].framerates[frate_index] >> 8;
	mc_data[13] = priv->mcu_cam_frmfmt[mode].framerates[frate_index] & 0xFF;

	/* frame rate denom */
	mc_data[14] = 0x00;
	mc_data[15] = 0x01;

	mc_data[16] = errorcheck(&mc_data[2], 14);
	err = cam_write(client, mc_data, 17);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (--retry > 0) {
		cmd_id = CMD_ID_STREAM_CONFIG;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev,
					" %s(%d) MCU GET CMD Status Error : loop : %d \n",
					__func__, __LINE__, loop);
			ret = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if(retcode == ERRCODE_AGAIN) {
			/* Issue Command Again if Set */
			retry = 1000;
			goto issue_cmd;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}

		/* Delay after retry */
		mdelay(10);
	}

	dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
			__LINE__, err);
	ret = -ETIMEDOUT;

exit:
	if(!ret)
		priv->prev_index = index;

	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;
}

static int mcu_list_ctrls(struct i2c_client *client,
		ISP_CTRL_INFO * mcu_cam_ctrl, struct isx031 *priv)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0,retry = 100;
	uint8_t mc_data[1024], mc_ret_data[1024];

	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);

	/* Array of Ctrl Info */
	while (retry-- > 0) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			continue;
		}
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			priv->num_ctrls = index;
			break;
		}
	
		//printk("priv->num_ctrls: %d \n\r ", priv->num_ctrls);

		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) Errcode - 0x%02x \n",
					__func__, __LINE__, errcode);
			continue;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
			errorcheck(&mc_ret_data[2],
					payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) Errcode - 0x%02x \n",
					__func__, __LINE__, errcode);
			continue;
		}

		if(mcu_cam_ctrl != NULL) {

			/* append ctrl info in array */
			mcu_cam_ctrl[index].ctrl_id =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			mcu_cam_ctrl[index].ctrl_type = mc_ret_data[6];

			switch (mcu_cam_ctrl[index].ctrl_type) {
				case CTRL_STANDARD:
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min =
						mc_ret_data[7] << 24 | mc_ret_data[8] << 16
						| mc_ret_data[9] << 8 | mc_ret_data[10];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max =
						mc_ret_data[11] << 24 | mc_ret_data[12] <<
						16 | mc_ret_data[13]
						<< 8 | mc_ret_data[14];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def =
						mc_ret_data[15] << 24 | mc_ret_data[16] <<
						16 | mc_ret_data[17]
						<< 8 | mc_ret_data[18];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step =
						mc_ret_data[19] << 24 | mc_ret_data[20] <<
						16 | mc_ret_data[21]
						<< 8 | mc_ret_data[22];
					break;

				case CTRL_EXTENDED:
					/* Not Implemented */
					break;
			}

			priv->ctrldb[index] = mcu_cam_ctrl[index].ctrl_id;
		}
		index++;
		if(retry == 0) {
			ret = -EIO;
			goto exit;
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;

}

static int mcu_get_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
		uint8_t * ctrl_type, int32_t * curr_val)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;

	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0xFFFF;
	int loop = 0, ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	uint32_t ctrl_id = 0;

	dev_err(&client->dev," %s(%d)\n", __func__,__LINE__);
	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);

	ctrl_id = arg_ctrl_id;

	/* Read the Ctrl Value from Micro controller */

	for (loop = 0; loop < priv->num_ctrls; loop++) {
		if (priv->ctrldb[loop] == ctrl_id) {
			index = loop;//priv->mcu_ctrl_info[loop].mcu_ctrl_index;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if (
			priv->mcu_ctrl_info[loop].ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags &
			V4L2_CTRL_FLAG_WRITE_ONLY
	   ) {
		ret = -EACCES;
		goto exit;
	}

	/* First Txn Payload length = 2 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -1;
		goto exit;
	}

	if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
		ret = -EIO;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
		errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	/* Ctrl type starts from index 6 */

	*ctrl_type = mc_ret_data[6];

	switch (*ctrl_type) {
		case CTRL_STANDARD:
			*curr_val =
				mc_ret_data[7] << 24 | mc_ret_data[8] << 16 | mc_ret_data[9]
				<< 8 | mc_ret_data[10];
			break;

		case CTRL_EXTENDED:
			/* Not Implemented */
			break;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;
}

static int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
		uint8_t ctrl_type, int32_t curr_val)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	uint8_t mc_data[512], mc_ret_data[512];

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);

	ctrl_id = arg_ctrl_id;
/*
	
	if(arg_ctrl_id == V4L2_CID_PWM_CONFIG)
	{
		//printk("PWM_config \n\r");
		if (set_pwm_frequency(curr_val, pwm_period, pwm_pulse) != 0)
		{
			printk("err\n\r");
		}
		printk(" PWM frequency control SUCCESS \n\r");
		goto exit;
	}
*/

	/* call ISP Ctrl config command */

	for (loop = 0; loop < priv->num_ctrls; loop++) {
		if (priv->ctrldb[loop] == ctrl_id) {
			index = loop;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	/* First Txn Payload length = 0 */
	payload_len = 11;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Control ID */
	mc_data[4] = ctrl_id >> 24;
	mc_data[5] = ctrl_id >> 16;
	mc_data[6] = ctrl_id >> 8;
	mc_data[7] = ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data[8] = ctrl_type;

	/* Ctrl Value */
	mc_data[9] = curr_val >> 24;
	mc_data[10] = curr_val >> 16;
	mc_data[11] = curr_val >> 8;
	mc_data[12] = curr_val & 0xFF;

	/* CRC */
	mc_data[13] = errorcheck(&mc_data[2], 11);

	err = cam_write(client, mc_data, 14);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (retry-- > 0) {
		cmd_id = CMD_ID_SET_CTRL;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			pr_err
				("(%s) %d ISP Error STATUS = 0x%04x RET = 0x%02x\n",
				 __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}
		msleep(10);
	}
	if(retry <= 0)
	{
		pr_err("(%s) %d Error setting control = 0x%04x RET = 0x%02x\n",
			 __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;

	}
exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;
}

static int cam_try_add_ctrls(struct isx031 *priv, int index,
		ISP_CTRL_INFO * mcu_ctrl)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl_config custom_ctrl_config;
	if (!priv || !priv->pdata)
		return -EINVAL;
	
	if(	mcu_ctrl->ctrl_id != V4L2_CID_BRIGHTNESS &&
		mcu_ctrl->ctrl_id != V4L2_CID_CONTRAST && 
		mcu_ctrl->ctrl_id != V4L2_CID_SATURATION && 
		mcu_ctrl->ctrl_id != V4L2_CID_SHARPNESS &&  
		mcu_ctrl->ctrl_id != V4L2_CID_FRAME_SYNC && 
		mcu_ctrl->ctrl_id != V4L2_CID_HUE && 
		mcu_ctrl->ctrl_id != V4L2_CID_EXPOSURE_AUTO &&
		mcu_ctrl->ctrl_id != V4L2_CID_EXPOSURE_ABSOLUTE ){
		return 0;
	}
	
	printk("mcu_ctrl->ctrl_id : %08x\n\r", mcu_ctrl->ctrl_id);
	priv->ctrl_handler.error = 0;
	/* Try Enumerating in standard controls */
	priv->ctrls[index] =
		v4l2_ctrl_new_std(&priv->ctrl_handler,
				&cam_ctrl_ops,
				mcu_ctrl->ctrl_id,
				mcu_ctrl->ctrl_data.std.ctrl_min,
				mcu_ctrl->ctrl_data.std.ctrl_max,
				mcu_ctrl->ctrl_data.std.ctrl_step,
				mcu_ctrl->ctrl_data.std.ctrl_def);
	if (priv->ctrls[index] != NULL) {
		debug_printk("%d. Initialized Control 0x%08x - %s \n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}

		if(mcu_ctrl->ctrl_id == V4L2_CID_PWM_CONFIG)
		goto custom;
	/*
 *
	if(mcu_ctrl->ctrl_id == V4L2_CID_EXPOSURE_AUTO)
		goto custom;
*/

	/* Try Enumerating in standard menu */
	priv->ctrl_handler.error = 0;
	priv->ctrls[index] =
		v4l2_ctrl_new_std_menu(&priv->ctrl_handler,
				&cam_ctrl_ops,
				mcu_ctrl->ctrl_id,
				mcu_ctrl->ctrl_data.std.ctrl_max,
				0, mcu_ctrl->ctrl_data.std.ctrl_def);
	if (priv->ctrls[index] != NULL) {
		debug_printk("%d. Initialized Control Menu 0x%08x - %s \n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}


custom:
	priv->ctrl_handler.error = 0;
	memset(&custom_ctrl_config, 0x0, sizeof(struct v4l2_ctrl_config));

	if (mcu_get_ctrl_ui(client, mcu_ctrl, index)!= ERRCODE_SUCCESS) {
		dev_err(&client->dev, "Error Enumerating Control 0x%08x !! \n",
				mcu_ctrl->ctrl_id);
		return -EIO;
	}

	/* Fill in Values for Custom Ctrls */
	custom_ctrl_config.ops = &cam_ctrl_ops;
	custom_ctrl_config.id = mcu_ctrl->ctrl_id;
	/* Do not change the name field for the control */
	custom_ctrl_config.name = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_name;

	/* Sample Control Type and Flags */
	custom_ctrl_config.type = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type;
	custom_ctrl_config.flags = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags;

	custom_ctrl_config.min = mcu_ctrl->ctrl_data.std.ctrl_min;
	custom_ctrl_config.max = mcu_ctrl->ctrl_data.std.ctrl_max;
	custom_ctrl_config.step = mcu_ctrl->ctrl_data.std.ctrl_step;
	custom_ctrl_config.def = mcu_ctrl->ctrl_data.std.ctrl_def;

	if (custom_ctrl_config.type == V4L2_CTRL_TYPE_MENU) {
		custom_ctrl_config.step = 0;
		custom_ctrl_config.type_ops = NULL;

		custom_ctrl_config.qmenu =
			(const char *const *)(mcu_ctrl->ctrl_ui_data.ctrl_menu_info.menu);
	}

	priv->ctrls[index] =
		v4l2_ctrl_new_custom(&priv->ctrl_handler,
				&custom_ctrl_config, NULL);
	if (priv->ctrls[index] != NULL) {
		debug_printk("%d. Initialized Custom Ctrl 0x%08x - %s \n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}

	dev_err(&client->dev,
			"%d.  default: Failed to init 0x%08x ctrl Error - %d \n",
			index, mcu_ctrl->ctrl_id, priv->ctrl_handler.error);
	return -EINVAL;
}

static int cam_ctrls_init(struct isx031 *priv, ISP_CTRL_INFO *mcu_cam_ctrls)
{
	struct i2c_client *client = priv->i2c_client;
	int err = 0, i = 0;

	/* Array of Ctrls */

	/* Custom Ctrl */
	if (!priv || !priv->pdata)
		return -EINVAL;

	if (mcu_list_ctrls(client, mcu_cam_ctrls, priv) < 0) {
		dev_err(&client->dev, "Failed to init ctrls\n");
		goto error;
	}

	v4l2_ctrl_handler_init(&priv->ctrl_handler, priv->num_ctrls+1);
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	for (i = 0; i < priv->num_ctrls; i++) {

		if (mcu_cam_ctrls[i].ctrl_type == CTRL_STANDARD) {
			cam_try_add_ctrls(priv, i,
					&mcu_cam_ctrls[i]);
		} else {
			/* Not Implemented */
		}
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int mcu_list_fmts(struct i2c_client *client,
	       	ISP_STREAM_INFO *stream_info, int *frm_fmt_size,struct isx031 *priv)
{
	uint32_t payload_len = 0, err = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0, skip = 0;
	uint16_t index = 0, mode = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	int loop = 0, num_frates = 0, ret = 0;

	/* Stream Info Variables */

	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);
	/* List all formats from MCU and append to mcu_cam_frmfmt array */
	for (index = 0;; index++) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			pr_err
				(" %s(%d) CRC 0x%02x != 0x%02x \n",
				 __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if(stream_info == NULL) {
				*frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
			}
			break;
		}

		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			pr_err
				(" %s(%d) Errcode - 0x%02x \n",
				 __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
					__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
			errorcheck(&mc_ret_data[2],
					payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			pr_err
				(" %s(%d) CRC 0x%02x != 0x%02x \n",
				 __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			pr_err
				(" %s(%d) Errcode - 0x%02x \n",
				 __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		if(stream_info != NULL) {
			/* check if any other format than UYVY is queried - do not append in array */
			stream_info->fmt_fourcc =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			stream_info->width = mc_ret_data[6] << 8 | mc_ret_data[7];
			stream_info->height = mc_ret_data[8] << 8 | mc_ret_data[9];
			stream_info->frame_rate_type = mc_ret_data[10];

			switch (stream_info->frame_rate_type) {
				case FRAME_RATE_DISCRETE:
					stream_info->frame_rate.disc.frame_rate_num =
						mc_ret_data[11] << 8 | mc_ret_data[12];

					stream_info->frame_rate.disc.frame_rate_denom =
						mc_ret_data[13] << 8 | mc_ret_data[14];

					break;

				case FRAME_RATE_CONTINOUS:
					debug_printk
						(" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,"
						 "which is unsupported !! \n", index);

					continue;

			}

			switch (stream_info->fmt_fourcc) {
				case V4L2_PIX_FMT_UYVY:
					/* cam_codes is already populated with V4L2_MBUS_FMT_UYVY8_1X16 */
					/* check if width and height are already in array - update frame rate only */
					for (loop = 0; loop < (mode); loop++) {
						if ((priv->mcu_cam_frmfmt[loop].size.width ==
									stream_info->width)
								&& (priv->mcu_cam_frmfmt[loop].size.height ==
									stream_info->height)) {

							num_frates =
								priv->mcu_cam_frmfmt
								[loop].num_framerates;
							*((int *)(priv->mcu_cam_frmfmt[loop].framerates) + num_frates)
								= (int)(stream_info->frame_rate.
										disc.frame_rate_num /
										stream_info->frame_rate.
										disc.frame_rate_denom);

							priv->mcu_cam_frmfmt
								[loop].num_framerates++;

							priv->streamdb[index] = loop;
							skip = 1;
							break;
						}
					}

					if (skip) {
						skip = 0;
						continue;
					}

					/* Add Width, Height, Frame Rate array, Mode into mcu_cam_frmfmt array */
					priv->mcu_cam_frmfmt[mode].size.width = stream_info->width;
					priv->mcu_cam_frmfmt[mode].size.height =
						stream_info->height;
					num_frates = priv->mcu_cam_frmfmt[mode].num_framerates;

					*((int *)(priv->mcu_cam_frmfmt[mode].framerates) + num_frates) =
						(int)(stream_info->frame_rate.disc.frame_rate_num /
								stream_info->frame_rate.disc.frame_rate_denom);

					priv->mcu_cam_frmfmt[mode].num_framerates++;

					priv->mcu_cam_frmfmt[mode].mode = mode;
					priv->streamdb[index] = mode;
					mode++;
					break;

				default:
					debug_printk
						(" The Stream format at index 0x%04x has format 0x%08x ,"
						 "which is unsupported !! \n", index,
						 stream_info->fmt_fourcc);
			}
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;
}

static int mcu_get_ctrl_ui(struct i2c_client *client,
		ISP_CTRL_INFO * mcu_ui_info, int index)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, i = 0, err = 0;
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	uint8_t mc_data[1024], mc_ret_data[1024];

	/* lock semaphore */
	mutex_lock(&priv->mcu_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
		errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_name, &mc_ret_data[2],MAX_CTRL_UI_STRING_LEN);

	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type = mc_ret_data[34];
	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags = mc_ret_data[35] << 8 |
		mc_ret_data[36];

	if (mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type == V4L2_CTRL_TYPE_MENU) {
		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem = mc_ret_data[37];

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu =
			devm_kzalloc(&client->dev,((mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem +1) * sizeof(char *)), GFP_KERNEL);
		for (i = 0; i < mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem; i++) {
			mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] =
				devm_kzalloc(&client->dev,MAX_CTRL_UI_STRING_LEN, GFP_KERNEL);
			strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i],
					&mc_ret_data[38 +(i *MAX_CTRL_UI_STRING_LEN)], MAX_CTRL_UI_STRING_LEN);

			debug_printk(" Menu Element %d : %s \n",
					i, mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i]);
		}

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] = NULL;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mcu_i2c_mutex);

	return ret;

}

static int mcu_get_cmd_status(struct i2c_client *client,
		uint8_t * cmd_id, uint16_t * cmd_status,
		uint8_t * ret_code)
{
	uint32_t payload_len = 0;
	uint8_t orig_crc = 0, calc_crc = 0;
	int err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* No Semaphore in Get command Status */

	/* First Txn Payload length = 0 */
	payload_len = 1;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = *cmd_id;
	err = cam_write(client, mc_data, 3);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		return -EIO;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		return -EIO;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 3);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		return -EINVAL;
	}

	*cmd_id = mc_ret_data[2];
	*cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
	*ret_code = mc_ret_data[payload_len - 1];

	return 0;
}

static int mcu_get_flash_connect_status(struct i2c_client *client, uint8_t * fl_status)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FL_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FL_STATUS;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 1];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*fl_status = mc_ret_data[2];

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int mcu_get_load_param_status(struct i2c_client *client, uint8_t * lp_status)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_LOAD_PARAM_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	mdelay(50);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_LOAD_PARAM_STATUS;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 1];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*lp_status = mc_ret_data[2];

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int mcu_get_nv_boot_status(struct i2c_client *client, uint8_t * nv_status)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_NV_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	mdelay(50);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_NV_STATUS;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 1];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*nv_status = mc_ret_data[2] ;

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	mdelay(50);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*sensor_id = mc_ret_data[2] << 8 | mc_ret_data[3];

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

int ecam_mcu_core_initialize(struct i2c_client *client, struct isx031 *priv)
{
	uint16_t retry = 0;
	uint16_t sensor_id = 0;
	uint8_t fl_cn_status = 0x00;
	uint8_t lp_pa_status = 0x00;
	uint8_t nv_bt_status = 0x00;

	retry = 0;
	while(retry++ < 5){
	if (mcu_get_sensor_id(client, &sensor_id) < 0) {
		dev_err(&client->dev, "Unable to get Sensor ID!.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get Sensor ID!.. Exiting..\n");
		return -EIO;
	}
	dev_info(&client->dev,"Sensor ID = 0x%x\n",sensor_id);


	/* Issue retry for init ISP */
	retry = 10;
	while(retry -- > 0) {
		if (mcu_isp_init(client) < 0) {
			dev_err(&client->dev,
				       	"Unable to INIT ISP, retry = %d \n", retry);
			continue;
		} else {
			break;
		}
	}

	if(retry == 0) {
		dev_err(&client->dev, "Unable to INIT ISP \n");
		return -EFAULT;
	}
	
	retry = 0;
	while(retry++ < 5){
	if (mcu_get_flash_connect_status(client, &fl_cn_status) < 0) {
		dev_err(&client->dev, "Unable to get NV Boot STATUS!.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get NV Boot STATUS!.. Exiting..\n");
		return -EIO;
	}
	dev_info(&client->dev,"Fl Connect Status = 0x%02x\n",fl_cn_status);



	retry = 0;
	while(retry++ < 5){
	if (mcu_get_load_param_status(client, &lp_pa_status) < 0) {
		dev_err(&client->dev, "Unable to get Load Param STATUS!.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get Load Param STATUS!.. Exiting..\n");
		return -EIO;
	}
	dev_info(&client->dev,"Load Param Status = 0x%02x\n",lp_pa_status);


	retry = 0;
	while(retry++ < 5){
	if (mcu_get_nv_boot_status(client, &nv_bt_status) < 0) {
		dev_err(&client->dev, "Unable to get NV Boot STATUS!.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get NV Boot STATUS!.. Exiting..\n");
		return -EIO;
	}
	dev_info(&client->dev,"NV BOOT  Status = 0x%02x\n",nv_bt_status);
	
	return 0;
}

int ecam_serdes_config(struct i2c_client *client, struct isx031 *priv)
{

	/* Configuring SIOA Serializer */
	if(priv->phy == PHY_A)
	{
		if(serdes_parse_regdata(client, SER1_CONF,
					ARRAY_SIZE(SER1_CONF),priv->ser_addr) < 0) {
			dev_err(&client->dev, 
					"%s: Failed to configure SIOA Ser\n",__func__);
			return -EIO;
		}
		debug_printk("configuring SIOA serializer successful\n");
	}

	/* Configuring SIOB Serializer */
	if(priv->phy == PHY_B)
	{
		if(serdes_parse_regdata(client, SER2_CONF,
					ARRAY_SIZE(SER2_CONF),priv->ser_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOB Ser\n",__func__);
			return -EIO;
		}
		debug_printk("configuring SIOB serializer successful\n");
	}

	/* Configuring Deserializer */
	if(serdes_parse_regdata(client, DSER_CONF,
				ARRAY_SIZE(DSER_CONF),priv->des_addr) < 0) {
		dev_err(&client->dev, "%s: Failed to configure DESER \n",__func__);
		return -EIO;
	}
	debug_printk("configuring Deserializer Successful\n");
	return 0;
}

int clear=0;

//int Gtrigger_gpio;
//EXPORT_SYMBOL(Gtrigger_gpio);

static int isx031_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct camera_common_data *common_data;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct isx031 *priv;
	int err;
	int ret = 0, frm_fmt_size = 0, loop = 0, retry = 0;
	int i = 0;
	uint8_t fl_cn_status = 0x00;
	uint8_t lp_pa_status = 0x00;
	uint8_t nv_bt_status = 0x00;
	uint8_t boot_variable = 0x00;
	uint8_t spi_flash_bootdata_changed = 0x00;
	const char *str_trig;
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};
	int deser_enable = 0;

	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	deser_enable = of_get_named_gpio(node, "deser-reset-gpio", 0);
	if(deser_enable > 0) {
		debug_printk("deser_enable = %d \n", deser_enable);		
		err = gpio_request(deser_enable,"deser-en");
		if (err < 0) {
			dev_err(&client->dev,"%s[%d]:Deser GPIO Fail, err:%d",__func__,__LINE__, err);
			goto skip_deser;
		}		
		toggle_gpio(deser_enable, 1);
		msleep(500);		
		serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x80);
		msleep(200);
	}

skip_deser:
	common_data =
		devm_kzalloc(dev,
				sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data)
		return -ENOMEM;

	priv = devm_kzalloc(dev, sizeof(struct isx031) + sizeof(struct v4l2_ctrl *) * 15 , GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	
	priv->pdata = cam_parse_dt(client);
	if (!priv->pdata) {
		dev_err(dev, "unable to get platform data\n");
		return -EFAULT;
	}

	priv->pwm_dev = devm_pwm_get(&client->dev, NULL);
	if (IS_ERR(priv->pwm_dev)) {
		err = PTR_ERR(priv->pwm_dev);
		if (err != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Unable to request PWM for trigger\n");
		priv->got_pwm_handle = false;
		goto skip_trigger;
	} else {
		priv->got_pwm_handle = true;
		dev_info(&client->dev, "Got PWM for trigger\n");
	}

skip_trigger:
	err = ecam_specific_dt_parse(client, priv);
       	if(err < 0){
		dev_err(dev, "Error in ecam_specific_dt_parse\n");
		return -EFAULT;
	}


	if(!is_fw_loaded){
		if(ecam_mcu_firmware_load(client) < 0){
			dev_err(&client->dev,"failed to load mcu firmware\n");
				return -ENOENT;
		}
		else{
			printk("Firmware Successfully read by driver\n");
			is_fw_loaded = 1;
		}
	}

	priv->des_addr = DES_ADDR;
	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	common_data->priv = (void *)priv;

	err = isx031_power_get(priv);
	if (err)
		return err;

	err = isx031_power_on(common_data);
	if (err)
		return err;

	err = serdes_config_init(client, priv);
	if(err < 0)
	{
		dev_err(&client->dev,"serdes_config_init_failed\n");
		err = -EIO;
		goto error;
	}

	msleep(10);
	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02BE , 0x40 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}

	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02CA , 0x40 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}
	msleep(10);
	if(serdes_write_16b_reg(client, priv->ser_addr,	0x02CA , 0x98 ) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}
	msleep(10);

	err = spi_get_fw_version(client, fw_version, txt_fw_version);
	if( err != ERRCODE_SUCCESS ){
		printk("Unable to get Boot-Data Version\n");
	}

	if(!strcmp(mcu_fw_name_ISP, "ISX031_066F.bin")){

		printk("BOOT-DATA FILE NAME = %s\n",mcu_fw_name_ISP);
	}

	else if(!strcmp(mcu_fw_name_ISP, "ISX031_054F.bin")){

		printk("BOOT-DATA FILE NAME = %s\n",mcu_fw_name_ISP);
	}

	else if(!strcmp(mcu_fw_name_ISP, "ISX031_0128F.bin")){

		printk("BOOT-DATA FILE NAME = %s\n",mcu_fw_name_ISP);
	}


	for(i = 0; i < VERSION_SIZE; i++) {
		if(fw_version[i] != txt_fw_version[i]) {
			spi_flash_bootdata_changed = 0x01;
			break;

		}
		else if ( i == 31 ){
			printk("Boot-Data Version Matched - (%.32s)\n",txt_fw_version);
			spi_flash_bootdata_changed = 0x00;
		}

	}


	if( spi_flash_bootdata_changed == 0x01 ){

		printk("Previous Boot-Data Version - (%.32s)\n", fw_version);
		printk("Current Boot-Data Version  - (%.32s)\n", txt_fw_version);

	}

	ret = check_ecam_mcu_fw_status(client, priv);
	if(ret < 0){
		dev_err (dev, "%s(%d):ecam_mcu_fw_status Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}

	if(!is_fw_loaded_ISP){
		if(ecam_mcu_firmware_load_ISP(client) < 0){
			dev_err(&client->dev,"failed to load mcu firmware\n");
			err = -ENOENT;
			goto error;
		}
		else{
			printk("Firmware Successfully read by driver\n");
			is_fw_loaded_ISP = 1;
		}
	}

	mutex_init(&priv->mcu_i2c_mutex);

	/* Query the number of controls from MCU*/
	if(mcu_list_ctrls(client, NULL, priv) < 0) {
		dev_err(&client->dev, "%s, Failed to init controls \n", __func__);
		err = -EFAULT;
		goto error;
	}

	/*Query the number for Formats available from MCU */
	if(mcu_list_fmts(client, NULL, &frm_fmt_size,priv) < 0) {
		dev_err(&client->dev, "%s, Failed to init formats \n", __func__);
		err = -EFAULT;
		goto error;
	}

	priv->mcu_ctrl_info = devm_kzalloc(&client->dev, 
			sizeof(ISP_CTRL_INFO) * priv->num_ctrls, GFP_KERNEL);
	if(!priv->mcu_ctrl_info) {
		dev_err(&client->dev, "Unable to allocate memory \n");
		err = -ENOMEM;
		goto error;
	}

	priv->ctrldb = devm_kzalloc(&client->dev, 
			sizeof(uint32_t) * priv->num_ctrls, GFP_KERNEL);
	if(!priv->ctrldb) {
		dev_err(&client->dev, "Unable to allocate memory \n");
		err = -ENOMEM;
		goto error;
	}

	priv->stream_info = devm_kzalloc(&client->dev, 
			sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);

	priv->streamdb = devm_kzalloc(&client->dev,
		       	sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!priv->streamdb) {
		dev_err(&client->dev,"Unable to allocate memory \n");
		err = -ENOMEM;
		goto error;
	}

	priv->mcu_cam_frmfmt = devm_kzalloc(&client->dev, 
			sizeof(struct camera_common_frmfmt) * (frm_fmt_size), GFP_KERNEL);
	if(!priv->mcu_cam_frmfmt) {
		dev_err(&client->dev, "Unable to allocate memory \n");
		err = -ENOMEM;
		goto error;
	}


	retry = 0;
	while(retry++ < 5){
		printk("mcu_get_flash_connect_status FLASH CONNECT status check\n");
	if (mcu_get_flash_connect_status(client, &fl_cn_status) < 0) {
		dev_err(&client->dev, "Unable to get FLASH CONNECT STATUS !.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get FLASH CONNECT STATUS !.. Exiting..\n");
	}else
		dev_info(&client->dev,"Fl Connect Status = 0x%02x\n",fl_cn_status);


	if( fl_cn_status != 0x01 ){

		dev_err(&client->dev, "FLASH IS NOT CONNECTED \n");
		dev_err(&client->dev,"Either Flash Corrupted or Camera Module is Not connected\n");
		msleep(100);
		goto bootdata_flash;
	}
	
	if(( spi_flash_bootdata_changed == 0x01 ))
	{

		printk("SPI FLASH Initiated Due to change in Boot-Data\n");
		goto bootdata_flash;

	}

	retry = 0;
	while(retry++ < 5){
	if (mcu_get_load_param_status(client, &lp_pa_status) < 0) {
		dev_err(&client->dev, "Unable to get LOAD PARAM STATUS !.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get LOAD PARAM STATUS !.. Exiting..\n");
		err = -EIO;
		goto error;
	}
	dev_info(&client->dev,"Load Param Status = 0x%02x\n",lp_pa_status);

	if( lp_pa_status != 0x01 ){

		dev_err(&client->dev, "SERIAL NOR FLASH IS NOT LOADED WITH DATA AT SENSOR START UP\nSKIP NV_BOOT STATUS Check and Write Boot_data to NOR FLASH\n");
		boot_variable = 0x01 ;
		goto bootdata_flash;
	}	


	retry = 0;
	while(retry++ < 5){
	if (mcu_get_nv_boot_status(client, &nv_bt_status) < 0) {
		dev_err(&client->dev, "Unable to get NV Boot STATUS!.. Retrying..\n");
		continue;
	}
	else
		break;
	}
	if(retry > 5){
		dev_err(&client->dev, 
				"Unable to get NV Boot STATUS!.. Exiting..\n");
		return -EIO;
	}
	dev_info(&client->dev,"NV BOOT  Status = 0x%02x\n",nv_bt_status);


	if( fl_cn_status == 0x01 && lp_pa_status == 0x01 && nv_bt_status == 0x01 ){

		dev_info(&client->dev,"SPI BOOTDATA IS VALID & SENSOR WILL BOOT FROM SPI\n\r");
		goto skip_bootdata_flash;	


	}

/* ----------------------------- ISX031 Boot Data write Implementation ------------------- */

bootdata_flash:
			retry = 5;
			while(retry -- > 0) {
				if (Sensor_flash_wp_disable(client) < 0) {
						dev_err(&client->dev,
				       			"Unable to ACCESS FLASH = %d \n", retry);
					continue;
				} else {
				break;
				}
			}
			
			if( retry <= 0 ){
				err = -EFAULT;
				goto error;
			}

			msleep(200);

			retry = 5;
			while(retry -- > 0) {
				if (mcu_isp_init_ISP_ERASE_FLASH(client) < 0) {
						dev_err(&client->dev,
				       			"Unable to ERASE SPI FLASH DATA,retry = %d \n", retry);
					continue;
				} else {
				break;
				}
			}
			
			if( retry <= 0 ){
				err = -EFAULT;
				goto error;
			}

			retry = 5;
			while(retry -- > 0) {
				if (mcu_isp_init_ISP(client) < 0) {
						dev_err(&client->dev,
				       			"Unable to WRITE BOOTDATA TO SPI FLASH, retry = %d \n", retry);
					continue;
				} else {
				break;
				}
			}

			if( retry <= 0 ){
				err = -EIO;
				goto error;
			}

			retry = 5;
			while(retry -- > 0) {
				if (Sensor_flash_wp_enable(client) < 0) {
						dev_err(&client->dev,
				       			"Unable to ACCESS FLASH = %d \n", retry);
					continue;
				} else {
				break;
				}
			}
			
			if( retry <= 0 ){
				err = -EIO;
				goto error;
			}
			msleep(200);



skip_bootdata_flash:

		/* Get sensor ID and Init the MCU */
	ret = ecam_mcu_core_initialize(client, priv);
	if(ret < 0){
		dev_err (&client->dev, "%s(%d):ecam MCU Init Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}

	
	/* Configure Serializer/ Deserializer */
	ret = ecam_serdes_config(client, priv);
	if(ret < 0){
		dev_err (&client->dev, "%s(%d):ecam SerDes config Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}	

	for(loop = 0; loop < frm_fmt_size; loop++) {
		priv->mcu_cam_frmfmt[loop].framerates = devm_kzalloc(&client->dev,
			       	sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if(!priv->mcu_cam_frmfmt[loop].framerates) {
			dev_err(&client->dev, "Unable to allocate memory \n");
			err = -ENOMEM;
			goto error;
		}
	}

	/* Enumerate Formats */
	if (mcu_list_fmts(client, priv->stream_info, &frm_fmt_size,priv) < 0) {
		dev_err(&client->dev, "Unable to List Fmts \n");
		err = -EFAULT;
		goto error;
	}

	common_data->ops = NULL;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->frmfmt = priv->mcu_cam_frmfmt;
	common_data->colorfmt =
		camera_common_find_datafmt(ISX031_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = priv->num_ctrls;
	common_data->numfmts = frm_fmt_size;
	common_data->def_mode = ISX031_DEFAULT_MODE;
	common_data->def_width = ISX031_DEFAULT_WIDTH;
	common_data->def_height = ISX031_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	//priv->trigger_gpio = Gtrigger_gpio;
	common_data->def_clk_freq = 24000000;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	priv->prev_index = 0xFFFE;

	err = camera_common_initialize(common_data, "isx031");
	if (err) {
		dev_err(&client->dev, "Failed to initialize isx031.\n");
		goto error;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &cam_subdev_ops);
	/* Enumerate Ctrls */
	err = cam_ctrls_init(priv, priv->mcu_ctrl_info);
	if (err)
		goto error;
	priv->subdev->internal_ops = &cam_subdev_internal_ops;
	priv->subdev->flags |=
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	/*
	   To unload the module driver module, 
	   Set (struct v4l2_subdev *)priv->subdev->sd to NULL.
	   Refer tegracam_v4l2subdev_register() in tegracam_v4l2.c
	 */
	if (priv->subdev->owner == THIS_MODULE) {
		common_data->owner = priv->subdev->owner;
		priv->subdev->owner = NULL;
	} else {
		// It shouldn't come here in probe();
		;
	}


#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &cam_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity,
		       	1, &priv->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		goto error;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		goto error;

	dev_err(&client->dev," ser_status=%x\n",ser_status);
	if(serdes_write_16b_reg(client, priv->des_addr, 0x0010, ser_status) < 0)
	{
		dev_err (&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		err = -EIO;
		goto error;
	}
	dev_info(&client->dev, "Detected ISX031 sensor\n");

	 if(priv->got_pwm_handle) {
                // Set PWM Trigger to default
                pwm_disable(priv->pwm_dev);
                pwm_config(priv->pwm_dev, PWM_30HZ_DUTY, PWM_30HZ_PERIOD);
                pwm_enable(priv->pwm_dev);
                priv->last_sync_mode = 1;
                dev_info(&client->dev, "PWM 30Hz set\n");
        }
	num_cam++;

	return 0;
error:
	printk("Power Off\n");
        isx031_power_off(common_data);
        return err;
	
}

static void toggle_gpio(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio)){
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio, val);
	} else{
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio, val);
	}
}

static int cam_power_put(struct isx031 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (!priv || !priv->pdata)
		return -EINVAL;

	if (unlikely(!pw))
		return -EFAULT;

	regulator_disable(pw->avdd);
	regulator_disable(pw->iovdd);
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (priv->pdata->use_cam_gpio)
		cam_gpio_deregister(&priv->i2c_client->dev, pw->pwdn_gpio);
	else {
		gpio_free(pw->pwdn_gpio);
		gpio_free(pw->reset_gpio);
	}

	return 0;
}
#define FREE_SAFE(dev, ptr) \
	if(ptr) { \
		devm_kfree(dev, ptr); \
	}


static int isx031_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct isx031 *priv = (struct isx031 *)s_data->priv;
	int loop = 0;
	uint8_t ser_read;
	struct device_node *node = client->dev.of_node;

	//int trigger_gpio=0;

	if (!priv || !priv->pdata)
		return -1;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif
	//calibration_exit();
#if 1
	if(priv->phy == PHY_B){
		if(serdes_write_16b_reg(client, priv->ser_addr,
				       	0x0000, SER1_ADDR<<1) < 0)
		{
			dev_err (&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}
		msleep(100);
	}
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	cam_power_put(priv);
	if(!s_data)
		return -1;

	camera_common_remove_debugfs(s_data);

	mutex_destroy(&priv->mcu_i2c_mutex);

if( is_fw_loaded == 1 || is_fw_loaded_ISP == 1 ){


	release_firmware(mcu_fw);
	release_firmware(mcu_fw_ISP);
	
	
	if( mcu_fw_buf != NULL )
	{
		kfree(mcu_fw_buf);
	}
	if( mcu_fw_buf_ISP != NULL )
	{
		kfree(mcu_fw_buf_ISP);
	}

	is_fw_loaded = 0;
	is_fw_loaded_ISP = 0;

	}

printk("CLEAR MEM : %d \n\r", clear);
#if 1
for(loop = 0; loop < priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem
			; loop++) {
		FREE_SAFE(&client->dev, priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu[loop]);
	}

	FREE_SAFE(&client->dev, priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu);

	FREE_SAFE(&client->dev, priv->mcu_ctrl_info);

	for(loop = 0; loop < s_data->numfmts; loop++ ) {
		FREE_SAFE(&client->dev, (void *)priv->mcu_cam_frmfmt[loop].framerates);
	}

	FREE_SAFE(&client->dev, priv->mcu_cam_frmfmt);

	FREE_SAFE(&client->dev, priv->ctrldb);
	FREE_SAFE(&client->dev, priv->streamdb);

	FREE_SAFE(&client->dev, priv->stream_info);
	FREE_SAFE(&client->dev, fw_version);
	FREE_SAFE(&client->dev, priv->pdata);
	FREE_SAFE(&client->dev, priv->s_data);
	FREE_SAFE(&client->dev, priv);

#endif
	return 0;
}

static const struct i2c_device_id isx031_id[] = {
	{ "isx031", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, isx031_id);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.name = "isx031",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(isx031_of_match),
	},
	.probe = isx031_probe,
	.remove = isx031_remove,
	.id_table = isx031_id,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for e-con Cameras");
MODULE_AUTHOR("e-con Systems");
MODULE_LICENSE("GPL v2");
