#include <media/camera_common.h>

#define CMD_SIGNATURE		0x43
#define TX_LEN_PKT			5
#define RX_LEN_PKT			6
#define HEADER_FOOTER_SIZE	4
#define CMD_STATUS_MSG_LEN	7

#define VERSION_SIZE			32
#define VERSION_FILE_OFFSET			99

#define MCU_CMD_STATUS_SUCCESS		0x0000
#define MCU_CMD_STATUS_PENDING		0xF000
#define MCU_CMD_STATUS_ISP_PWDN		0x0FF0
#define MCU_CMD_STATUS_ISP_UNINIT	0x0FF1

#define MAX_NUM_FRATES                 10
#define MAX_CTRL_DATA_LEN 		100
#define MAX_CTRL_UI_STRING_LEN 		32
/* Defines related to SOC find */
#define JETSON_TX2 	0x18
#define JETSON_TX1 	0x21
/* Defines related to Deserializer SIO port*/
#define PHY_A 'A'
#define PHY_B 'B'

#define ISX031_DEFAULT_MODE	0

#define ISX031_DEFAULT_WIDTH    1920
#define ISX031_DEFAULT_HEIGHT	1536
#define ISX031_DEFAULT_DATAFMT  MEDIA_BUS_FMT_UYVY8_1X16




#define ISX_BUFFER_SIZE			256




typedef enum _errno {
	ERRCODE_SUCCESS = 0x00,
	ERRCODE_BUSY = 0x01,
	ERRCODE_INVAL = 0x02,
	ERRCODE_PERM = 0x03,
	ERRCODE_NODEV = 0x04,
	ERRCODE_IO = 0x05,
	ERRCODE_HW_SPEC = 0x06,
	ERRCODE_AGAIN = 0x07,
	ERRCODE_ALREADY = 0x08,
	ERRCODE_NOTIMPL = 0x09,
	ERRCODE_RANGE = 0x0A,

	/*   Reserved 0x0B - 0xFE */

	ERRCODE_UNKNOWN = 0xFF,
} RETCODE;

typedef enum _cmd_id {
	CMD_ID_VERSION = 0x00,
	CMD_ID_GET_SENSOR_ID = 0x01,
	CMD_ID_GET_STREAM_INFO = 0x02,
	CMD_ID_GET_CTRL_INFO = 0x03,
	CMD_ID_INIT_CAM = 0x04,
	CMD_ID_GET_STATUS = 0x05,
	CMD_ID_DE_INIT_CAM = 0x06,
	CMD_ID_STREAM_ON = 0x07,
	CMD_ID_STREAM_OFF = 0x08,
	CMD_ID_STREAM_CONFIG = 0x09,
	CMD_ID_GET_CTRL_UI_INFO = 0x0A,

	/* Reserved 0x0B to 0x0F */

	CMD_ID_GET_CTRL = 0x10,
	CMD_ID_SET_CTRL = 0x11,
	
	CMD_ID_ISP_READ = 0x12,
	CMD_ID_ISP_WRITE = 0x13,

	CMD_ID_FW_UPDT = 0x14,
	CMD_ID_ISP_PDOWN = 0x15,
	CMD_ID_ISP_PUP = 0x16,

	/* Configuring MIPI Lanes */
	CMD_ID_LANE_CONFIG = 0x17,
	CMD_ID_MIPI_CLK_CONFIG = 0x18,

	CMD_ID_SENSOR_READ = 0x19,
	CMD_ID_SENSOR_WRITE = 0x1A,


	/* Below CMD ID's for ISX031 SEnsor */

	CMD_ID_INIT_CAM_ISP 		= 0xA0,
	CMD_ID_INIT_CAM_ISP_ERASE 	= 0xA1,
	CMD_ID_FL_STATUS 		= 0xA2,
	CMD_ID_LOAD_PARAM_STATUS 	= 0xA3,
	CMD_ID_NV_STATUS 		= 0xA4,
	CMD_ID_XERROR_STATUS 		= 0xA5,
	CMD_ID_GET_FW_KEY		= 0xA6,
	CMD_ID_INIT_FLASH_WP		= 0xA7,
	CMD_ID_INIT_FLASH_WP_DISABLE	= 0xA8,
	CMD_ID_VERSION_SPI		= 0xA9,

	/* Reserved - 0x1B to 0xFE (except 0x43) */

	CMD_ID_UNKNOWN = 0xFF,

} HOST_CMD_ID;

enum {
	FRAME_RATE_DISCRETE = 0x01,
	FRAME_RATE_CONTINOUS = 0x02,
};

enum {
	CTRL_STANDARD = 0x01,
	CTRL_EXTENDED = 0x02,
};



enum {
/*  0x01 - Integer (32bit)
		0x02 - Long Int (64 bit)
		0x03 - String
		0x04 - Pointer to a 1-Byte Array
		0x05 - Pointer to a 2-Byte Array
		0x06 - Pointer to a 4-Byte Array
		0x07 - Pointer to Generic Data (custom Array)
*/

	EXT_CTRL_TYPE_INTEGER = 0x01,
	EXT_CTRL_TYPE_LONG = 0x02,
	EXT_CTRL_TYPE_STRING = 0x03,
	EXT_CTRL_TYPE_PTR8 = 0x04,
	EXT_CTRL_TYPE_PTR16 = 0x05,
	EXT_CTRL_TYPE_PTR32 = 0x06,
	EXT_CTRL_TYPE_VOID = 0x07,
};

typedef struct _isp_stream_info {
	uint32_t fmt_fourcc;
	uint16_t width;
	uint16_t height;
	uint8_t frame_rate_type;
	union {
		struct {
			uint16_t frame_rate_num;
			uint16_t frame_rate_denom;
		} disc;
		struct {
			uint16_t frame_rate_min_num;
			uint16_t frame_rate_min_denom;
			uint16_t frame_rate_max_num;
			uint16_t frame_rate_max_denom;
			uint16_t frame_rate_step_num;
			uint16_t frame_rate_step_denom;
		} cont;
	} frame_rate;
} ISP_STREAM_INFO;


typedef struct _isp_ctrl_ui_info {
	struct {
		char ctrl_name[MAX_CTRL_UI_STRING_LEN];
		uint8_t ctrl_ui_type;
		uint8_t ctrl_ui_flags;
	} ctrl_ui_info;

	/* This Struct is valid only if ctrl_ui_type = 0x03 */
	struct {
		uint8_t num_menu_elem;
		char **menu;
	} ctrl_menu_info;
} ISP_CTRL_UI_INFO;

typedef struct _isp_ctrl_info_std {
	uint32_t ctrl_id;
	uint8_t ctrl_type;
	union {
		struct {
			int32_t ctrl_min;
			int32_t ctrl_max;
			int32_t ctrl_def;
			int32_t ctrl_step;
		} std;
		struct {
			uint8_t val_type;
			uint32_t val_length;
			// This size may vary according to ctrl types
			uint8_t val_data[MAX_CTRL_DATA_LEN];
		} ext;
	} ctrl_data;
	ISP_CTRL_UI_INFO ctrl_ui_data;
} ISP_CTRL_INFO;


enum {
	V4L2_CID_FACEDETECT = (V4L2_CID_AUTO_FOCUS_RANGE+1),
	V4L2_CID_FACEMARK,
	V4L2_CID_SMILEDETECT,
	V4L2_GET_FACEINFO,
	V4L2_CID_ROI_WINDOW,
	V4L2_CID_ROI_FOCUS,
	V4L2_CID_ROI_EXPOSURE,
	V4L2_CID_TRIGGER_FOCUS,

	/* New Controls */
	V4L2_CID_HDR,
	V4L2_CID_CUSTOM_TRIGGER,
	V4L2_CID_COLORKILL,
	V4L2_CID_FRAME_SYNC,
	V4L2_CID_CUSTOM_EXPOSURE_AUTO,
	V4L2_CID_CUSTOM_FLASH_STROBE,
	V4L2_CID_DENOISE,
	V4L2_CID_GRAYSCALE,
	V4L2_CID_LSCMODE,
	V4L2_CID_EXPOSURE_COMPENSATION,
	V4L2_CID_FOCUS_WINDOW,
	V4L2_CID_FRAME_RATE_CONTROL,
	V4L2_CID_SPECIAL_EFFECT,
	V4L2_CID_PWM_CONFIG,
};

DEFINE_MUTEX(g_i2c_mutex);



static int cam_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int cam_s_ctrl(struct v4l2_ctrl *ctrl);
static int cam_read(struct i2c_client *client, u8 * val, u32 count);
static int cam_write(struct i2c_client *client, u8 * val, u32 count);
static int cam_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
int cam_s_power(struct v4l2_subdev *sd, int on);
static void toggle_gpio(unsigned int gpio, int value);
static int mcu_get_fw_version(struct i2c_client *client, unsigned char * fw_version, unsigned char *txt_fw_version);
unsigned char errorcheck(char *data, unsigned int len);

//static int mcu_list_fmts(struct i2c_client *client, ISP_STREAM_INFO *stream_info, int *frm_fmt_size,struct isx031 *);
//static int mcu_list_ctrls(struct i2c_client *client,ISP_CTRL_INFO * mcu_ctrl_info, struct isx031 *);
static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id);
static int mcu_get_cmd_status(struct i2c_client *client, uint8_t * cmd_id,
			      uint16_t * cmd_status, uint8_t * ret_code);
static int mcu_isp_init(struct i2c_client *client);
static int mcu_stream_config(struct i2c_client *client, uint32_t format,
			     int mode, int frate_index);
static int mcu_set_ctrl(struct i2c_client *client, uint32_t ctrl_id,
			uint8_t ctrl_type, int32_t curr_val);
static int mcu_get_ctrl(struct i2c_client *client, uint32_t ctrl_id,
			uint8_t * ctrl_type, int32_t * curr_val);
static int mcu_get_ctrl_ui(struct i2c_client *client,
			   ISP_CTRL_INFO * mcu_ui_info, int index);
static int mcu_fw_update(struct i2c_client *client, unsigned char *txt_fw_version);
static int mcu_cam_stream_on(struct i2c_client *client);
static int mcu_cam_stream_off(struct i2c_client *client);
static int mcu_get_nv_boot_status(struct i2c_client *client, uint8_t * nv_status);
static int mcu_get_load_param_status(struct i2c_client *client, uint8_t * lp_status);
static int mcu_get_flash_connect_status(struct i2c_client *client, uint8_t * fl_status);
//static int gen_mcu_stream_config(struct i2c_client *client,struct isx031 *priv);
static int Sensor_flash_wp_enable(struct i2c_client *client);
static int Sensor_flash_wp_disable(struct i2c_client *client);