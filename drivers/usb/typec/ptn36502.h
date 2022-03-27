#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/version.h>

#include <linux/semaphore.h>
#include <linux/completion.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/nfcinfo.h>


#define CHIP_ID 0X00
#define CHIP_REVISION 0X01
#define USB_US_TX_RX_CONTROL 0X04
#define USB_DS_TX_RX_CONTROL 0X05
#define DP_LINK_CONTROL 0X06
#define DP_LINK_0_TX_RX_CONTROL 0X07
#define DP_LINK_1_TX_RX_CONTROL 0X08
#define DP_LINK_2_TX_RX_CONTROL 0X09
#define DP_LINK_3_TX_RX_CONTROL 0X0A
#define MODE_CONTROL_1 0X0B
#define SQUELCH_THRESHOLD 0X0C
#define DEVICE_CONTROL 0X0D

void set_ptn36502_safe_state_mode (void);
void set_ptn36502_usp_3p0_only_mode (int cc_orient_reversed);
void set_ptn36502_usp3_and_dp2lane_mode (int cc_orient_reversed);
void set_ptn36502_dp4lane_mode (int cc_orient_reversed);

extern int typec_cc_orientation;
extern u8 ptn36502_is_safe_mode;
