#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// #include <zephyr.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>
#include <stdint.h>

/* STEP 1.2 - Include the header files for SPI, GPIO and devicetree */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/sys/printk.h>

#include "sensor_func.h"

LOG_MODULE_REGISTER(PAW3395_SPI, LOG_LEVEL_INF);

#define SLEEP_TIME_MS   100
#define SPIOP        	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA //If CPOL is set -> CPOL=1 otherwise 0, same for CPHA
#define MOTION_NODE 	DT_ALIAS(sw0)




int64_t volatile Time_read=0;
int64_t volatile Time_write=0;
bool volatile flag_write=false;
int8_t volatile flag_cb=0;


static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(5);
static enum usb_dc_status_code usb_status;

static struct gpio_dt_spec const led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static struct gpio_dt_spec const motion_pin  = GPIO_DT_SPEC_GET(MOTION_NODE, gpios);
static struct gpio_dt_spec const print_pin  = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
struct gpio_dt_spec const ncs_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ncs), gpios);
static struct gpio_dt_spec const nreset_pin = GPIO_DT_SPEC_GET(DT_ALIAS(nreset), gpios);
struct spi_dt_spec const spispec = SPI_DT_SPEC_GET(DT_NODELABEL(paw3395), SPIOP, 0);
// struct motion_burst volatile MotionBurstData;

extern long get_time(void);
extern int paw_read_regs(uint8_t reg, uint8_t *pdata, uint8_t size);
extern int paw_write_reg(uint16_t add, uint8_t data);
extern int update_values_motion_burst(void);
extern void CPI_set(uint16_t resolution_x, uint16_t resolution_y);
extern void power_up_init_reg(void);

extern bool burst;

enum mouse_report_idx
{
    MOUSE_BTN_REPORT_IDX = 0,
    MOUSE_X_REPORT_IDX = 1,
    MOUSE_Y_REPORT_IDX = 2,
    MOUSE_WHEEL_REPORT_IDX = 3,
    MOUSE_REPORT_COUNT = 4,
};

static int8_t report[MOUSE_REPORT_COUNT];
// static bool report_updated = false;

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    usb_status = status;
}

static ALWAYS_INLINE void rwup_if_suspended(void)
{
    if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP))
    {
        if (usb_status == USB_DC_SUSPEND)
        {
            usb_wakeup_request();
            return;
        }
    }
}
// static void input_cb(struct input_event *evt)
// {
//     uint8_t tmp[MOUSE_REPORT_COUNT];

//     (void)memcpy(tmp, report, sizeof(tmp));

//     if (memcmp(tmp, report, sizeof(tmp)))
//     {
//         memcpy(report, tmp, sizeof(tmp));
//         report_updated = true;
//     }
// }


static void power_up_sequence(void){
	k_msleep(50);
	// gpio_pin_configure_dt(&ncs_pin, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&ncs_pin,1);
	k_msleep(10);
	gpio_pin_set_dt(&ncs_pin,0);
	// spi_is_ready_dt(&spispec);
	// k_usleep(10);
	// gpio_pin_set_dt(&ncs_pin,1);
	paw_write_reg(POWER_UP_RESET_REG, 0x5A);
	k_msleep(5);
	power_up_init_reg();
	uint8_t value[1];
	for(uint8_t i = 0x02; i<=0x06; i++){
		paw_read_regs(i, value, 1);
		// printk("initial values");
	}
	// paw_read_regs(MOTION_CTRL_REG, value, 1);
	// paw_write_reg(MOTION_CTRL_REG,(value[0]|(1<<7)));
	// paw_read_regs(MOTION_CTRL_REG, value, 1);
	// printk("\nMOTION CTRL: %X\n",value[0]);
	gpio_pin_set_dt(&nreset_pin,1);
	k_usleep(10);
	gpio_pin_set_dt(&nreset_pin,0);

	// paw_read_regs(PERFORMANCE_REG, value, 1);
	// paw_write_reg(PERFORMANCE_REG,(value[0]|(1<<7)));
	// paw_read_regs(PERFORMANCE_REG, value, 1);
	// printk("\nPERFORMANCE: %X\n",value[0]);

	// paw_write_reg(ACCESS_REG,0x0C);
	// paw_read_regs(0x4E, value, 1);
	// paw_write_reg(0x4E,(value[0]|(1<<1)));
	// paw_read_regs(0x4E, value, 1);
	// printk("\nLIFT_CONFIGG: %X\n",value[0]);
	// paw_write_reg(ACCESS_REG,0x00);
}

void cb_motion(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	// gpio_pin_toggle_dt(&led);
	flag_cb++;              //eventually create semaphore for this task
}


ALWAYS_INLINE uint16_t complement(uint16_t x)
{
	return ((~x) + 1);
}

// INPUT_CALLBACK_DEFINE(&motion_pin, input_cb);

int main(void)
{
	int err;
	const struct device *hid_dev;
	#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	#else
	hid_dev = device_get_binding("HID_0");
	#endif

	if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&motion_pin) || !gpio_is_ready_dt(&ncs_pin)) {
		return 0;
	}
	err = spi_is_ready_dt(&spispec);
	if (!err) {
		LOG_ERR("Error: SPI device is not ready, err: %d", err);
		return 0;
	}
	
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&ncs_pin, GPIO_OUTPUT);
	gpio_pin_configure_dt(&nreset_pin, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&motion_pin, GPIO_INPUT);
	gpio_pin_configure_dt(&print_pin, GPIO_INPUT);
	
	gpio_pin_set_dt(&nreset_pin,1);
	k_usleep(10);
	gpio_pin_set_dt(&nreset_pin,0);
	power_up_sequence();			//Power on reset for the sensor
	
	CPI_set(10000,10000);
	
	

	gpio_pin_interrupt_configure_dt(&motion_pin, GPIO_INT_EDGE_TO_ACTIVE);
	static struct gpio_callback motion_cb_data;
	
	uint8_t val[1];
	paw_read_regs(RESOLUTION_X_LOW_REG,val,1);  
	printk("%X ", val[0]);
	paw_read_regs(RESOLUTION_Y_LOW_REG,val,1);  
	printk("%X\n", val[0]); 
	// paw_read_regs(0x00,val,1);  //default value 0x51   (product ID reg)
	paw_read_regs(0x5f,val,1);     // default value 0xAE  (inv product ID reg)
	printk("product ID reg: %x\n",val[0]);
	paw_write_reg(0x15,0x00);

	k_msleep(550);

	paw_read_regs(0x15,val,1);
	printk("Observation reg: %x\n",val[0]);
	paw_read_regs(0x02,val,1);
	k_msleep(10);
	//configurng and setting up USB as HID
	usb_hid_register_device(hid_dev,									
                            hid_report_desc, sizeof(hid_report_desc),
                            NULL);

    usb_hid_init(hid_dev);
	usb_enable(status_cb);

	gpio_init_callback(&motion_cb_data,cb_motion,BIT(motion_pin.pin));
	gpio_add_callback(motion_pin.port, &motion_cb_data);
	// update_values_motion_burst();
	uint16_t resolution_x=100;
	uint16_t resolution_y=100;
	uint8_t value[12];
	uint16_t del_x=0;
	uint16_t del_y=0;
	report[MOUSE_BTN_REPORT_IDX] = 0U;
	report[MOUSE_WHEEL_REPORT_IDX] = 0U;
	while (1){
		
		if(flag_cb){
			// for(uint32_t volatile i = 0 ; i<500000/2 ; i++);
			
			int err=0;
			// !burst ? paw_write_reg(MOTION_BURST_REG,0x00) : NULL;
			err=paw_read_regs(MOTION_BURST_REG, value, 12);
			if(value[0] & 0x80)
			{
				//upload the mouse report
				del_x = (value[2]<<8 | value[1]);
            	del_y = (value[4]<<8 | value[3]);

				uint8_t temp;
				if(del_x>32767)
				{
					gpio_pin_set_dt(&led,1);
					temp = -1*complement(del_x)*128/32768;
				}
				else{
					gpio_pin_set_dt(&led,0);
					temp = del_x*128/32768; 
				}
				report[MOUSE_X_REPORT_IDX] = temp;

				if(del_y>32767)
				{
					// gpio_pin_set_dt(&led,1);
					temp = -1*complement(del_y)*128/32768;
				}
				else{
					// gpio_pin_set_dt(&led,0);
					temp = del_y*128/32768; 
				}
				report[MOUSE_Y_REPORT_IDX] = temp;





				// report[MOUSE_X_REPORT_IDX] = del_x>32767 ? (-1*complement(del_x))/(2*resolution_x) : del_x/(2*resolution_x);
				// report[MOUSE_Y_REPORT_IDX] = del_y>32767 ? (-1*((~del_y) +1))/(2*resolution_y) : del_y/(2*resolution_y);

				// report[MOUSE_X_REPORT_IDX] = -(value[2]<<8 | value[1])/resolution_x;
            	// report[MOUSE_Y_REPORT_IDX] = -(value[4]<<8 | value[3])/resolution_y;
				err = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
				report[MOUSE_X_REPORT_IDX] = 0U;
				report[MOUSE_Y_REPORT_IDX] = 0U;
			}
			// paw_write_reg(0x02,0x00);
			// burst = true;
			flag_cb--;
		}
		// k_usleep(1);
	}
	return 0;
}