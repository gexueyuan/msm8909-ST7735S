#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include "7735s_gpio.c"
uint8_t image_buffer[] = {

	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
	0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,
};

static int spi_fd;
static const char *device = "/dev/spidev0.0";
static uint8_t mode = 3;
static uint8_t bits = 9;
static uint32_t speed =19200000;
static uint16_t delay;
uint8_t startbyte = 0;
int bgr = 0;
int rotate = 0;

#define  TFT_WIDTH  	240
#define  TFT_HEIGHT 	320
#define	 TFT_BPP		16
#define	 TFT_CMD 	0
#define	 TFT_DATA	1
#define	 GAMMA_NUM	2
#define	GAMMA_VALUE	16

#define  SPI_GPIO_DC	912
#define	 SPI_GPIO_RESET	914
#define	 SPI_GPIO_PWM	1006

uint8_t  spi_tft_buf[240*320*2*2];
uint8_t  spi_buf[128];
#define FBTFT_MAX_INIT_SEQUENCE      512
#define FBTFT_GAMMA_MAX_VALUES_TOTAL 128
/* MIPI DCS commands */
enum {
	MIPI_DCS_NOP			= 0x00,
	MIPI_DCS_SOFT_RESET		= 0x01,
	MIPI_DCS_GET_DISPLAY_ID		= 0x04,
	MIPI_DCS_GET_RED_CHANNEL	= 0x06,
	MIPI_DCS_GET_GREEN_CHANNEL	= 0x07,
	MIPI_DCS_GET_BLUE_CHANNEL	= 0x08,
	MIPI_DCS_GET_DISPLAY_STATUS	= 0x09,
	MIPI_DCS_GET_POWER_MODE		= 0x0A,
	MIPI_DCS_GET_ADDRESS_MODE	= 0x0B,
	MIPI_DCS_GET_PIXEL_FORMAT	= 0x0C,
	MIPI_DCS_GET_DISPLAY_MODE	= 0x0D,
	MIPI_DCS_GET_SIGNAL_MODE	= 0x0E,
	MIPI_DCS_GET_DIAGNOSTIC_RESULT	= 0x0F,
	MIPI_DCS_ENTER_SLEEP_MODE	= 0x10,
	MIPI_DCS_EXIT_SLEEP_MODE	= 0x11,
	MIPI_DCS_ENTER_PARTIAL_MODE	= 0x12,
	MIPI_DCS_ENTER_NORMAL_MODE	= 0x13,
	MIPI_DCS_EXIT_INVERT_MODE	= 0x20,
	MIPI_DCS_ENTER_INVERT_MODE	= 0x21,
	MIPI_DCS_SET_GAMMA_CURVE	= 0x26,
	MIPI_DCS_SET_DISPLAY_OFF	= 0x28,
	MIPI_DCS_SET_DISPLAY_ON		= 0x29,
	MIPI_DCS_SET_COLUMN_ADDRESS	= 0x2A,
	MIPI_DCS_SET_PAGE_ADDRESS	= 0x2B,
	MIPI_DCS_WRITE_MEMORY_START	= 0x2C,
	MIPI_DCS_WRITE_LUT		= 0x2D,
	MIPI_DCS_READ_MEMORY_START	= 0x2E,
	MIPI_DCS_SET_PARTIAL_AREA	= 0x30,
	MIPI_DCS_SET_SCROLL_AREA	= 0x33,
	MIPI_DCS_SET_TEAR_OFF		= 0x34,
	MIPI_DCS_SET_TEAR_ON		= 0x35,
	MIPI_DCS_SET_ADDRESS_MODE	= 0x36,
	MIPI_DCS_SET_SCROLL_START	= 0x37,
	MIPI_DCS_EXIT_IDLE_MODE		= 0x38,
	MIPI_DCS_ENTER_IDLE_MODE	= 0x39,
	MIPI_DCS_SET_PIXEL_FORMAT	= 0x3A,
	MIPI_DCS_WRITE_MEMORY_CONTINUE	= 0x3C,
	MIPI_DCS_READ_MEMORY_CONTINUE	= 0x3E,
	MIPI_DCS_SET_TEAR_SCANLINE	= 0x44,
	MIPI_DCS_GET_SCANLINE		= 0x45,
	MIPI_DCS_READ_DDB_START		= 0xA1,
	MIPI_DCS_READ_DDB_CONTINUE	= 0xA8,
};

/* MIPI DCS pixel formats */
#define MIPI_DCS_PIXEL_FMT_24BIT	7
#define MIPI_DCS_PIXEL_FMT_18BIT	6
#define MIPI_DCS_PIXEL_FMT_16BIT	5
#define MIPI_DCS_PIXEL_FMT_12BIT	3
#define MIPI_DCS_PIXEL_FMT_8BIT		2
#define MIPI_DCS_PIXEL_FMT_3BIT		1

struct spi_ioc_transfer tr;	

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int file_exist(const char* _fileName)
{

    if((access(_fileName,F_OK))!=-1)   
    {   
        printf("\nfile : %s exist.\n",_fileName);
        return 1;
    }
    else{
        
        printf("\nfile : %s  Non-existent.\n",_fileName);
        return 0;

    }



}

/*
 * 函数说明:  读二进制文件
*  参数描述: _fileName, 文件名称
*             _buf, 读出来的数据存放位置
*             _bufLen, 数据的长度信息
*    返回值:  0, 成功
*             -1, 失败
*
*/
int readFile(const char* _fileName, void* _buf, int _bufLen)
{
    FILE* fp = NULL;
    if( NULL == _buf || _bufLen <= 0 ) return (-1);

    fp = fopen(_fileName, "rb"); // 必须确保是以 二进制读取的形式打开 

    if( NULL == fp )
    {
        return (-1);
    }

    fread(_buf, _bufLen, 1, fp); // 二进制读

    fclose(fp);
    return 0;        
}

static void pabort(const char *s)
{
	perror(s);
	abort();
}


static void transfer(int fd)
{
	int ret;
	uint8_t tx[100] = {
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55,
	};
	memcpy(tx,image_buffer,100);	
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
}
#define  MAX_BYTE_ONCE	 4096
static int fbtft_write_spi(void *buf, int len)
{
int ret = 0;
int i = 0;
uint8_t* p_buf = buf;
/*
int ret;
	uint8_t tx[100] = {
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55,
	};
	memcpy(tx,image_buffer,100);	
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");	
*/
/*
	int ret = 0;
	int i = 0;
	
	struct spi_ioc_transfer tft_spi = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = 0,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tft_spi);
	if (ret < 1)
		pabort("can't send spi message");
*/
/*
	uint8_t * p_buf = buf;
	for(i = 0;i< len;i++){

		printf("buf[%d] is %x ",i,*(uint8_t*)p_buf++);

	}
	printf("\n");
*/
	while(len > MAX_BYTE_ONCE ){
		
		ret = write(spi_fd,p_buf,MAX_BYTE_ONCE);

		len -= MAX_BYTE_ONCE;

		p_buf +=MAX_BYTE_ONCE;

	}

	if(len > 0)	
		ret = write(spi_fd,p_buf,len);
	if (ret < 0)
		pabort("can't send spi message");
	return ret;

}

/* 16 bit pixel over 9-bit SPI bus: dc + high byte, dc + low byte */
int fbtft_write_vmem16_bus9(int fd, void*buffer,size_t offset, size_t len)
{
	uint8_t *vmem8;
	uint16_t *txbuf16 = (uint16_t*)spi_tft_buf;
	int remain;
	int to_copy;
	int tx_array_size;
	int i;
	int ret = 0;

	printf("%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	if (!buffer) {
		printf("%s: txbuf.buf is NULL\n", __func__);
		return -1;
	}
	vmem8 = buffer;

	for (i = 0; i < len; i ++) {
		//txbuf16[i + 1] = ((0x0100 | (uint16_t)vmem8[i]))>>1 | ((((uint16_t)vmem8[i]&0x01)<<15)&0x8000);
		txbuf16[i] = ((0x0100 | (uint16_t)vmem8[i]))>>1 | ((((uint16_t)vmem8[i]&0x01)<<15)&0x8000);
		

//		if (!(i % 6))
//			puts("");
		//if(vmem8[i] != 0)
			//printf("\n%.2X %.2X %.2X %.2X\n", vmem8[i],vmem8[i+1],txbuf16[i],txbuf16[i+1]);
	}
	printf("buff len is %d\n",len);
	fbtft_write_spi(spi_tft_buf,len * 2);
	if (ret < 0)
		return ret;
	return ret;
}

static void send_cmd(int fd, uint8_t cmd)
{
	int ret;
	uint16_t tx[1] = {0};
	uint16_t rx[ARRAY_SIZE(tx)] = {0};

	tx[0] = ((0x00FF & (uint16_t)cmd))>>1 | ((((uint16_t)cmd&0x01)<<15)&0x8000);
	//tx[0] = cmd;
    //printf("%.4X ", tx[0]);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = sizeof(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
 
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send cmd");
	/*
	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
 */
}
 
static void send_data(int fd, uint8_t data)
{
	int ret;
	uint16_t tx[1];
	uint16_t rx[ARRAY_SIZE(tx)] = {0};
 
	tx[0] = ((0x0100 | (uint16_t)data))>>1 | ((((uint16_t)data&0x01)<<15)&0x8000);
	//tx[0] = 0x100 | (uint16_t)data;
	//printf("%.4X ", tx[0]);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = sizeof(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
 
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi data");
	/*
 	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
	*/
}
 

int fbtft_write_buf_dc(void *buf, size_t len, int dc)
{
	int ret;

	if(dc ){

		//printf("write data to spi\n");
		//send_data(spi_fd,)

	}
	else{

		//printf("write cmd to spi\n");
		//send_cmd()
	}
	usleep(2);
	u_gpio_set_value(SPI_GPIO_DC, dc);
	usleep(2);
	ret = fbtft_write_spi(buf, len);
	
	usleep(2);
	if (ret < 0)
		printf("write() failed and returned %d\n", ret);
	return ret;
}

/*****************************************************************************
 *
 *	 void (*write_reg)(struct fbtft_par *par, int len, ...);
 *
 *****************************************************************************/
static void write_register(uint8_t *reg_buf, int len, ...)
{
	va_list args;
	int i, ret;
	uint8_t *buf = reg_buf;
	//printf("buf address is %x,spi_buf is %x\n",buf,spi_buf);
	va_start(args, len);

	*buf = (uint8_t)va_arg(args, unsigned int);
	
	u_gpio_set_value(SPI_GPIO_DC, 0);
	//ret = fbtft_write_spi(reg_buf, sizeof(uint8_t));
	ret = fbtft_write_buf_dc(reg_buf, sizeof(uint8_t),0);
	if (ret < 0) {
		va_end(args);
		printf("write() failed and returned %d\n", ret);
		return;
	}
	len--;

	if (len) {
		i = len;
		while (i--)
			*buf++ = (uint8_t)va_arg(args, unsigned int);
		//ret = fbtft_write_spi(reg_buf, len * (sizeof(uint8_t)));
		ret = fbtft_write_buf_dc(reg_buf, len * (sizeof(uint8_t)),1);
		if (ret < 0) {
			va_end(args);
			printf("write() failed and returned %d\n", ret);
			return;
		}
	}
	u_gpio_set_value(SPI_GPIO_DC, 1);
	va_end(args);
}

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__})/sizeof(int))

#define write_reg(par, ...)                                              \
	write_register(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

/*****************************************************************************
 st7735s param                                              *
*****************************************************************************/
static const int16_t default_init_sequence[] = {
	-1, MIPI_DCS_SOFT_RESET,
	-2, 150,                               /* delay */

	-1, MIPI_DCS_EXIT_SLEEP_MODE,
	-2, 500,                               /* delay */

	/* FRMCTR1 - frame rate control: normal mode
	 * frame rate = fosc / (1 x 2 + 40) * (LINE + 2C + 2D)
	 */
	-1, 0xB1, 0x05, 0x3A, 0x3A,

	/* FRMCTR2 - frame rate control: idle mode
	 * frame rate = fosc / (1 x 2 + 40) * (LINE + 2C + 2D)
	 */
	-1, 0xB2, 0x05, 0x3A, 0x3A,

	/* FRMCTR3 - frame rate control - partial mode
	 * dot inversion mode, line inversion mode
	 */
	-1, 0xB3, 0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A,

	/* INVCTR - display inversion control
	 * no inversion
	 */
	-1, 0xB4, 0x03,

	/* PWCTR1 - Power Control
	 * -4.6V, AUTO mode
	 */
	-1, 0xC0, 0x28, 0x08, 0x84,

	/* PWCTR2 - Power Control
	 * VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	 */
	-1, 0xC1, 0xC0,

	/* PWCTR3 - Power Control
	 * Opamp current small, Boost frequency
	 */
	-1, 0xC2, 0x0C, 0x00,

	/* PWCTR4 - Power Control
	 * BCLK/2, Opamp current small & Medium low
	 */
	-1, 0xC3, 0x8C, 0x2A,

	/* PWCTR5 - Power Control */
	-1, 0xC4, 0x8A, 0xEE,

	/* VMCTR1 - Power Control */
	-1, 0xC5, 0x0C,

	-1, MIPI_DCS_EXIT_INVERT_MODE,

	-1, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT,

	-1, MIPI_DCS_SET_DISPLAY_ON,
	-2, 100,                               /* delay */

	-1, MIPI_DCS_ENTER_NORMAL_MODE,
	-2, 10,                               /* delay */

	/* end marker */
	-3
};
	
/*****************************************************************************
 spi ops                                 *
*****************************************************************************/
static void fbtft_set_addr_win(int xs, int ys, int xe,
			       int ye)
{
	write_reg(spi_buf, MIPI_DCS_SET_COLUMN_ADDRESS,
		  (xs >> 8) & 0xFF, xs & 0xFF, (xe >> 8) & 0xFF, xe & 0xFF);

	write_reg(spi_buf, MIPI_DCS_SET_PAGE_ADDRESS,
		  (ys >> 8) & 0xFF, ys & 0xFF, (ye >> 8) & 0xFF, ye & 0xFF);

	write_reg(spi_buf, MIPI_DCS_WRITE_MEMORY_START);
}

#define BIT(nr)			(1UL << (nr))

#define MY BIT(7)
#define MX BIT(6)
#define MV BIT(5)
static int set_var(void)
{
	/* MADCTL - Memory data access control
	 * RGB/BGR:
	 * 1. Mode selection pin SRGB
	 *    RGB H/W pin for color filter setting: 0=RGB, 1=BGR
	 * 2. MADCTL RGB bit
	 *    RGB-BGR ORDER color filter panel: 0=RGB, 1=BGR
	 */
	switch (rotate) {
	case 0:
		write_reg(spi_buf, MIPI_DCS_SET_ADDRESS_MODE,
			  MX | MY | (bgr << 3));
		break;
	case 270:
		write_reg(spi_buf, MIPI_DCS_SET_ADDRESS_MODE,
			  MY | MV | (bgr << 3));
		break;
	case 180:
		write_reg(spi_buf, MIPI_DCS_SET_ADDRESS_MODE,
			  bgr << 3);
		break;
	case 90:
		write_reg(spi_buf, MIPI_DCS_SET_ADDRESS_MODE,
			  MX | MV | (bgr << 3));
		break;
	}

	return 0;
}

#define DEFAULT_GAMMA   "0F 1A 0F 18 2F 28 20 22 1F 1B 23 37 00 07 02 10\n" \
"0F 1B 0F 17 33 2C 29 2E 30 30 39 3F 00 07 03 10"

uint8_t defualt_gamma[GAMMA_NUM][GAMMA_VALUE]={0x0F,0x1A,0x0F,0x18,0x2F,0x28,0x20,0x22,0x1F,0x1B,0x23,0x37,0x00,0x07,0x02,0x10,\
0x0F,0x1B,0x0F,0x17,0x33,0x2C,0x29,0x2E,0x30,0x30,0x39,0x3F,0x00,0x07,0x03,0x10
};
/*
 * Gamma string format:
 * VRF0P VOS0P PK0P PK1P PK2P PK3P PK4P PK5P PK6P PK7P PK8P PK9P SELV0P SELV1P SELV62P SELV63P
 * VRF0N VOS0N PK0N PK1N PK2N PK3N PK4N PK5N PK6N PK7N PK8N PK9N SELV0N SELV1N SELV62N SELV63N
 */
#define CURVE(num, idx)  curves[num * GAMMA_VALUE + idx]
static int set_gamma(uint32_t *curves)
{
	int i, j;

	/* apply mask */
	for (i = 0; i < GAMMA_NUM; i++)
		for (j = 0; j < GAMMA_VALUE; j++)
			CURVE(i, j) &= 0x3f;

	for (i = 0; i < GAMMA_NUM; i++)
		write_reg(spi_buf, 0xE0 + i,
			CURVE(i, 0), CURVE(i, 1), CURVE(i, 2), CURVE(i, 3),
			CURVE(i, 4), CURVE(i, 5), CURVE(i, 6), CURVE(i, 7),
			CURVE(i, 8), CURVE(i, 9), CURVE(i, 10), CURVE(i, 11),
			CURVE(i, 12), CURVE(i, 13), CURVE(i, 14), CURVE(i, 15));

	return 0;
}
#undef CURVE

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}
static void fbtft_reset(void)
{

	u_gpio_set_value(SPI_GPIO_RESET,0);
	usleep(40);
	u_gpio_set_value(SPI_GPIO_RESET, 1);
	usleep(120*1000);
}

/**
 * fbtft_init_display() - Generic init_display() function
 * @par: Driver data
 *
 * Uses par->init_sequence to do the initialization
 *
 * Return: 0 if successful, negative if error
 */
int fbtft_init_display(void)
{
	int buf[64];
	char msg[128];
	char str[16];
	int i = 0;
	int j;

	/* make sure stop marker exists */
	for (i = 0; i < FBTFT_MAX_INIT_SEQUENCE; i++)
		if (default_init_sequence[i] == -3)
			break;
	if (i == FBTFT_MAX_INIT_SEQUENCE) {
		printf("missing stop marker at end of init sequence\n");
		return -EINVAL;
	}

	fbtft_reset();
	
	//gpio_set_value(par->gpio.cs, 0);  /* Activate chip */

	i = 0;
	while (i < FBTFT_MAX_INIT_SEQUENCE) {
		if (default_init_sequence[i] == -3) {
			/* done */
			return 0;
		}
		if (default_init_sequence[i] >= 0) {
			printf("missing delimiter at position %d\n", i);
			return -EINVAL;
		}
		if (default_init_sequence[i + 1] < 0) {
			printf("missing value after delimiter %d at position %d\n",
				default_init_sequence[i], i);
			return -EINVAL;
		}
		
		//printf("%d\n", __LINE__);
		switch (default_init_sequence[i]) {
		case -1:
			i++;
			/* make debug message */
			strcpy(msg, "");
			j = i + 1;
			while (default_init_sequence[j] >= 0) {
				sprintf(str, "0x%02X ", default_init_sequence[j]);
				strcat(msg, str);
				j++;
			}
			printf("init: write(0x%02X) %s\n",default_init_sequence[i], msg);

			/* Write */
			j = 0;
			while (default_init_sequence[i] >= 0) {
				if (j > 63) {
					printf("%s: Maximum register values exceeded\n",__func__);
					return -EINVAL;
				}
				buf[j++] = default_init_sequence[i++];
			}
			
			//printf("%d  %d\n", __LINE__,j);
			write_register(spi_buf, j,
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7],
				buf[8], buf[9], buf[10], buf[11],
				buf[12], buf[13], buf[14], buf[15],
				buf[16], buf[17], buf[18], buf[19],
				buf[20], buf[21], buf[22], buf[23],
				buf[24], buf[25], buf[26], buf[27],
				buf[28], buf[29], buf[30], buf[31],
				buf[32], buf[33], buf[34], buf[35],
				buf[36], buf[37], buf[38], buf[39],
				buf[40], buf[41], buf[42], buf[43],
				buf[44], buf[45], buf[46], buf[47],
				buf[48], buf[49], buf[50], buf[51],
				buf[52], buf[53], buf[54], buf[55],
				buf[56], buf[57], buf[58], buf[59],
				buf[60], buf[61], buf[62], buf[63]);
			break;
		case -2:
			i++;
			printf("init: mdelay(%d)\n", default_init_sequence[i]);
			usleep(1000*default_init_sequence[i++]);
			
			printf("%d\n", __LINE__);
			break;
		default:
			printf("unknown delimiter %d at position %d\n",
				default_init_sequence[i], i);
			
			printf("%d\n", __LINE__);
			return -EINVAL;
		}
	}

	printf("%s: something is wrong. Shouldn't get here.\n", __func__);
	return -EINVAL;
}
/*****************************************************************************
 *
 *   int (*write_vmem)(struct fbtft_par *par);
 *
 *****************************************************************************/

/* 16 bit pixel over 8-bit databus */
void fbtft_write_vmem16_bus8(size_t offset, size_t len)
{
	uint16_t *vmem16;
	size_t remain;
	size_t to_copy;
	size_t tx_array_size;
	int i;
	int ret = 0;
	size_t startbyte_size = 0;

	printf("%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	vmem16 = (uint16_t *)(spi_tft_buf + offset);
	usleep(1);
	u_gpio_set_value(SPI_GPIO_DC, 1);
	usleep(1);
	/* non buffered write */
	fbtft_write_spi(vmem16, len);
	

}

//ktime_t update_time;

static void fbtft_update_display(unsigned int start_line,
				 unsigned int end_line)
{
	size_t offset, len;
	//ktime_t ts_start, ts_end;
	long fps, throughput;
	//bool timeit = false;
	int ret = 0;
	unsigned int line_length; 
	
	//ts_start = ktime_get();
	//timeit = true;

	printf("%s:%d\n", __func__,__LINE__);
	
	line_length = TFT_WIDTH * TFT_BPP / 8;
	
	/* Sanity checks */
	if (start_line > end_line) {
		printf("%s: start_line=%u is larger than end_line=%u. Shouldn't happen, will do full display update\n",
			 __func__, start_line, end_line);
		start_line = 0;
		end_line = TFT_HEIGHT - 1;
	}
	if (start_line > TFT_HEIGHT- 1 ||
	    end_line > TFT_HEIGHT - 1) {
		printf("%s: start_line=%u or end_line=%u is larger than max=%d. Shouldn't happen, will do full display update\n",
			 __func__, start_line,
			 end_line, TFT_HEIGHT - 1);
		start_line = 0;
		end_line = TFT_HEIGHT - 1;
	}
	printf("%s:%d(start_line=%u, end_line=%u)\n", __func__,__LINE__,start_line,end_line);

	fbtft_set_addr_win(0, start_line,
				TFT_WIDTH, end_line);

	offset = start_line * line_length ;
	len = (end_line - start_line + 1) * line_length;
	
	printf("%s:%d(start_line=%u, end_line=%u)\n", __func__,__LINE__,offset,len);
	fbtft_write_vmem16_bus8(offset, len);
	if (ret < 0)
		printf("%s: write_vmem failed to update display buffer\n",
			__func__);

/*
	if (unlikely(timeit)) {
		ts_end = ktime_get();
		if (!ktime_to_ns(update_time))
			update_time = ts_start;

		fps = ktime_us_delta(ts_start, update_time);
		update_time = ts_start;
		fps = fps ? 1000000 / fps : 0;

		throughput = ktime_us_delta(ts_end, ts_start);
		throughput = throughput ? (len * 1000) / throughput : 0;
		throughput = throughput * 1000 / 1024;

		printf("Display update: %ld kB/s, fps=%ld\n",
			 throughput, fps);
	}
*/
}

#define RED    0xf800
#define GREEN  0x07e0
#define BLUE   0x001f
#define YELLOW 	0xffe0
#define WHITE  0xffff
#define BLACK  0x0000
#define PURPLE 0xf81f

void  WriteComm(uint8_t cmd)
{
//	uint8_t spi_cmd[1] = {0};

//	spi_cmd[0] = cmd;
//	fbtft_write_buf_dc(spi_cmd, 1, TFT_CMD);

	send_cmd(spi_fd,cmd);

}

void  WriteData(uint8_t data)
{
//	uint8_t spi_data[1] ={0};

//	spi_data[0] = data;
//	fbtft_write_buf_dc(spi_data, 1, TFT_DATA);
	send_data(spi_fd,data);


}
//========================================================
void LCD_Write_Data(uint16_t dat16)
{
	uint8_t  data_in;

	data_in = (uint8_t)(dat16>>8);
	WriteData(data_in);

	data_in = (uint8_t)dat16;
	WriteData(data_in);

}
//========================================================

void DISP_WINDOWS(void)
{
	 WriteComm(0x2A);
	 WriteData(0x00);
	 WriteData(0x00);
	 
	 WriteData(0x00);
	 WriteData(0xEF);

	 WriteComm(0x2B);
	 WriteData(0x00);
	 WriteData(0x00);
	 
	 WriteData(0x01);
	 WriteData(0x3F);
	 WriteComm(0x2C);
}


//========================================================
void DISPLAY_COLOR(uint16_t color)
{
    int i,j;
    DISP_WINDOWS();
    for (i=TFT_HEIGHT;i>0;i--)
    for (j=TFT_WIDTH; j>0;j--)
    LCD_Write_Data(color);
    //HOLD_DISP ();
}


void DISPLAY_RGB(void)
{
    int i,j,k;
	
		DISP_WINDOWS();
        for (i=100;i>0;i--)
        for (j=TFT_WIDTH;j>0;j--)
        {
    LCD_Write_Data(RED);
        }
    for (i=100;i>0;i--)
        for (j=TFT_WIDTH;j>0;j--)
        {
    LCD_Write_Data(GREEN);
        }
    for (k=120;k>0;k--)
        for (j=TFT_WIDTH;j>0;j--)
        {
    LCD_Write_Data(BLUE);
        }
    //HOLD_DISP ();
}
//========================================================
void Frame(void)
{
        int i,j,k;
        DISP_WINDOWS();
    for (i=TFT_WIDTH;i>0;i--)
        {
    LCD_Write_Data(WHITE);
        }
        for (j=TFT_WIDTH-2;j>0;j--)
        {
    LCD_Write_Data(WHITE);
    for (k=TFT_WIDTH-2;k>0;k--)
        {
    LCD_Write_Data(BLACK);
        }
        LCD_Write_Data(WHITE);
        }
        for (i=TFT_WIDTH;i>0;i--)
        {
    LCD_Write_Data(WHITE);
        }
        //HOLD_DISP ();
}
//========================================================


void DISPLAY_image(unsigned char* pimage)
{
        uint i;
        uint p=0;
        uint q=0;
        DISP_WINDOWS();
    for (i=TFT_WIDTH*TFT_HEIGHT*2;i>0;i--)
    {
    WriteData(pimage[p++]);
     }  
        //HOLD_DISP ();
}

void LCD_Init(void)
{
	//CS0=0;
	
	u_gpio_set_value(SPI_GPIO_RESET,1);
	usleep(1*1000);
	
	u_gpio_set_value(SPI_GPIO_RESET,0);
	usleep(10*1000);

	u_gpio_set_value(SPI_GPIO_RESET,1);
	usleep(120*1000);


	
WriteComm(0x11);
usleep(120*1000);

//************* Start Initial Sequence **********//
//-----------------------------------ST7789S reset sequence------------------------------------// 
//--------------------------------------Display Setting------------------------------------------//
WriteComm(0x36);
WriteData(0x00);
WriteComm(0x3a);
WriteData(0x55);
//--------------------------------ST7789S Frame rate setting----------------------------------//
WriteComm(0xb2);
WriteData(0x0c);
WriteData(0x0c);
WriteData(0x00);
WriteData(0x33);
WriteData(0x33);
WriteComm(0xb7);
WriteData(0x35);
//---------------------------------ST7789S Power setting--------------------------------------//
WriteComm(0xbb);
WriteData(0x2b);
WriteComm(0xc0);
WriteData(0x2c);
WriteComm(0xc2);
WriteData(0x01);
WriteComm(0xc3);
WriteData(0x11);
WriteComm(0xc4);
WriteData(0x20);
WriteComm(0xc6);
WriteData(0x0f);
WriteComm(0xd0);
WriteData(0xa4);
WriteData(0xa1);
//--------------------------------ST7789S gamma setting---------------------------------------//
WriteComm(0xe0);
WriteData(0xd0);
WriteData(0x00);
WriteData(0x05);
WriteData(0x0e);
WriteData(0x15);
WriteData(0x0d);
WriteData(0x37);
WriteData(0x43);
WriteData(0x47);
WriteData(0x09);
WriteData(0x15);
WriteData(0x12);
WriteData(0x16);
WriteData(0x19);
WriteComm(0xe1);
WriteData(0xd0);
WriteData(0x00);
WriteData(0x05);
WriteData(0x0d);
WriteData(0x0c);
WriteData(0x06);
WriteData(0x2d);
WriteData(0x44);
WriteData(0x40);
WriteData(0x0e);
WriteData(0x1c);
WriteData(0x18);
WriteData(0x16);
WriteData(0x19);
WriteComm(0x29);


}

void dislpay_tft_init(void)
{
	int ret = 0;
	spi_fd = open(device, O_RDWR);
		
	if (spi_fd < 0)
		pabort("can't open device");

	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

}

void  spi_tft_init(void)
{
	
	int ret = 0;
	uint32_t speed_t = 8000000;//19200000;//
	
	spi_fd = open(device, O_RDWR);
	if (spi_fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_t);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_t);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed_t, speed_t/1000);

	
	spi_user_gpio_init(SPI_GPIO_DC,SPI_GPIO_RESET,SPI_GPIO_PWM);

	LCD_Init();
	
	u_gpio_set_value(SPI_GPIO_PWM,1);
	//dislpay_tft_init();
//	while(1){
//		DISPLAY_COLOR(PURPLE);
//		usleep(500000);
//		DISPLAY_RGB();
//		
//		usleep(500000);
//		Frame();
//		
//	usleep(500000);
//	}
	//DISPLAY_image(picc1);
	close(spi_fd);

}
int   display_image(unsigned char*  image_data,int len)
{

	int ret = 0;
#if 1
	if(len == TFT_WIDTH*TFT_HEIGHT*2){
		
		DISP_WINDOWS();
	
		//fbtft_write_buf_dc(image_data,len,TFT_DATA);
		fbtft_write_vmem16_bus9(spi_fd,image_data,0,len);

		ret = 1;
	}
	else
		ret= -1;

		
#endif
//			DISPLAY_COLOR(PURPLE);
//			DISPLAY_RGB();	
//			Frame();

	return ret;
}
#if 0
int main(int argc, char *argv[])
{
	int ret = 0;
	int i = 0;
	char filepath[32] = {0};

	parse_opts(argc, argv);

	init();

	display_init();

    struct timeval tvl_s,tvl_e;
    struct tm * local_ts;
    struct tm * local_te;
/*	
    gettimeofday(&tvl_s, NULL);
    local_ts = localtime(&tvl_s.tv_sec);
	printf("\nstart:[%02d:%02d:%02d.%03d]\n\r",local_ts->tm_hour,local_ts->tm_min,local_ts->tm_sec,(int)(tvl_s.tv_usec/1000));

	for(i = 1;i <=1596;i++ ){
		
		sprintf(filepath, "./test/test%d",i);
		//printf("%s",filepath);
		if(file_exist(filepath)){
			
			//DISP_WINDOWS();
			
			readFile(filepath, spi_tft_buf, sizeof(spi_tft_buf));
			//fbtft_write_buf_dc(spi_tft_buf,sizeof(spi_tft_buf),1);
			display_image(spi_tft_buf, sizeof(spi_tft_buf));
			usleep(5000);
		}
	}

	printf("finished \n");
    gettimeofday(&tvl_e, NULL);
    local_te = localtime(&tvl_e.tv_sec);
	
	
	printf("\nend:[%02d:%02d:%02d.%03d]\n\r",local_te->tm_hour,local_te->tm_min,local_te->tm_sec,(int)(tvl_e.tv_usec/1000));
*/
	readFile("./456", spi_tft_buf, sizeof(spi_tft_buf));
	//fbtft_write_buf_dc(spi_tft_buf,sizeof(spi_tft_buf),1);
	display_image(spi_tft_buf, sizeof(spi_tft_buf));
	close(spi_fd);

	return ret;
}
#endif
