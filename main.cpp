/**
 * 	Andrew Capatina 
 * 	1/5/2022
 *	File to set gpios and interact through 
 *	SPI with the nrf24l01 wireless transceiver.
 *
 *	GPIO functions taken from ridgerun.
 *	
 * */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "nrf24l01.h"

#define GPIO_CE 4 
#define GPIO_IRQ 27
#define GPIO_EDGE_FALL "falling"
#define GPIO_EDGE_RISE "rising"
#define GPIO_LVL_HIGH 0x01
#define GPIO_LVL_LOW 0x00
#define GPIO_DIR_INPUT 0x00
#define GPIO_DIR_OUTPUT 0x01
/*
 * Constants.
 * */

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (10 * 1000) // 2 seconds
#define MAX_BUF 64

using namespace std;

/**
 *	Function to bring a selected 
 *	GPIO into userspace.
 * */
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);

	if(fd < 0)
	{
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;

}

/**
 *	Function to bring a GPIO back into 
 *	the kernel space settings.
 * */
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/unexport");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/**
 *	Function to set GPIO direction.
 *
 *	out_flag = 1: set as output.
 *	out_flag = 0: set as input.
 *
 * */
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/direction");
		return fd;
	}

	if(out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	close(fd);

	return 0;
}

/**
 *	Returns file descriptor for gpio number 
 *	value.
 * */
int gpio_get_value_fd(unsigned int gpio)
{
	int fd;
	char path[32];

	sprintf(path, SYSFS_GPIO_DIR "/gpio%i/value", gpio);

	fd = open(path, O_RDONLY);
	if(fd < 0)
	{
		perror("failed to open gpio fd");
		exit(1);
	}

	return fd;
}

/**
 *	Set value of a GPIO pin.
 *
 * */
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/set-value");
		return 0;
	}

	if(value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);

	close(fd);

	return 0;
}

int gpio_get_active_edge(unsigned int gpio, unsigned int * value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/active_low", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK);
	if(fd < 0)
	{
		perror("gpio/active_low");
		return 0;
	}
	read(fd, &ch, 1);

	if(ch == '0')
		*value = 0;
	else 
		*value = 1;
	

	close(fd);

	return 0;
}


int gpio_set_active_edge(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/active_low", gpio);

	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/active_low");
		return 0;
	}

	write(fd, "1", 2);	// Inverts value attribute for both 
				// reading and writing.
	
	close(fd);

	return 0;
}

/**
 *	Gets value of a GPIO pin.
 * */
int gpio_get_value(unsigned int gpio, unsigned int * value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/value", gpio);
	
	fd = open(buf, O_RDONLY);
	if(fd < 0)
	{
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if(ch != '0')
		*value = 1;
	else
		*value = 0;

	close(fd);

	return 0;
}

int gpio_set_value(unsigned int gpio, char * value)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/value", gpio);
	
	fd = open(buf, O_RDONLY);
	if(fd < 0)
	{
		perror("gpio/get-value");
		return fd;
	}

	write(fd, value, strlen(value) + 1);
	close(fd);

	return 0;
}

/**
 *	Set the GPIO active edge.
 *	
 * */
int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/edge");
		return fd;
	}

	write(fd, edge, strlen(edge) + 1);
	close(fd);

	return 0;
}

/**
 *	Open the GPIO file for reading its 
 *	current value.
 *
 * */
int gpio_fd_open(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR
			"/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK);
	if(fd < 0)
		perror("gpio/fd_open");

	return fd;

}

/**
 * Close the file descriptor.
 * */
int gpio_fd_close(int fd)
{
	return close(fd);
}
/*
int spi_read_msg(int spi_dev_fd, char addr, char * copy_to, int len)
{
	char data_buffer;
	char recv_buffer[len];
	struct spi_ioc_transfer xfer[2];	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));

	data_buffer = addr;
	xfer[0].tx_buf = (unsigned long) &data_buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = 1;
	xfer[0].bits_per_word = 8;
	xfer[0].speed_hz = 1000000;
	xfer[0].cs_change = 1;
	
	xfer[1].rx_buf = (unsigned long) &recv_buffer;
	xfer[1].tx_buf = 0;
	xfer[1].len = len;
	xfer[1].bits_per_word = 8;
	xfer[1].speed_hz = 1000000;
	xfer[1].cs_change = 0;
	xfer[1].rx_nbits = len * 8;	// EXPERIMENT WITH THIS

	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(2), xfer);
	printf("errno: %i \n", errno);

	if(recv_buffer[0])
	{
		strcpy(copy_to, recv_buffer);
	}

	return 0;

}
int spi_send_msg(int spi_dev_fd, char addr, char * data, int len)
{
	char data_buffer[len + 1];
	char recv_buffer;
	struct spi_ioc_transfer xfer[2];

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));
	
	// Copy data to send
	data_buffer[0] = addr;
	printf("BUFF: %x\n", data_buffer[0]);
	for(int i = 1; i < len + 1; ++i)
	{
		data_buffer[i] = data[i-1];
		printf("BUFF: %x\n", data_buffer[i]);
	}

	xfer[0].tx_buf = (unsigned long) &data_buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = len + 1;
	xfer[0].bits_per_word = 8;
	xfer[0].speed_hz = 1000000;
	xfer[0].cs_change = 1;
	
	xfer[1].rx_buf = (unsigned long) &recv_buffer;
	xfer[1].tx_buf = 0;
	xfer[1].len = 1;
	xfer[1].bits_per_word = 8;
	xfer[1].speed_hz = 1000000;
	xfer[1].cs_change = 0;

	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(2), xfer);



	return 0;
}
*/

int test_send(int spi_dev_fd, char addr, char * data, int len)
{
	char data_buffer[len + 1];
	char recv_buffer;
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));
	
	data_buffer[0] = addr;
	printf("BUFF: %x\n", data_buffer[0]);
	for(int i = 1; i < len + 1; ++i)
	{
		data_buffer[i] = data[i-1];
		printf("BUFF: %x\n", data_buffer[i]);
	}
	xfer.tx_buf = (unsigned long) data_buffer;
	xfer.rx_buf = (unsigned long) &recv_buffer;
	xfer.len = len + 2;
	xfer.bits_per_word = 8;
	xfer.speed_hz = 1000000;
	xfer.cs_change = 0;
	xfer.rx_nbits = 8;	// EXPERIMENT WITH THIS
	xfer.tx_nbits = 8 * len + 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);

	return 0;

}

int test_read(int spi_dev_fd, char addr, char * copy_to, int len)
{
	char data_buffer;
	char recv_buffer[len];
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));

	data_buffer = addr;
	xfer.tx_buf = (unsigned long) &data_buffer;
	xfer.rx_buf = (unsigned long) recv_buffer;
	xfer.len = len + 1;
	xfer.bits_per_word = 8;
	xfer.speed_hz = 1000000;
	xfer.cs_change = 0;
	xfer.rx_nbits = len * 8;	// EXPERIMENT WITH THIS
	xfer.tx_nbits = 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);
	
	if(recv_buffer[0])
	{
		strcpy(copy_to, recv_buffer);
	}
	return 0;

}

int main() 
{
	unsigned int val;
	char rising[7] = GPIO_EDGE_RISE;

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000;
	uint16_t delay;

	// Setting (chip enable)
	gpio_export(GPIO_CE);
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set gpio 4 as output.
	//gpio_set_edge(GPIO_CE, rising);

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);

	// Setting (IRQ)
	gpio_export(GPIO_IRQ);
	gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);
	gpio_set_edge(GPIO_IRQ, rising);
	gpio_set_active_edge(GPIO_IRQ);	// set to active low

	// File object for SPI device.
	int spi_dev_fd = open(dev, O_RDWR);

	printf("fd: %i\n", spi_dev_fd);
	printf("errno: %i \n", errno);

	if(errno)
		return 0;

	ioctl(spi_dev_fd, SPI_IOC_WR_MODE, &mode);	// Set mode.

	int lsb_setting = 0;
	ioctl(spi_dev_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	// Wait 100ms for power on reset.
	usleep(100000);

	char addr;
	int len = 1;
	char snd_msg[len];
	memset(snd_msg, 0, sizeof(snd_msg));

	// Power up transceiver.
	addr = W_REGISTER | CONFIG;
	snd_msg[0] = PWR_UP;
	//spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
	test_send(spi_dev_fd, addr, snd_msg, 1);

	// Waiting 1.5ms.
	usleep(2000);


	len = 3;
	char msg[len];
	memset(msg, 0, sizeof(msg));

	//spi_read_msg(spi_dev_fd, FIFO_STATUS, msg, 2);
	test_read(spi_dev_fd, FIFO_STATUS, msg, 2);

	if(msg[0])
		printf("msg0: %x \n", msg[0]);
	if(msg[1])
		printf("msg1: %x \n", msg[1]);


	memset(msg, 0, sizeof(msg));

	//spi_read_msg(spi_dev_fd, STATUS, msg, 1);
	test_read(spi_dev_fd, STATUS, msg, 2);

	if(msg[0])
		printf("msg0: %x \n", msg[0]);
	if(msg[1])
		printf("msg1: %x \n", msg[1]);


/*
	char addr;
	int len = 1;
	char snd_msg[len];
	memset(snd_msg, 0, sizeof(snd_msg));

	// Power up transceiver.
	addr = W_REGISTER | CONFIG;
	snd_msg[0] = PWR_UP;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	// Waiting 1.5ms.
	usleep(1500);

	// Set PRIM_RX bit to enter RX mode.
	//
	//gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_HIGH);	// Setting chip enable to 1.
	addr = W_REGISTER | CONFIG;
	snd_msg[0] = PWR_UP | PRIM_RX;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	// Enabling data pipes.
	addr = W_REGISTER | EN_RXADDR;
	snd_msg[0] = 0x00 | ERX_P0;
//	snd_msg[0] = 0x00 | ERX_P5 | ERX_P4 | ERX_P3 | 
//		ERX_P2 | ERX_P1 | ERX_P0;	
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	// Enabling data pipe auto acknowledge.
	addr = W_REGISTER | EN_AA;	
	snd_msg[0] = 0x00 | ENAA_P0;	

//	snd_msg[0] = 0x00 | ENAA_P5 | ENAA_P4 | ENAA_P3 |
//		ENAA_P2 | ENAA_P1 | ENAA_P0;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	// Set the payload widths.
	addr = W_REGISTER | RX_PW_P0;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
*/
/*
	addr = W_REGISTER | RX_PW_P1;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RX_PW_P2;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RX_PW_P3;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RX_PW_P4;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RX_PW_P5;
	snd_msg[0] = RX_WIDTH;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
*/
/*	
	// Set up receive addresses.
	char msg_addr[5];
	msg_addr[0] = 0xE7;	
	msg_addr[1] = 0xE7;	
	msg_addr[2] = 0xE7;	
	msg_addr[3] = 0xE7;	
	msg_addr[4] = 0xE7;	

	addr = W_REGISTER | RX_ADDR_P0;
	spi_send_msg(spi_dev_fd, addr, msg_addr, 5);

	// Set GPIO CE high to start active RX mode.
	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_HIGH);	// Setting chip enable to 1.

	// Wait 130us to enter the mode.
//	usleep(130);

	char ch;
	struct pollfd pfd;
	int gpio_fd = gpio_get_value_fd((unsigned int) GPIO_IRQ);

	pfd.fd = gpio_fd;
	pfd.events = POLLPRI;

	lseek(gpio_fd, 0, SEEK_SET);	// Handle any current interrupts.
	read(gpio_fd, &ch, sizeof(ch));
	ch = 0x01;

	poll(&pfd, 1, POLL_TIMEOUT);

	lseek(gpio_fd, 0, SEEK_SET);
	read(gpio_fd, &ch, sizeof(ch));

	printf("GPIO_IRQ: %x \n", ch);	

	if(ch == 0x00)
	{
		// Get payload 
		char recv_msg[32];
		spi_read_msg(spi_dev_fd, 0x00 | R_RX_PAYLOAD , recv_msg, 32);
		if(recv_msg[0])
		{
			printf("recv_msg: %x \n", recv_msg[0]);
		}

	}


	

	// Power down transceiver.
	snd_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);
*/
	// Bring back default settings for GPIO. 
	gpio_unexport(4);
	gpio_unexport(27);

	return 0;
}
