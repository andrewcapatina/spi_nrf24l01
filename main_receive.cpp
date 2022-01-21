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
#include <cstring> 
#include <string>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "nrf24l01.h"

#include <iostream>

using namespace std;

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

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

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
		perror("gpio/set-value");
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

int spi_send_msg(int spi_dev_fd, char addr, char * data, int len)
{
	char data_buffer[len + 1];
	char recv_buffer;
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));
	
	data_buffer[0] = addr;
	//printf("BUFF addr: %x\n", data_buffer[0]);
	for(int i = 1; i < len + 1; ++i)
	{
		data_buffer[i] = data[i-1];
		//printf("BUFF: %x\n", data_buffer[i]);
	}
	xfer.tx_buf = (long long unsigned int) data_buffer;
	//xfer.rx_buf = (unsigned long) &recv_buffer;
	xfer.rx_buf = NULL;
	//xfer.len = len + 2;
	xfer.len = len + 1;
	xfer.bits_per_word = 8;
	xfer.speed_hz = 1000000;
	//xfer.cs_change = 0;
	//xfer.rx_nbits = 8;
	xfer.rx_nbits = 0;
	xfer.tx_nbits = (8 * len) + 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);

	return res;

}

int spi_read_msg(int spi_dev_fd, char addr, char * status, char * copy_to, int len)
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
	xfer.rx_nbits = len * 8;
	xfer.tx_nbits = 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);
	
	if(res > 0)
	{
		status[0] = recv_buffer[0];
		string temp = string(recv_buffer);
		temp = temp.substr(1);
		sprintf(copy_to, "%s", temp.c_str());
	}
	


	return res;

}


void read_all_registers(int spi_dev_fd)
{
	char msg[10];
	char status;

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | CONFIG, &status, msg, 2);	
	printf("CONFIG: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, &status, msg, 2);	
	printf("STATUS: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_AA, &status, msg, 2);	
	printf("EN_AA: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_RXADDR, &status, msg, 2);	
	printf("RX_ADDR: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_AW, &status, msg, 2);	
	printf("SETUP_AW: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_RETR, &status, msg, 2);	
	printf("SETUP_RETR: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_CH, &status, msg, 2);	
	printf("RF_CH: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_SETUP, &status, msg, 2);	
	printf("RF_SETUP: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, &status, msg, 2);	
	printf("STATUS: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | OBSERVE_TX, &status, msg, 2);	
	printf("OBSERVE_TX: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RPD, &status, msg, 2);	
	printf("RPD: 0x%x \n", msg[0]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_ADDR_P0, &status, msg, 6);	
	printf("RX_ADDR_P0: 0x%x %x %x %x %x \n", msg[4], msg[3], msg[2], msg[1], msg[0]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | TX_ADDR, &status, msg, 6);	

	printf("TX_ADDR: 0x%x %x %x %x %x \n", msg[4], msg[3], msg[2], msg[1], msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_PW_P0, &status, msg, 2);	
	printf("RX_PW_P0: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FIFO_STATUS, &status, msg, 2);	
	printf("FIFO_STATUS: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | DYNPD, &status, msg, 2);	
	printf("DYNPD: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FEATURE, &status, msg, 2);	
	printf("FEATURE: 0x%x \n", msg[0]);



	return;
}

int main() 
{
	unsigned int val;
	char rising[7] = GPIO_EDGE_RISE;

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000;
	int lsb_setting = 0;
	uint16_t delay;
	
	// Setting (chip enable)
	gpio_export(GPIO_CE);
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set gpio 4 as output.

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

	ioctl(spi_dev_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	ioctl(spi_dev_fd, SPI_IOC_RD_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	char addr;
	char shutdown_msg[2];
	char snd_msg[3];
	
	shutdown_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, shutdown_msg, 1);

	usleep(10000);


	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);	// Setting chip enable to 1.

	int rtn;

	addr = W_REGISTER | EN_AA;
	snd_msg[0] = 0x01;	// Turn off auto acknowledgement.
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | SETUP_AW;
	snd_msg[0] = 0x03;	// 5 byte address width.
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
	
	addr = W_REGISTER | RF_CH;
	snd_msg[0] = 0x02;	// channel 76.
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | EN_RXADDR;
	snd_msg[0] = 0x01;	// Enable data pipe 0.
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RF_SETUP;
	//snd_msg[0] = 0x24;
	snd_msg[0] = 0x01;	// 1Mbps, -12dBm power output. 
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | RX_PW_P0;
	snd_msg[0] = 32;	// 32 byte payload
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
	
	addr = W_REGISTER | DYNPD;
	snd_msg[0] = 0x01;	 
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	addr = W_REGISTER | FEATURE;
	snd_msg[0] = 0x06;	 
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);


	addr = FLUSH_RX;
	snd_msg[0] = FLUSH_RX;
	rtn = spi_send_msg(spi_dev_fd, addr, snd_msg, 1);
	
	addr = W_REGISTER | CONFIG;
	snd_msg[0] = EN_CRC | PWR_UP | PRIM_RX;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	char msg_addr[5];
	msg_addr[0] = 0xb6;	
	msg_addr[1] = 0xb6;	
	msg_addr[2] = 0xb6;	
	msg_addr[3] = 0xb6;	
	msg_addr[4] = 0xb6;	

	addr = W_REGISTER | TX_ADDR;
	//spi_send_msg(spi_dev_fd, addr, msg_addr, 5);


	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_HIGH);	// Setting chip enable to 1.

	msg_addr[0] = 'R';	
	msg_addr[1] = 'x';	
	msg_addr[2] = 'A';	
	msg_addr[3] = 'A';	
	msg_addr[4] = 'A';	

	addr = W_REGISTER | RX_ADDR_P0;
	spi_send_msg(spi_dev_fd, addr, msg_addr, 5);

	val = 0;

	gpio_get_value((unsigned int) GPIO_CE, &val);

	printf("val: %i \n", val);

	snd_msg[0] = 0xf0;
	spi_send_msg(spi_dev_fd, W_REGISTER | STATUS, snd_msg, 1);
		
	read_all_registers(spi_dev_fd);

	bool recv = false;
	bool detected = false;
	char status;
	do
	{
		memset(snd_msg, 0, sizeof(snd_msg));

		spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, &status, snd_msg, 2);
		
		if((status & RX_DR) > 0)
		{

			status = (status >> RX_P_NO) & 0x07;
			
			if(status >= 0 && status <= 5)
			{
				char recv_msg[64];
				char size[2];
				memset(recv_msg, 0, sizeof(recv_msg));

				spi_read_msg(spi_dev_fd, R_RX_PL_WID, &status, size, 2);

				spi_read_msg(spi_dev_fd, R_RX_PAYLOAD , &status, recv_msg, (int) size[1]);
				if(recv_msg)
				{
					printf("recv_msg: %s \n", recv_msg);
				}

				recv = true;
				gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);	// Setting chip enable to 0.
			}

			snd_msg[0] = 0xf0;
			spi_send_msg(spi_dev_fd, W_REGISTER | STATUS, snd_msg, 1);
		}

		spi_read_msg(spi_dev_fd, R_REGISTER | RPD, &status, snd_msg, 2);
		if(snd_msg[0] > 0x00 && detected == false)
		{
			detected = true;
			printf("carrier detected\n");
	
			read_all_registers(spi_dev_fd);
		}

		spi_read_msg(spi_dev_fd, R_REGISTER | FIFO_STATUS, &status, snd_msg, 2);
		if((snd_msg[0] & 0x02) == 0x02)
		{
				char recv_msg[32];
				memset(recv_msg, 0, sizeof(recv_msg));
				spi_read_msg(spi_dev_fd, 0x00 | R_RX_PAYLOAD , &status, recv_msg, 5);
				if(recv_msg)
				{
					printf("recv_msg: %x \n", recv_msg[1]);
				}

				recv = true;
		}

	}while(recv == false);

	// Power down transceiver.
	shutdown_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, shutdown_msg, 1);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(GPIO_CE);
	gpio_unexport(GPIO_IRQ);
	// echo 4 > /sys/class/gpio/unexport

	return 0;
}
