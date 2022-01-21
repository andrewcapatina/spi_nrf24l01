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

#define GPIO_CE 200
#define GPIO_IRQ 168

#define GPIO_EDGE_FALL "falling"
#define GPIO_EDGE_RISE "rising"
#define GPIO_LVL_HIGH 0x01
#define GPIO_LVL_LOW 0x00
#define GPIO_DIR_INPUT 0x00
#define GPIO_DIR_OUTPUT 0x01

#define SPI_SPEED 4000000
/*
 * Constants.
 * */

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (6 * 1000) // 6 seconds
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

	while(access(buf, W_OK) < 0);

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
	
	while(access(path, R_OK) < 0);

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

	while(access(buf, W_OK) < 0);

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
	while(access(buf, R_OK) < 0);

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
	while(access(buf, W_OK) < 0);

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
	while(access(buf, R_OK) < 0);
	
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
	
	while(access(buf, W_OK) < 0);

	fd = open(buf, O_WRONLY);
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

	while(access(buf, W_OK) < 0);


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
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	
	data_buffer[0] = addr;
	//printf("BUFF: %x\n", data_buffer[0]);
	for(int i = 1; i < len + 1; ++i)
	{
		data_buffer[i] = data[i-1];
		//printf("BUFF: %x\n", data_buffer[i]);
	}
	xfer.tx_buf = (unsigned long) data_buffer;
	xfer.rx_buf = (unsigned long) NULL;
	xfer.len = len + 1;
	xfer.bits_per_word = 8;
	xfer.speed_hz = SPI_SPEED;
	xfer.cs_change = 0;
	xfer.rx_nbits = 0;
	xfer.tx_nbits = (8 * len) + 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);

	return 0;

}

int spi_read_msg(int spi_dev_fd, char addr, char * copy_to, int len)
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
	xfer.speed_hz = SPI_SPEED;
	xfer.cs_change = 0;
	xfer.rx_nbits = len * 8;
	xfer.tx_nbits = 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);
	
	if(recv_buffer[0])
	{
		strcpy(copy_to, recv_buffer);
	}
	return 0;

}

int read_all_registers(int spi_dev_fd)
{
	char msg[64];

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | CONFIG, msg, 2);	
	printf("CONFIG: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, msg, 2);	
	printf("STATUS: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_AA, msg, 2);	
	printf("EN_AA: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_RXADDR, msg, 2);	
	printf("RX_ADDR: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_AW, msg, 2);	
	printf("SETUP_AW: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_RETR, msg, 2);	
	printf("SETUP_RETR: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_CH, msg, 2);	
	printf("RF_CH: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_SETUP, msg, 2);	
	printf("RF_SETUP: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, msg, 2);	
	printf("STATUS: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | OBSERVE_TX, msg, 2);	
	printf("OBSERVE_TX: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RPD, msg, 2);	
	printf("RPD: 0x%x \n", msg[1]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_ADDR_P0, msg, 6);	
	printf("RX_ADDR_P0: 0x%x %x %x %x %x \n", msg[5], msg[4], msg[3], msg[2], msg[1]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | TX_ADDR, msg, 6);	

	printf("TX_ADDR: 0x%x %x %x %x %x \n", msg[5], msg[4], msg[3], msg[2], msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_PW_P0, msg, 2);	
	printf("RX_PW_P0: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FIFO_STATUS, msg, 2);	
	printf("FIFO_STATUS: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | DYNPD, msg, 2);	
	printf("DYNPD: 0x%x \n", msg[1]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FEATURE, msg, 2);	
	printf("FEATURE: 0x%x \n", msg[1]);

	return 0;
}

int main() 
{
	struct pollfd pfd[2];

	unsigned int val;
	char rising[10] = "rising";

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint16_t delay;
	int speed = SPI_SPEED;

	// Setting (chip enable)
	gpio_export(GPIO_CE);
	//gpio_set_edge(GPIO_CE, rising); not needed since set to output.
	//printf("errno: %i\n", errno);
	
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set chip enable as output.

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);

	// Setting (IRQ)
	gpio_export(GPIO_IRQ);
	gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);
	gpio_set_active_edge(GPIO_IRQ);	// set to active low
	gpio_set_edge(GPIO_IRQ, rising);

	int gpio_irq_fd = gpio_get_value_fd((unsigned int) GPIO_IRQ);
	//pfd[1].fd = gpio_irq_fd;
	//pfd[1].events = POLLPRI;

	//pfd[0].fd = STDIN_FILENO;
	//pfd[0].events = POLLIN;

	// File object for SPI device.
	int spi_dev_fd = open(dev, O_RDWR);

	printf("fd: %i\n", spi_dev_fd);
	printf("errno: %i\n", errno);

	if(spi_dev_fd < 0)
		return 0;

	int rtn = ioctl(spi_dev_fd, SPI_IOC_WR_MODE, &mode);	// Set mode.

	if(rtn < 0)
	{
		printf("ioctl failed\n");
		return 1;
	}

	int lsb_setting = 0;
	ioctl(spi_dev_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	ioctl(spi_dev_fd, SPI_IOC_RD_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	// Set the TX_ADDR
	char msg[10];
	char addr;
	char snd_msg[10];

	msg[0] = 'R';	
	msg[1] = 'x';	
	msg[2] = 'A';	
	msg[3] = 'A';	
	msg[4] = 'A';	
	spi_send_msg(spi_dev_fd, W_REGISTER | TX_ADDR, msg, 5);
	
	msg[0] = 0xb6;	
	msg[1] = 0xb6;	
	msg[2] = 0xb6;	
	msg[3] = 0xb6;	
	msg[4] = 0xb6;	
	
	spi_send_msg(spi_dev_fd, W_REGISTER | RX_ADDR_P0, msg, 5);

	snd_msg[0] = 0x00;
	spi_send_msg(spi_dev_fd, W_REGISTER | EN_AA, snd_msg, 1);

	snd_msg[0] = 0x03;
	spi_send_msg(spi_dev_fd, W_REGISTER | SETUP_AW, snd_msg, 1);

	snd_msg[0] = 0x02;
	spi_send_msg(spi_dev_fd, W_REGISTER | RF_CH, snd_msg, 1);

	snd_msg[0] = 0x01;
	spi_send_msg(spi_dev_fd, W_REGISTER | EN_RXADDR, snd_msg, 1);

	snd_msg[0] = 0x0f;
	spi_send_msg(spi_dev_fd, W_REGISTER | SETUP_RETR, snd_msg, 1);

	snd_msg[0] = 0x01;
	spi_send_msg(spi_dev_fd, W_REGISTER | RF_SETUP, snd_msg, 1);

	snd_msg[0] = 32;
	spi_send_msg(spi_dev_fd, W_REGISTER | RX_PW_P0, snd_msg, 1);

	snd_msg[0] = FLUSH_TX;
	spi_send_msg(spi_dev_fd, FLUSH_TX, snd_msg, 1);


	addr = W_REGISTER | CONFIG;
	snd_msg[0] = PWR_UP;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);



	//char ch;

	//lseek(pfd[1].fd, 0, SEEK_SET);
	//read(pfd[1].fd, &ch, sizeof(ch));
	
	// set the payload TX_PLD

/*	int rc = poll(pfd, 2, POLL_TIMEOUT);

	if(pfd[1].revents & POLLPRI)
	{
		lseek(pfd[1].fd, 0, SEEK_SET);
		read(pfd[1].fd, &ch, sizeof(ch));
		printf("poll() GPIO interrupt: %x\n", ch);
	}
	if(pfd[0].revents & POLLIN)
	{
		lseek(pfd[0].fd, 0, SEEK_SET);
		read(pfd[0].fd, &ch, sizeof(ch));
		printf("poll() stdin read 0x%x\n", ch);

	}
*/

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);		
	char word[32];
	memset(word, 0, sizeof(word));
	word[0] = 'h';
	word[1] = 'e';
	word[2] = 'l';
	word[3] = 'l';
	word[4] = 'o';

	spi_send_msg(spi_dev_fd, W_TX_PAYLOAD_NO_ACK, word, 32);

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_HIGH);
	
	memset(snd_msg, 0, sizeof(snd_msg));
	do
	{
		spi_read_msg(spi_dev_fd, NOP, snd_msg, 1);
		snd_msg[0] = snd_msg[0] & (TX_DS | MAX_RT);
	} while(snd_msg[0] != TX_DS && snd_msg[0] != MAX_RT);

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);		

	//usleep(1000);
	char status[2];

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, status, 2);

	printf("status[0]: %x\n", status[0]);
	printf("status[1]: %x\n", status[1]);

	status[1] = status[1] & 0x20;
	if(status[1] == 0x20)
	{
		printf("sent!\n");
	}
	else 
	{
		snd_msg[0] = 0x10;
		spi_send_msg(spi_dev_fd, W_REGISTER | STATUS, snd_msg, 1);
	}	


	read_all_registers(spi_dev_fd);

//	usleep(100000);

	// Power down transceiver.
	snd_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, snd_msg, 1);

	close(gpio_irq_fd);
	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(GPIO_CE);
	gpio_unexport(GPIO_IRQ);

	return 0;
}

