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

/*
 * Constants.
 * */

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) // 3 seconds
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
	xfer[0].cs_change = 0;
	
	xfer[1].rx_buf = (unsigned long) &recv_buffer;
	xfer[1].tx_buf = 0;
	xfer[1].len = len;
	xfer[1].bits_per_word = 8;
	xfer[1].speed_hz = 1000000;
	xfer[1].cs_change = 0;

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
	xfer[0].cs_change = 0;
	
	xfer[1].rx_buf = (unsigned long) &recv_buffer;
	xfer[1].tx_buf = 0;
	xfer[1].len = 1;
	xfer[1].bits_per_word = 8;
	xfer[1].speed_hz = 1000000;
	xfer[1].cs_change = 0;

	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(2), xfer);



	return 0;
}

int main() 
{
	unsigned int val;
	char rising[7] = "rising";

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000;
	uint16_t delay;

	// Setting (chip enable)
	gpio_export(4);
	gpio_set_dir(4, 1);	// set gpio 200 as output.
	gpio_set_edge(4, rising);
	gpio_set_active_edge(4);	// set to active low

	char level = 0x00;
	gpio_set_value(4, level);

	// Setting (IRQ)
	gpio_export(27);
	gpio_set_dir(27, 0);
	gpio_set_edge(27, rising);

	// File object for SPI device.
	int spi_dev_fd = open(dev, O_RDWR);

	printf("fd: %i\n", spi_dev_fd);
	printf("%i \n", errno);

	if(errno)
		return 0;

	ioctl(spi_dev_fd, SPI_IOC_WR_MODE, &mode);	// Set mode.

	int lsb_setting = 0;
	ioctl(spi_dev_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	int len = 1;
	char snd_msg[len];
	memset(snd_msg, 0, sizeof(snd_msg));

	// Power up transceiver.
	snd_msg[0] = 0x02;
	spi_send_msg(spi_dev_fd, 0x20, snd_msg, 1);

	usleep(1500);

	len = 1;
	char msg[len];
	memset(msg, 0, sizeof(msg));

	//spi_read_msg(spi_dev_fd, 0x07, msg, len);
	spi_read_msg(spi_dev_fd, 0x17, msg, len);
	//spi_read_msg(spi_dev_fd, 0x08, msg, len);

	if(msg[0])
		printf("0: %x \n", msg[0]);

	//if(msg[1])
	//	printf("1: %x \n", msg[1]);
/*	if(msg[2])
		printf("2: %i \n", msg[2]);
	if(msg[3])
		printf("3: %i \n", msg[3]);
	if(msg[4])
		printf("4: %i \n", msg[4]);
	if(msg[5])
		printf("5: %i \n", msg[5]);

	*/
	// Power down transceiver.
	snd_msg[0] = 0x00;
	spi_send_msg(spi_dev_fd, 0x20, snd_msg, 1);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(4);
	gpio_unexport(27);

	return 0;
}
