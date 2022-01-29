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

#define GPIO_CE 200 
#define GPIO_IRQ 168
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

#define NUM_PAYLOAD_BYTES 30

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

int spi_init(const char * dev, int mode, int bits, int speed, int lsb_setting)
{
	// File object for SPI device.
	int spi_dev_fd = open(dev, O_RDWR);

	ioctl(spi_dev_fd, SPI_IOC_WR_MODE, &mode);	// Set mode.

	ioctl(spi_dev_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.

	ioctl(spi_dev_fd, SPI_IOC_RD_LSB_FIRST, &lsb_setting);	// MSB first.

	ioctl(spi_dev_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);	// Number of bits per word.

	ioctl(spi_dev_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);	// Set MHz for transfer.


	return spi_dev_fd;
}

int spi_send_msg(int spi_dev_fd, char addr, char * data, int len)
{
	char data_buffer[len + 1];
	char recv_buffer;
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));
	
	data_buffer[0] = addr;
	for(int i = 1; i < len + 1; ++i)
	{
		data_buffer[i] = data[i-1];
	}
	xfer.tx_buf = (unsigned long) data_buffer;
	xfer.rx_buf = (unsigned long) NULL;
	xfer.len = len + 1;
	xfer.bits_per_word = 8;
	xfer.speed_hz = 1000000;
	xfer.cs_change = 0;
	//xfer.rx_nbits = 8;
	xfer.rx_nbits = 0;
	xfer.tx_nbits = (8 * len) + 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);

	return res;

}

int spi_read_msg(int spi_dev_fd, char addr, char * status, char * copy_to, int len)
{
	char data_buffer;
	char recv_buffer[len + 1];
	struct spi_ioc_transfer xfer;	

	memset(&xfer, 0, sizeof(xfer));
	memset(&recv_buffer, 0, sizeof(recv_buffer));

	data_buffer = addr;
	xfer.tx_buf = (unsigned long) &data_buffer;
	xfer.rx_buf = (unsigned long) recv_buffer;
	xfer.len = len + 2;
	xfer.bits_per_word = 8;
	xfer.speed_hz = 1000000;
	xfer.cs_change = 0;
	xfer.rx_nbits = len * 8;
	xfer.tx_nbits = 8;
	
	int res = ioctl(spi_dev_fd, SPI_IOC_MESSAGE(1), xfer);
	
	if(res > 0)
	{
		status[0] = recv_buffer[0];
		if(copy_to != NULL)
		{
			for(int i = 0; i < len; ++i)
			{
				copy_to[i] = recv_buffer[i+1];
			}
		}
	}

	return res;

}


void nrf_print_all_registers(int spi_dev_fd)
{
	char msg[10];
	char status;

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | CONFIG, &status, msg, 1);	
	printf("CONFIG: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | STATUS, &status, msg, 1);	
	printf("STATUS: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_AA, &status, msg, 1);	
	printf("EN_AA: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | EN_RXADDR, &status, msg, 1);	
	printf("RX_ADDR: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_AW, &status, msg, 1);	
	printf("SETUP_AW: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | SETUP_RETR, &status, msg, 1);	
	printf("SETUP_RETR: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_CH, &status, msg, 1);	
	printf("RF_CH: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RF_SETUP, &status, msg, 1);	
	printf("RF_SETUP: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | OBSERVE_TX, &status, msg, 1);	
	printf("OBSERVE_TX: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RPD, &status, msg, 1);	
	printf("RPD: 0x%x \n", msg[0]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_ADDR_P0, &status, msg, 5);	
	printf("RX_ADDR_P0: 0x%x %x %x %x %x \n", msg[4], msg[3], msg[2], msg[1], msg[0]);

	memset(msg, 0, sizeof(msg));

	spi_read_msg(spi_dev_fd, R_REGISTER | TX_ADDR, &status, msg, 5);	

	printf("TX_ADDR: 0x%x %x %x %x %x \n", msg[4], msg[3], msg[2], msg[1], msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | RX_PW_P0, &status, msg, 1);	
	printf("RX_PW_P0: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FIFO_STATUS, &status, msg, 1);	
	printf("FIFO_STATUS: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | DYNPD, &status, msg, 1);	
	printf("DYNPD: 0x%x \n", msg[0]);

	spi_read_msg(spi_dev_fd, R_REGISTER | FEATURE, &status, msg, 1);	
	printf("FEATURE: 0x%x \n", msg[0]);



	return;
}

void nrf_write_reg_byte(int spi_dev_fd, char reg, char value)
{
	int rtn;
	char addr;

	addr = W_REGISTER | reg;
	value = value;	
	rtn = spi_send_msg(spi_dev_fd, addr, &value, 1);

	return;
}

void nrf_write_reg_multi_byte(int spi_dev_fd, char reg, char * value, int len)
{
	int rtn;
	char addr;

	addr = W_REGISTER | reg;
	rtn = spi_send_msg(spi_dev_fd, addr, value, len);

	return;
}

void nrf_write_command_byte(int spi_dev_fd, char cmd, char value)
{
	int rtn;
	char addr;

	addr = cmd;
	rtn = spi_send_msg(spi_dev_fd, addr, &value, 1);

	return;
}

void nrf_write_command_multi_byte(int spi_dev_fd, char cmd, char * value, int len)
{
	int rtn;
	char addr;

	addr = cmd;	
	rtn = spi_send_msg(spi_dev_fd, addr, value, len);

	return;
}

/**
 *	Initializes nrf registers 
 *	for transmit mode.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * */
void nrf_tx_init(int spi_dev_fd)
{
	char reg, value;
	char value_mb[5];

	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);

        value_mb[0] = 'R';
        value_mb[1] = 'x';
        value_mb[2] = 'A';
        value_mb[3] = 'A';
        value_mb[4] = 'A';	

	reg = TX_ADDR;
	nrf_write_reg_multi_byte(spi_dev_fd, reg, value_mb, 5);

	value = 0x3f;
	reg = EN_AA;
	nrf_write_reg_byte(spi_dev_fd, reg, value);
	
	value = 0x03;
	reg = SETUP_AW;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = 0x4c;
	reg = RF_CH;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = 0x01;
	reg = EN_RXADDR;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = 0xff;
	reg = SETUP_RETR;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	//value = 0x01;
	value = 0x22;
	reg = RF_SETUP;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = NUM_PAYLOAD_BYTES;
	reg = RX_PW_P0;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = 0x04;
	reg = FEATURE;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	value = 0x03;
	reg = DYNPD;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	reg = FLUSH_RX;
	value = FLUSH_RX;
	nrf_write_command_byte(spi_dev_fd, reg, value);

	reg = FLUSH_TX;
	value = FLUSH_TX;
	nrf_write_command_byte(spi_dev_fd, reg, value);

	reg = STATUS;
	value = RX_DR | TX_DS | MAX_RT;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	reg = CONFIG;
	value = EN_CRC | CRCO | PWR_UP;
	nrf_write_reg_byte(spi_dev_fd, reg, value);
	
	return;
}

int nrf_set_rx_address(int spi_dev_fd, char * addr, int pipe)
{
	char reg;

	if(pipe > 5)
		return 1;

	reg = RX_ADDR_P0 << pipe;
	nrf_write_reg_multi_byte(spi_dev_fd, reg, addr, 5);

	return 0;
}

/**
 *	Set the transmit address of the nrf.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * 	addr: tx address
 * */
int nrf_set_tx_address(int spi_dev_fd, char * addr)
{
	char reg;

	reg = TX_ADDR;
	nrf_write_reg_multi_byte(spi_dev_fd, reg, addr, 5);

	return 0;
}

bool nrf_rx_pipe_available(int spi_dev_fd, int * pipe)
{
	
	char addr = NOP;
	char status;
	spi_read_msg(spi_dev_fd, addr, &status, NULL, 0);
		
	if((status & RX_DR) > 0)
	{
		*pipe = (status >> RX_P_NO) & 0x07;

		if(*pipe > 5)
		{
			return 1;
		}

		return 0;
	}

	return 1;
}

void nrf_shutdown(int spi_dev_fd)
{
	char reg, value;

	value = 0x00;	// Power down transceiver.
	reg = CONFIG;
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	return;
}

/**	
 *	Adds new payload to the transceiver.
 *
 *	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_tx_new_payload(int spi_dev_fd, char * payload, int len)
{

	char reg = W_TX_PAYLOAD;

	nrf_write_command_multi_byte(spi_dev_fd, reg, payload, len);

	return 0;
}

/**
 *	Checks if packet has been sent.
 *
 * 	if 0 is returned: send pending.
 *	If 1 is returned: TX_DS set.
 *	If 2 is returned: MAX_RT set.
 *
 *	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_tx_pending_send(int spi_dev_fd)
{
	char addr, code, msg, status;

	addr = R_REGISTER | STATUS;
	spi_read_msg(spi_dev_fd, addr, &status, NULL, 0);

	code = status & TX_DS;
	if(code == TX_DS)
	{
	//	addr = W_REGISTER | STATUS;
	//	msg = TX_DS | MAX_RT;
	//	spi_send_msg(spi_dev_fd, addr, &msg, 1);
		return 1;

	}

	code = status & MAX_RT;
	if(code == MAX_RT)
	{
		//addr = W_REGISTER | STATUS;
		//msg = MAX_RT;
		//spi_send_msg(spi_dev_fd, addr, &msg, 1);
		return 2;
	}
	

	return 0;
}

/**
 *	Function to load a payload and send a packet.
 *
 *
 *	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_tx_send_packet(int spi_dev_fd, char * payload, int len)
{
	int rtn; 
	// Put low so we can add the payload.
	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);
	// Set a new payload.
	nrf_tx_new_payload(spi_dev_fd, payload, len);

	/*
	// TODO: debug code
	char status;
	spi_read_msg(spi_dev_fd, W_TX_PAYLOAD, &status, payload, NUM_PAYLOAD_BYTES); 
	for(int i = 0; i < len; ++i)
	{
		printf("i: %x \n", payload[i]);
	}
	// end debug code.
*/

	// Start tx transmission.
	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_HIGH);


	do
	{
		rtn = nrf_tx_pending_send(spi_dev_fd);

		if(rtn == 2)
		{	
			char clr = MAX_RT;
			spi_send_msg(spi_dev_fd, W_REGISTER | STATUS, &clr, 1);
		}
	}while(rtn != 1);

	// Go back to standby mode
	gpio_set_value((unsigned int) GPIO_CE, (unsigned int) GPIO_LVL_LOW);	// Setting chip enable to 0.

	char reg = W_REGISTER | STATUS;
	char val = RX_DR | TX_DS | MAX_RT;
	spi_send_msg(spi_dev_fd, reg, &val, 1);

	return 0;
}

void send_files(int spi_dev_fd, FILE * fp)
{
	int num_bytes=0, read_size, rtn, size;
	char payload[NUM_PAYLOAD_BYTES + 1] = {0};

	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	printf("file size: %i \n", size);

	strcpy(payload, "SENDING_IMAGE");

	nrf_tx_send_packet(spi_dev_fd, payload, NUM_PAYLOAD_BYTES);

	memset(payload, 0, sizeof(payload));

	sprintf(payload, "%i", size);

	nrf_tx_send_packet(spi_dev_fd, payload, NUM_PAYLOAD_BYTES);

	rewind(fp);
	memset(payload, 0, sizeof(payload));
	while(!feof(fp))
	{
		memset(payload, 0, sizeof(payload));
		read_size = fread(payload, sizeof(char), NUM_PAYLOAD_BYTES, fp);

		rtn = nrf_tx_send_packet(spi_dev_fd, payload, read_size);

		num_bytes += read_size;

		//printf("num_bytes: %i \n", num_bytes);
		for(int i = 0; i < read_size; ++i)
		{
			printf("%x ", payload[i]);
		}
		printf("\n");
	}

}

int nrf_rx_mode(int spi_dev_fd)
{
	char reg, value;

	reg = STATUS;
	value = EN_CRC | CRCO | PWR_UP | PRIM_RX;	// RX mode
	nrf_write_reg_byte(spi_dev_fd, reg, value);

	return 0;
}

int nrf_tx_mode(int spi_dev_fd)
{
	char reg, value;

	reg = STATUS;
	value = EN_CRC | CRCO | PWR_UP;	// TX mode.
	nrf_write_reg_byte(spi_dev_fd, reg, value);

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
	int lsb_setting = 0;
	int spi_dev_fd;
	
	// Setting (chip enable)
	gpio_export(GPIO_CE);
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set gpio 4 as output.

	// Setting (IRQ)
	gpio_export(GPIO_IRQ);
	gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);
	gpio_set_edge(GPIO_IRQ, rising);
	gpio_set_active_edge(GPIO_IRQ);	// set to active low

	if(errno)
		return 0;

	spi_dev_fd = spi_init(dev, mode, bits, speed, lsb_setting);

	char addr;
	char shutdown_msg[2];
	char snd_msg[3];
	
	shutdown_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, shutdown_msg, 1);

	usleep(10000);


	char msg_addr[5];
	msg_addr[0] = 'R';	
	msg_addr[1] = 'x';	
	msg_addr[2] = 'A';	
	msg_addr[3] = 'A';	
	msg_addr[4] = 'A';	

	nrf_set_rx_address(spi_dev_fd, msg_addr, 0);

	nrf_set_tx_address(spi_dev_fd, msg_addr);


	int rtn;

	nrf_tx_init(spi_dev_fd); 
	nrf_print_all_registers(spi_dev_fd);

	
	char word[31];
	//strcpy(word, "fox jumped over the blue fence");
	
	FILE * fp = fopen("img.jpg", "r");
	send_files(spi_dev_fd, fp);
	fclose(fp);

	nrf_print_all_registers(spi_dev_fd);

	nrf_shutdown(spi_dev_fd);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(GPIO_CE);
	gpio_unexport(GPIO_IRQ);
	// echo 4 > /sys/class/gpio/unexport

	return 0;
}
