#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/types.h>
#include <getopt.h>
#include <unistd.h>
#include <libusb.h>
#include <pthread.h>
#include "bootrom.h"
#include "data/code.h"
#include <fcntl.h>

#include "version.h"
#include "rkcrc.h"
#include "rkflashtool.h"

#define VENDOR	0x2207
#define PRODUCT	0x300a

#define RKFT_BLOCKSIZE      0x4000      /* must be multiple of 512 */
#define RKFT_IDB_BLOCKSIZE  0x210
#define RKFT_IDB_INCR       0x20
#define RKFT_MEM_INCR       0x80
#define RKFT_OFF_INCR       (RKFT_BLOCKSIZE>>9)
#define MAX_PARAM_LENGTH    (128*512-12) /* cf. MAX_LOADER_PARAM in rkloader */
#define SDRAM_BASE_ADDRESS  0x60000000

#define RKFT_CMD_TESTUNITREADY      0x80000600
#define RKFT_CMD_READFLASHID        0x80000601
#define RKFT_CMD_READFLASHINFO      0x8000061a
#define RKFT_CMD_READCHIPINFO       0x8000061b
#define RKFT_CMD_READEFUSE          0x80000620

#define RKFT_CMD_SETDEVICEINFO      0x00000602
#define RKFT_CMD_ERASESYSTEMDISK    0x00000616
#define RKFT_CMD_SETRESETFLASG      0x0000061e
#define RKFT_CMD_RESETDEVICE        0x000006ff

#define RKFT_CMD_TESTBADBLOCK       0x80000a03
#define RKFT_CMD_READSECTOR         0x80000a04
#define RKFT_CMD_READLBA            0x80000a14
#define RKFT_CMD_READSDRAM          0x80000a17
#define RKFT_CMD_UNKNOWN1           0x80000a21

#define RKFT_CMD_WRITESECTOR        0x00000a05
#define RKFT_CMD_ERASESECTORS       0x00000a06
#define RKFT_CMD_UNKNOWN2           0x00000a0b
#define RKFT_CMD_WRITELBA           0x00000a15
#define RKFT_CMD_WRITESDRAM         0x00000a18
#define RKFT_CMD_EXECUTESDRAM       0x00000a19
#define RKFT_CMD_WRITEEFUSE         0x00000a1f
#define RKFT_CMD_UNKNOWN3           0x00000a22

#define RKFT_CMD_WRITESPARE         0x80001007
#define RKFT_CMD_READSPARE          0x80001008

#define RKFT_CMD_LOWERFORMAT        0x0000001c
#define RKFT_CMD_WRITENKB           0x00000030

#define SETBE16(a, v) do { \
                        ((uint8_t*)a)[1] =  v      & 0xff; \
                        ((uint8_t*)a)[0] = (v>>8 ) & 0xff; \
                      } while(0)

#define SETBE32(a, v) do { \
                        ((uint8_t*)a)[3] =  v      & 0xff; \
                        ((uint8_t*)a)[2] = (v>>8 ) & 0xff; \
                        ((uint8_t*)a)[1] = (v>>16) & 0xff; \
                        ((uint8_t*)a)[0] = (v>>24) & 0xff; \
                      } while(0)

#define BLK_SIZE 4096


static volatile int do_exit = 0;
static uint8_t cmd[31], res[13], buf[RKFT_BLOCKSIZE];
static int stage, total;
static int offset = 0;
static int cmd_count = 0;
static int ddr_fd, usb_fd;
static int sector;
static uint16_t crc16;

struct init_seq {
	char cmd[31];
	int flag;
};

static struct init_seq seq[31] = {
	{
		{
			0x55,0x53,0x42,0x43,0xcf,0x31,0x90,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x06,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		0
	},

	{
		{
			0x55,0x53,0x42,0x43,0xe5,0x59,0x7a,0x95,0x00,0x00,0x00,0x00,0x80,0x00,0x06,0x1b,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		16
	},
	{
		{
			0x55,0x53,0x42,0x43,0xdb,0x2c,0xbf,0xd2,0x00,0x00,0x00,0x00,0x80,0x00,0x06,0x1a,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		11
	},
	{
		{
			0x55,0x53,0x42,0x43,0x03,0x4d,0x83,0xb5,0x00,0x00,0x00,0x00,0x80,0x00,0x06,0x01,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		5
	},
	{
		{
			0x55,0x53,0x42,0x43,0x2a,0x25,0x5d,0x17,0x00,0x00,0x00,0x00,0x80,0x00,0x0a,0x03,
			0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		64
	},
	{
		{
			0x55,0x53,0x42,0x43,0x01,0x1e,0x72,0xfd,0x00,0x00,0x00,0x00,0x80,0x00,0x0a,0x04,
			0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		2112
	},
	{
		{
			0x55,0x53,0x42,0x43,0x8b,0x17,0x6a,0x6a,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0xff,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		},
		0
	}
};

static int load_state = 0;

static void prepare_cmd(uint32_t command, uint32_t offset, uint16_t nsectors)
{
    long int r = random();
	int i;

    memset(cmd, 0 , 31);
    memcpy(cmd, "USBC", 4);

    if (r)          SETBE32(cmd+4, r);
    if (offset)     SETBE32(cmd+17, offset);
    if (nsectors)   SETBE16(cmd+22, nsectors);
    if (command)    SETBE32(cmd+12, command);

	for (i=0; i<31; i++) {
		printf("%02x", cmd[i]);
	}

	printf("\n");
}


static void LIBUSB_CALL cmd_cb(struct libusb_transfer *xfr);
static void LIBUSB_CALL res_cb(struct libusb_transfer *xfr);

static void LIBUSB_CALL res_cb(struct libusb_transfer *xfr)
{
	printf("enter res_cb\n");

	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "res_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	sector++;
	if (sector == 0x06) {
		printf("start step 4, load dram firmware.\n");

		load_state = 4;
		exit(0);
	} else {
		prepare_cmd(RKFT_CMD_ERASESECTORS, sector, 0x01);

		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, cmd_cb, NULL, 60*1000);
		libusb_submit_transfer(xfr);
	}
}


static void LIBUSB_CALL test_device_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "cmd_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	printf("start step 4, erase sectors.\n");

	load_state = 3;
	sector = 0x02;
	prepare_cmd(RKFT_CMD_ERASESECTORS, sector, 0x01);

	libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, res_cb, NULL, 60*1000);
	libusb_submit_transfer(xfr);
}

static void LIBUSB_CALL cmd_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "cmd_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	/* read 13 bytes return value */
	libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x81, cmd, 13, res_cb, NULL, 60*1000);
	libusb_submit_transfer(xfr);
}

static void LIBUSB_CALL ddr_xfr_cb(struct libusb_transfer *xfr)
{
	int len;

	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "ddr_xfr_cb transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (load_state == 0) {
		len = read(ddr_fd, buf + LIBUSB_CONTROL_SETUP_SIZE, BLK_SIZE);
		if (len < 0) {
			fprintf(stderr, "read ddr image failed\n");
			exit(-1);
		}

		if (len == 0) {
			printf("start step 2, load usbplug image.\n");

			len = read(usb_fd, buf + LIBUSB_CONTROL_SETUP_SIZE, BLK_SIZE);
			if (len < 0) {
				fprintf(stderr, "read ddr image failed\n");
				exit(-1);
			}

			load_state = 1;
			crc16 = 0xffff;
			crc16 = rkcrc16(crc16, buf + LIBUSB_CONTROL_SETUP_SIZE, len);
			libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, 0x0472, len);
			libusb_submit_transfer(xfr);
		} else {
			crc16 = rkcrc16(crc16, buf + LIBUSB_CONTROL_SETUP_SIZE, len);

			if (len != BLK_SIZE) {
				buf[LIBUSB_CONTROL_SETUP_SIZE + len++] = crc16 >> 8;
				buf[LIBUSB_CONTROL_SETUP_SIZE + len++] = crc16 & 0xff;
			}

			libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, 0x0471, len);
			libusb_submit_transfer(xfr);
		}
	} else if (load_state == 1) {
		len = read(usb_fd, buf + LIBUSB_CONTROL_SETUP_SIZE, BLK_SIZE);
		if (len < 0) {
			fprintf(stderr, "read ddr image failed\n");
			exit(-1);
		}

		if (len == 0) {
			int cfg;
			char data[18];
			int status;

#if 1
			printf("close old device\n");
			libusb_release_interface(xfr->dev_handle, 0);
			libusb_close(xfr->dev_handle);
			libusb_exit(NULL);

			libusb_init(NULL);

			/* try to pick up missing parameters from known devices */
			xfr->dev_handle = libusb_open_device_with_vid_pid(NULL, VENDOR, PRODUCT);
			if (xfr->dev_handle == NULL) {
				fprintf(stderr, "libusb_open() failed\n");
				exit(-1);
			}

			/* We need to claim the first interface */
			libusb_set_auto_detach_kernel_driver(xfr->dev_handle, 1);
			status = libusb_claim_interface(xfr->dev_handle, 0);
			if (status != LIBUSB_SUCCESS) {
				fprintf(stderr, "libusb_claim_interface failed: %s\n", libusb_error_name(status));
				exit(-1);
			}
#endif
			usleep(1000 * 1000);
			libusb_get_descriptor(xfr->dev_handle, 0x01, 0, data, 18);
			libusb_get_configuration(xfr->dev_handle, &cfg);
			if (cfg != 1) {
				fprintf(stderr, "set config\n");
				libusb_set_configuration(xfr->dev_handle, 1);
			}

			usleep(1000 * 1000);
			printf("start step 3, test device.\n");
			load_state = 2;

			prepare_cmd(RKFT_CMD_TESTUNITREADY, 0, 0);
			libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, test_device_cb, NULL, 60*1000);
			libusb_submit_transfer(xfr);
		} else {
			crc16 = rkcrc16(crc16, buf + LIBUSB_CONTROL_SETUP_SIZE, len);

			if (len != 4096) {
				buf[LIBUSB_CONTROL_SETUP_SIZE + len++] = crc16 >> 8;
				buf[LIBUSB_CONTROL_SETUP_SIZE + len++] = crc16 & 0xff;
			}

			libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, 0x0472, len);
			libusb_submit_transfer(xfr);
		}
	}
}


void *handle_event_func()
{
	while (!do_exit) {
		int rc;
		rc = libusb_handle_events(NULL);
		if (rc != LIBUSB_SUCCESS)
			break;
	}

	return NULL;
}

int main(int argc, char **argv)
{
	int status;
	libusb_device_handle *device = NULL;
	struct libusb_transfer *xfr;
	pthread_t tid;
	int opt, len;
	char *ddr_file, *usb_file;

	ddr_file = usb_file = NULL;
	while ((opt = getopt(argc, argv, "d:u:")) != -1) {
		switch (opt) {
			case 'd':
				ddr_file = strdup(optarg);
				break;
			case 'u':
				usb_file = strdup(optarg);
				break;
			default:
				printf("usage: fxload -d ddr_file -u usb_file\n");
		}
	}

	if (ddr_file == NULL || usb_file == NULL) {
		printf("usage: fxload -d ddr_file -u usb_file\n");
		return 0;
	}

	ddr_fd = open(ddr_file, O_RDONLY);
	if (ddr_fd < 0) {
		fprintf(stderr, "can't open ddr image\n");
		return -1;
	}

	usb_fd = open(usb_file, O_RDONLY);
	if (usb_fd < 0) {
		fprintf(stderr, "can't open usb image\n");
		return -1;
	}

	/* open the device using libusb */
	status = libusb_init(NULL);
	if (status < 0) {
		fprintf(stderr, "libusb_init() failed: %s\n", libusb_error_name(status));
		return -1;
	}
	libusb_set_debug(NULL, 3);

	/* try to pick up missing parameters from known devices */
	device = libusb_open_device_with_vid_pid(NULL, VENDOR, PRODUCT);
	if (device == NULL) {
		fprintf(stderr, "libusb_open() failed\n");
		goto err;
	}

	/* We need to claim the first interface */
	libusb_set_auto_detach_kernel_driver(device, 1);
	status = libusb_claim_interface(device, 0);
	if (status != LIBUSB_SUCCESS) {
		fprintf(stderr, "libusb_claim_interface failed: %s\n", libusb_error_name(status));
		goto err;
	}

	status = pthread_create(&tid, NULL, handle_event_func, NULL);
	if (status < 0)
		goto err;

	/* step 1, load ddr image */
	xfr = libusb_alloc_transfer(0);

	memset(buf, 0, RKFT_BLOCKSIZE);

	printf("start step 1, load ddr image.\n");
	len = read(ddr_fd, buf + LIBUSB_CONTROL_SETUP_SIZE, BLK_SIZE);
	crc16 = 0xffff;
	crc16 = rkcrc16(crc16, buf + LIBUSB_CONTROL_SETUP_SIZE, len);

	libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, 0x0471, len);
	libusb_fill_control_transfer(xfr, device, buf, ddr_xfr_cb, NULL, 60 * 1000);
	libusb_submit_transfer(xfr);

	pthread_join(tid, NULL);

	libusb_release_interface(device, 0);
	libusb_close(device);
	libusb_exit(NULL);
	return status;
err:
	libusb_exit(NULL);
	return -1;
}
