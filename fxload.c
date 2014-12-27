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
static int load_state = 0;

static void LIBUSB_CALL cmd_cb(struct libusb_transfer *xfr);
static void LIBUSB_CALL res_cb(struct libusb_transfer *xfr);
int stage1(struct libusb_device_handle **handle, struct libusb_device *dev);
int stage2(struct libusb_device_handle **handle, struct libusb_device *dev);
int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
		libusb_hotplug_event event, void *user_data);
static void LIBUSB_CALL dram_cb(struct libusb_transfer *xfr);

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

static void LIBUSB_CALL dram_cmd_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "res_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	printf("%d transfered\n", xfr->actual_length);

	sector += 0x10;
	if (sector == 0x20f0 + 0x10)
		sector = 0x3000;
	prepare_cmd(RKFT_CMD_WRITESECTOR, sector, 0x10);

	printf("write sector cmd: 0x%04x\n", sector);
	libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, dram_cb, NULL, 60*1000);
	libusb_submit_transfer(xfr);
}

static void LIBUSB_CALL dram_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "res_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (dram_offset < sizeof(code)) {
		int len;
		printf("write sector content\n");

		len = sizeof(code) - dram_offset < DRAM_BLK_SIZE ? sizeof(code) - dram_offset : DRAM_BLK_SIZE;
		if (sector == 0x20f0 + 0x10)
			len = 6336;
		memset(buf, 0, RKFT_BLOCKSIZE);
		memcpy(buf, code+dram_offset, len);
		dram_offset += len;
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, buf, len, cmd_cb, NULL, 60*1000);
		libusb_submit_transfer(xfr);
	}
}

static void LIBUSB_CALL res_cb(struct libusb_transfer *xfr)
{
	printf("enter res_cb\n");

	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "res_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (load_state == 3) {
		sector++;
		if (sector == 0x07) {
			printf("start step 4, load dram firmware.\n");
			load_state = 4;

			sector = 0x2000;
			dram_offset = 0;
			prepare_cmd(RKFT_CMD_WRITESECTOR, sector, 0x10);

			printf("write sector cmd\n");

			libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, dram_cb, NULL, 60*1000);
			libusb_submit_transfer(xfr);
		} else {
			prepare_cmd(RKFT_CMD_ERASESECTORS, sector, 0x01);

			libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, cmd_cb, NULL, 60*1000);
			libusb_submit_transfer(xfr);
		}
	} else if (load_state == 4) {
		printf("i don't knonw\n");
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

	libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, cmd, 31, cmd_cb, NULL, 60*1000);
	libusb_submit_transfer(xfr);
}

static void LIBUSB_CALL cmd_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "cmd_cb, transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (load_state == 3) {
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x81, cmd, 13, res_cb, NULL, 60*1000);
	} else if (load_state == 4) {
		printf("read sector reponse\n");
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x81, cmd, 13, dram_cmd_cb, NULL, 60*1000);
	}
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
			fprintf(stderr, "read usb image failed\n");
			exit(-1);
		}

		if (len > 0) {
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


int main(int argc, char **argv)
{
	int status;
	libusb_device_handle *device = NULL;
	int opt, len;
	char *ddr_file, *usb_file;
	libusb_hotplug_callback_handle handle;
	int rc;

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

	stage = 0;
	rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
			LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, 0, VENDOR, PRODUCT,
			LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, NULL,
			&handle);
	if (LIBUSB_SUCCESS != rc) {
		printf("Error creating a hotplug callback\n");
		libusb_exit(NULL);
		return EXIT_FAILURE;
	}

	while (!do_exit) {
		int rc;
		rc = libusb_handle_events(NULL);
		if (rc != LIBUSB_SUCCESS)
			break;
	}

	/*
	while (1) {
		libusb_handle_events_completed(NULL, NULL);
		usleep(10000);
	}
	*/

	libusb_hotplug_deregister_callback(NULL, handle);
	libusb_exit(NULL);
	return 0;
}

int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
		libusb_hotplug_event event, void *user_data)
{
	static libusb_device_handle *handle = NULL;
	struct libusb_device_descriptor desc;
	int rc;

	libusb_get_device_descriptor(dev, &desc);

	if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
		printf("arrived\n");
		if (stage == 0) {
			printf("stage1\n");
			rc = stage1(&handle, dev);
			stage++;
		} else {
			printf("stage2\n");
			rc = stage2(&handle, dev);
		}
		/*
		rc = libusb_open(dev, &handle);
		if (LIBUSB_SUCCESS != rc) {
			printf("Could not open USB device\n");
		}
		*/
	} else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
		printf("left\n");
		if (handle) {
			libusb_close(handle);
			handle = NULL;
		}
	} else {
		printf("Unhandled event %d\n", event);
	}

	return 0;
}


int stage1(struct libusb_device_handle **handle, struct libusb_device *dev)
{
	struct libusb_transfer *xfr;
	libusb_device_handle *device = NULL;
	int status, len;
	int rc;

	/* try to pick up missing parameters from known devices */
	rc = libusb_open(dev, handle);
	if (LIBUSB_SUCCESS != rc) {
		printf("Could not open USB device\n");
		return -1;
	}
	
	device = *handle;

	/* We need to claim the first interface */
	libusb_set_auto_detach_kernel_driver(device, 1);
	status = libusb_claim_interface(device, 0);
	if (status != LIBUSB_SUCCESS) {
		fprintf(stderr, "libusb_claim_interface failed: %s\n", libusb_error_name(status));
		goto err;
	}

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

err:
	return 0;
}

int stage2(struct libusb_device_handle **handle, struct libusb_device *dev)
{
	struct libusb_transfer *xfr;
	libusb_device_handle *device = NULL;
	int status, len;
	int rc;
	int cfg;

	/* try to pick up missing parameters from known devices */
	rc = libusb_open(dev, handle);
	if (LIBUSB_SUCCESS != rc) {
		printf("Could not open USB device\n");
		return -1;
	}
	
	device = *handle;

	/* We need to claim the first interface */
	libusb_set_auto_detach_kernel_driver(device, 1);
	status = libusb_claim_interface(device, 0);
	if (status != LIBUSB_SUCCESS) {
		fprintf(stderr, "libusb_claim_interface failed: %s\n", libusb_error_name(status));
		goto err;
	}

	/* step 3, test device */
	printf("start step 3, test device.\n");
	xfr = libusb_alloc_transfer(0);

	prepare_cmd(RKFT_CMD_TESTUNITREADY, 0, 0);
	libusb_fill_bulk_transfer(xfr, device, 0x02, cmd, 31, test_device_cb, NULL, 60*1000);
	libusb_submit_transfer(xfr);

err:
	return 0;
}
