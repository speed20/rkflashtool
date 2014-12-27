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
#include "code.h"

void logerror(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vfprintf(stderr, format, ap);
	va_end(ap);
}

#define VENDOR	0x2207
#define PRODUCT	0x300a

static volatile int do_exit = 0;
char cmd_buf[4096], cmd_ret[4096];
static int stage, total;
static int offset = 0;
static int cmd_count = 0;

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

static void LIBUSB_CALL recv_complete_cb(struct libusb_transfer *xfr);

static void LIBUSB_CALL send_complete_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (seq[cmd_count].flag == 0) {
		fprintf(stderr, "start to receive 13\n");
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x81, cmd_ret, 13, recv_complete_cb, NULL, 60*1000);
	} else {
		fprintf(stderr, "start to receive %d\n", seq[cmd_count].flag);
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x81, cmd_ret, seq[cmd_count].flag, send_complete_cb, NULL, 60*1000);
		seq[cmd_count].flag = 0;
	}

	libusb_submit_transfer(xfr);
}

static void LIBUSB_CALL recv_complete_cb(struct libusb_transfer *xfr)
{
	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	printf("write init seq %d complete\n", cmd_count);
	cmd_count++;

	if (cmd_count < 7) {
		libusb_fill_bulk_transfer(xfr, xfr->dev_handle, 0x02, seq[cmd_count].cmd, 31, send_complete_cb, NULL, 60*1000);
		libusb_submit_transfer(xfr);
	} else {
		exit(0);
	}
}

static void LIBUSB_CALL cb_xfr(struct libusb_transfer *xfr)
{
	unsigned char *buf;
	int addr, len;

	fprintf(stderr, "stage %d complete\n", stage);

	if (xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "transfer status %d\n", xfr->status);
		libusb_free_transfer(xfr);
		exit(3);
	}

	if (offset == total) {
		int cfg, i, rc;
		unsigned char data[18];
		libusb_device_handle *device = NULL;
		struct libusb_transfer *transfer;

		usleep(2000*1000);

		device = xfr->dev_handle;
#if 1
		libusb_get_descriptor(device, 0x01, 0, data, 18);

		libusb_get_configuration(device, &cfg);
		if (cfg != 1) {
			fprintf(stderr, "set config\n");
			libusb_set_configuration(device, 1);
		}

		libusb_free_transfer(xfr);
#endif
		fprintf(stderr, "transfer complete\n");
		do_exit = 1;

#if 0
		fprintf(stderr, "reopen device\n");
		device = libusb_open_device_with_vid_pid(NULL, VENDOR, PRODUCT);
		if (device == NULL) {
			logerror("libusb_open() failed\n");
		}

		libusb_set_auto_detach_kernel_driver(device, 1);
		rc = libusb_claim_interface(device, 0);
		if (rc != LIBUSB_SUCCESS) {
			logerror("libusb_claim_interface failed: %s\n", libusb_error_name(rc));
		}
#endif

	} else {
		stage++;

		if (stage < 3)
			addr = 0x0471;
		else
			addr = 0x0472;

		if (stage == 2 || total - offset == 2050)
			len = 2050;
		else
			len = 4096;

		buf = (unsigned char *)xfr->user_data;
		memset(buf, 0, LIBUSB_CONTROL_SETUP_SIZE + 4096);
		memcpy(buf + LIBUSB_CONTROL_SETUP_SIZE, bootrom + offset, len);
		offset += len;

		libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, addr, len);
		libusb_fill_control_transfer(xfr, xfr->dev_handle, buf, cb_xfr, buf, 60 * 1000);

		libusb_submit_transfer(xfr);
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

#define FIRMWARE 0
#define LOADER 1

int main(int argc, char **argv)
{
	int status;
	libusb_device_handle *device = NULL;
	struct libusb_transfer *xfr;
	pthread_t tid;
	unsigned char *buf;

	total = sizeof(bootrom) / sizeof(bootrom[0]);

	/* open the device using libusb */
	status = libusb_init(NULL);
	if (status < 0) {
		logerror("libusb_init() failed: %s\n", libusb_error_name(status));
		return -1;
	}
	libusb_set_debug(NULL, 3);

	/* try to pick up missing parameters from known devices */
	device = libusb_open_device_with_vid_pid(NULL, VENDOR, PRODUCT);
	if (device == NULL) {
		logerror("libusb_open() failed\n");
		goto err;
	}

	/* We need to claim the first interface */
	libusb_set_auto_detach_kernel_driver(device, 1);
	status = libusb_claim_interface(device, 0);
	if (status != LIBUSB_SUCCESS) {
		logerror("libusb_claim_interface failed: %s\n", libusb_error_name(status));
		goto err;
	}

	status = pthread_create(&tid, NULL, handle_event_func, NULL);
	if (status < 0)
		goto err;

	xfr = libusb_alloc_transfer(0);
	if (argv[1][1] == '1') {
		stage = 0;

		buf = (unsigned char*) malloc(LIBUSB_CONTROL_SETUP_SIZE + 4096);
		memset(buf, 0, LIBUSB_CONTROL_SETUP_SIZE + 4096);
		memcpy(buf + LIBUSB_CONTROL_SETUP_SIZE, bootrom, 4096);
		offset += 4096;

		libusb_fill_control_setup(buf, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, 12, 0, 0x0471, 4096);
		libusb_fill_control_transfer(xfr, device, buf, cb_xfr, buf, 60 * 1000);
	} else if (argv[1][1] == '2') {
		libusb_fill_bulk_transfer(xfr, device, 0x02, seq[cmd_count].cmd, 31, send_complete_cb, NULL, 60*1000);
	} else if (argv[1][1] == '3') {
		/* erase first */
		libusb_fill_bulk_transfer(xfr, device, 0x02, seq[cmd_count].cmd, 31, send_complete_cb, NULL, 60*1000);
	} else if (argv[1][1] == '4') {

55 53 42 43 9b bc 62 99 00 00 00 00 00 00 0a 05
	00 00 00 20 00 00 00 10 00 00 00 00 00 00 00
	}

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
