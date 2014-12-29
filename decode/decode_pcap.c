#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pcap.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

typedef unsigned int guint32;
typedef unsigned short guint16;
typedef int gint32;

typedef struct pcaprec_hdr_s {
        guint32 ts_sec;         /* timestamp seconds */
        guint32 ts_usec;        /* timestamp microseconds */
        guint32 incl_len;       /* number of octets of packet saved in file */
        guint32 orig_len;       /* actual length of packet */
} pcaprec_hdr_t;

#pragma pack(1)
typedef struct
{
    uint16_t       headerLen; /* This header length */
    uint64_t       irpId;     /* I/O Request packet ID */
    uint32_t	   status;    /* USB status code
                               (on return from host controller) */
    uint16_t       function;  /* URB Function */
    uint8_t        info;      /* I/O Request info */

    uint16_t       bus;       /* bus (RootHub) number */
    uint16_t       device;    /* device address */
    uint8_t        endpoint;  /* endpoint number and transfer direction */
    uint8_t        transfer;  /* transfer type */

    uint32_t       dataLength;/* Data length */
	uint8_t	   stage;
} USBPCAP_BUFFER_PACKET_HEADER, *PUSBPCAP_BUFFER_PACKET_HEADER;

int main(int argc, char *argv[])
{
	void *base;
	struct stat info;
	int fd, offset;
	pcaprec_hdr_t *rec_hdr;
	struct pcap_file_header *pcap_header;
	PUSBPCAP_BUFFER_PACKET_HEADER usb_pkt_hdr;

	stat(argv[1], &info);

	fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		perror("open");
		return -1;
	}

	base = mmap(NULL, info.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (base == MAP_FAILED)
		perror("mmap");

	offset = 0;
	pcap_header = (struct pcap_file_header *)(base + offset);
	printf("magic: 0x%08x\n", pcap_header->magic);
	printf("linktype: %d\n", pcap_header->linktype);
	printf("pcap_file_header len: %ld\n", sizeof(struct pcap_file_header));
	offset += sizeof(struct pcap_file_header);

	FILE *c_header_file = fopen("firmware.h", "w");

	rewind(c_header_file);
	fprintf(c_header_file, "typedef struct {\n\tint len;\n\tuint8_t data[8448];\n} dram_t;\n\ndram_t firmware[] = {\n\t");


	int count = 0;
//	for (count = 0; count < 133; count++) {
	while (offset <= info.st_size) {
		rec_hdr = (pcaprec_hdr_t *)(base + offset);
//		printf("total len: %d actual len: %d\n", rec_hdr->incl_len, rec_hdr->orig_len);
		offset += sizeof(pcaprec_hdr_t);

		usb_pkt_hdr = (PUSBPCAP_BUFFER_PACKET_HEADER)(base + offset);
//		printf("stage: %ld\n", usb_pkt_hdr->stage);

		if (usb_pkt_hdr->endpoint == 0x00 && usb_pkt_hdr->stage == 1) {
			printf("control transfer: %d\n", usb_pkt_hdr->dataLength);
		}

		if (usb_pkt_hdr->endpoint == 0x02 && usb_pkt_hdr->transfer == 0x03 && usb_pkt_hdr->dataLength > 31 && usb_pkt_hdr->dataLength <= 8448) {
			printf("bus: %d device: %d endpoint: 0x%02x dataLength: %d\n", usb_pkt_hdr->bus, usb_pkt_hdr->device,
					usb_pkt_hdr->endpoint, usb_pkt_hdr->dataLength);

			fprintf(c_header_file, "\n\t/******************************** sector %d ********************************/\n\t", count++);
			fprintf(c_header_file, "{\n\t\t");

			fprintf(c_header_file, "%d,{\n\t\t", usb_pkt_hdr->dataLength);

//			fprintf(c_header_file, "{\n\t\t");

			int i;
			for (i=0; i<usb_pkt_hdr->dataLength; i++) {
				if (i % 16 == 0 && i != 0)
					fprintf(c_header_file, "\n\t\t");

				fprintf(c_header_file, "0x%02x,", *((uint8_t *)base + offset + i + usb_pkt_hdr->headerLen));
			}
			fseek(c_header_file, -1, SEEK_CUR);
			fprintf(c_header_file, "}\n\t},");
//			fprintf(c_header_file, "\n");
		}
		offset += rec_hdr->incl_len;

//		printf("data offset: %d %d %d\n", sizeof(pcaprec_hdr_t), sizeof(*usb_pkt_hdr), offset);
	}

	fprintf(c_header_file, "{.len = -1}");

//	fseek(c_header_file, -1, SEEK_CUR);
	fprintf(c_header_file, "\n};");

	munmap(base, info.st_size);
	close(fd);

	return 0;
}
