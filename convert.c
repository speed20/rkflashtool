#include <stdio.h>
#include "rom.h"

int main()
{
	int i, j;

	for (i=0; i<sizeof(data)/sizeof(data[0]); i++) {
		if (i % 4 == 0 && i != 0)
			printf("\n");
		for (j=3; j>=0; j--) {
			printf("0x%02x,", (data[i] >> (j * 8)) & 0xff);
		}
//		printf("0x%08x\n", data[i]);
	}

	return 0;
}
