#!/bin/bash

OUT=code.h

cat << EOF >$OUT
#define DRAM_BLK_SIZE 8448
int dram_offset;
EOF

echo "unsigned char code[] = {" >> $OUT

for file in $(seq 16);
do
	echo $file
	awk 'BEGIN{printf "\n\t/********************************** %d **********************************/", "'$((file-1))'"}
		{for (i=2; i<=NF; i++) {
			if ((i-2)%16 == 0)
				printf "\n\t";
			printf "0x%s,", $i;
			}
		}' 8475.$file >> $OUT
done

echo "};" >> $OUT
