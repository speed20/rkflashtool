#!/bin/bash

OUT=code.h

rm $OUT

echo "unsigned char code[][8475] = {" >> $OUT

for file in $(seq 16);
do
	echo $file
	awk 'BEGIN{printf "\t/********************************** %d **********************************/", "'$((file-1))'"}
		{for (i=2; i<NF; i++) {
			if ((i-2)%16 == 0)
				printf "\n\t";
			printf "0x%s,", $i;
			}
		}' 8475.$file >> $OUT
		echo >> $OUT
done

echo "};" >> $OUT
