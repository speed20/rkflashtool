awk 'BEGIN {printf "{\t";}
	{for (i=2; i<=NF; i++) {
		if ((i-2)%16 == 0)
			printf "\n\t";
		printf "0x%s,", $i;
		}
	};
	END{printf "\n};"}' $1
