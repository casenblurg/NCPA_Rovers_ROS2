#!/bin/bash
 

echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I wlp2s0 -l | grep "00:91:9e:fd:c9:dd" | cut -f1)


if [ -n "$addr" ]
then
	echo  "IP found.. ssh ncpa@$addr"
	ssh -X ncpa@$addr
else
	echo "IP not found. Is the device turned on?"
fi





