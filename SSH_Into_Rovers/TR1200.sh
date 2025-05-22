#!/bin/bash
 

echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I wlp2s0 -l | grep "enter:the:rovers:wifi:mac:address" | cut -f1)


if [ -n "$addr" ]
then
	echo  "IP found.. ssh ncpa@$addr"
	ssh -X ncpa@$addr
else
	echo "IP not found. Is the device turned on?"
fi





