#!/bin/bash


echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I enp0s31f6 -l | grep "enter:Mac" | cut -f1) #only enter the first two sections for examnple 00:00


if [ -n "$addr" ]
then
        echo  "IP found.. ssh ncpa@$addr"
        ssh -X ncpa@$addr
else
        echo "IP not found. Is the device turned on?"
fi













