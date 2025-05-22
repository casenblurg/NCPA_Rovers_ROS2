#!/bin/bash


echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I enp0s31f6 -l | grep "78:45:58:a2:49:ec" | cut -f1)


if [ -n "$addr" ]
then
        echo  "IP found.. ssh ncpa@$addr"
        ssh -X ncpa@$addr
else
        echo "IP not found. Is the device turned on?"
fi













