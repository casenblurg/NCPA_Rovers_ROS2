# TR1200 Setup/Guide

>This section will explain step by step how to acces the TR1200 rover and begin remote control.

The purpose of this documentation is to act as a guide to accessing the Up Squared
Pro on the **TR1200** as well as a guide to access the controls. The first step is to open a **terminal**
and follow these steps to find and **SSH** into the blue rover’s computer:

```
#!/bin/bash
echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I wlp2s0 -l | grep "00:91:9e:fd:c9:dd" | cut -f1)
if [ -n "$addr" ]
then
 echo "IP found.. ssh ncpa@$addr"
 ssh -X ncpa@$addr
else
 echo "IP not found. Is the device turned on?"
fi
```

