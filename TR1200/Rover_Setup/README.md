# TR1200 Setup/Guide
## **Ensure the TR1200 is turned on before beginning!**
>This section will explain step by step how to acces the TR1200 rover and begin remote control.

The purpose of this documentation is to act as a guide to accessing the onboard computer on the **TR1200** as well as a guide to access the controls. The first step is to open a **terminal**. The onboard computer does not have a static IP therefore a script was made to find the IP on the network. The script uses the onboard computer's wifi mac address to find the IP address on the network. Depending on your Linux distrobution you may have to install **arp-scan**.

```bash
$ sudo apt update
$ sudo apt install arp-scan
```


blah bllah put words here 


```bash
#!/bin/bash
echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I wlp2s0 -l | grep ":::::" | cut -f1)
if [ -n "$addr" ]
then
 echo "IP found.. ssh ncpa@$addr"
 ssh -X ncpa@$addr
else
 echo "IP not found. Is the device turned on?"
fi
```

