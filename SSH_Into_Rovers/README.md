**Troubleshooting SSH** *(This Document is for troubleshooting the **SSH** scripts)*

These scripts are designed to automate the **SSH** process. If they do not work, you have two options. First, if you know the IP address of the rover, you can manually SSH into it.

```bash
$ ssh user@IP
```

The second option is to use the known mac address to find the **IP address** on the network. There are two network types that the rovers connect to: 
1. enp0s31f6 **(Ethernet)**
2. wlp2s0 **(WIFI)**

If you are trying to connect to the **LT2** you will use:

```bash
$ sudo arp-scan -I enp0s31f6 -l
```

If you are trying to connect to the **TR1200** you will use:

```bash
$ sudo arp-scan -I wlp2s0 -l
```

Note that when you use these commands you need to be connected to the same network as the device you are looking for.

After running this command, you should see output similar to the following, depending on the network you're connected to:

```bash
	 IP              Mac                NA        

192.168.0.1	  00:50:8b:02:59:b8	 Hewlett Packard
192.168.1.1	  00:14:bf:38:0d:52	 Cisco-Linksys, LLC
192.168.1.1	  c4:04:15:17:e4:31	 NETGEAR (DUP: 2)
192.168.1.1	  c8:d7:19:12:77:7e	 Cisco-Linksys, LLC (DUP: 3)
192.168.2.65      a0:36:bc:ce:44:a2	 (Unknown)
```

Use the **MAC address** column to locate the device you're searching for. (The corresponding MAC addresses are provided in the Box.)

Once you've identified the correct MAC address, use the associated **IP address** to manually SSH into the rover's onboard computer:

```bash
$ ssh user@IP
```

Replace **ip** with the actual **IP address** and **user** with the actual **username**.

