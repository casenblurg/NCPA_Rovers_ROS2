# TR1200 Setup/Guide
## **Ensure the TR1200, onboard computer, and the USB to CAN are on before beginning!**
>This section will explain step by step how to access the TR1200 rover and begin remote control from a laptop or PC.

For additional troubleshooting and references see [TR1200 Getting Started](https://docs.trossenrobotics.com/tr1200_docs/getting_started.html) and [OleMissBox](https://olemiss.app.box.com/folder/314410283580)


The purpose of this documentation is to act as a guide to accessing the onboard computer on the **TR1200** as well as a guide to access the controls. The first step is to open a **terminal**. The onboard computer does not have a static IP; therefore a script was made to find the IP on the network. The script uses the onboard computer's WiFi mac address to find the IP address on the network.

**(Mac addresses are in the corresponding Box folder)**

Depending on your computer's Linux distribution you may have to install **arp-scan**.

```bash
$ sudo apt update
$ sudo apt install arp-scan
```
Create a file and name it **ssh_tr1200.sh**, open the file using your preferred text editor and paste the following code into the file.

```bash
$ touch ssh_tr1200.sh
$ vim ssh_tr1200.sh
```
Paste the following code into the file. (Replace **enter:the:rovers:wifi:mac:address** with the actual mac address)

```bash
#!/bin/bash
echo -e "Fetching NCPA Rovers IP Address..\n"
addr=$(sudo arp-scan -I wlp2s0 -l | grep "enter:the:rovers:wifi:mac:address" | cut -f1)
if [ -n "$addr" ]
then
 echo "IP found.. ssh ncpa@$addr"
 ssh -X ncpa@$addr
else
 echo "IP not found. Is the device turned on?"
fi
```
Save and exit the file. Make the file executable and run it. 

```bash
$ chmod +x ssh_tr1200.sh
$ ./ssh_tr1200.sh
```
When you execute this script you will be prompted with **“[sudo] password for user:”**, enter the password.

Once you have logged in to the onboard computer, run the following commands.

```bash
$ screen
$ sudo ip link set can0 up type can bitrate 500000
$ source ~/tr1200_ws/install/setup.bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch tr1200_control control.launch.py
```

Then press CTRL A C (hold control and press A then C). You should still be in screen
mode as instructed earlier. This will take you to a new terminal. Run the commands:

```bash
$ source ~/tr1200_ws/install/setup.bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch tr1200_teleop keyop.launch.py
```
After running this command, a window should pop up. ( “i” corresponds to forward;
“,” corresponds to backwards; “l” corresponds to a right turn; “j” corresponds to a left turn and
the rest of the letters are combinations of those four inputs) You can also increase or decrease
movement speeds and turning speeds as explained by the popup window.
