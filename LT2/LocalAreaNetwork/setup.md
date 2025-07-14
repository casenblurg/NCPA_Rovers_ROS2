## Network Configuration

### Local Machine Connection

| Step                            | Setting               |
|---------------------------------|------------------------|
| IP Address                      | 192.168.2.10           |
| Subnet Mask                     | 255.255.255.0          |
| Gateway                         | 192.168.2.1            |
| Ubiquiti Dish Connection        | POE injector (POE port)|
| Local Machine Connection        | POE injector (LAN port)|

---

### Rover Setup

| Step            | Setting             |
|-----------------|---------------------|
| IP Address      | 192.168.2.rover#    |
| Subnet Mask     | 255.255.255.0       |
| Gateway         | 192.168.2.1         |


The rover you are trying to connect to should have a static ip set based on which rover it is. For example LEO is LT2-3 so I set its static ip to 192.168.2.3.
If the rover is connected to the POE injector on the rover and the static ip is conigured. Run the LT2.sh script and you should be connected.
