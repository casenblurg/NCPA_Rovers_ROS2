Summary: RS232 to TTL cable. Plugged into COM1 on the UpBoard<sup>2</sup> and TELEM1 on the PixHawk. (Note, this connection is not currently on the Wiring Diagram.)

---
## Purpose
We need a way to connect to the uXRCE-DDS client on the PixHawk while keeping the USB connection open for QGC. Additionally, the most convenient serial ports available on the UpBoard<sup>2</sup> is the COM ports. 

However, the COM ports operate on the rs232 standard which has a `HIGH` of `12v` and a `LOW` of `-12v` on its' data pins; whereas the TELEM ports on the PixHawk, which operate on the USART standard, use `3.3v` and `0v` for `HIGH` and `LOW` respectively.

Thus the Control Cable is a female rs232 to USART converter with a TTL header on the opposite end.

---
## Pin-out
The data cable is made up of **six** conductors, each of which have two colors -- a primary and secondary. For simplicity, only the *primary* color is relevant.

Here is the cable's pin-out:

| Color  | TTL Pin             |
| ------ | ------------------- |
| Red    | `5v`                |
| Black  | `Ground`            |
| Blue   | Ready to Send `RTS` |
| Green  | Clear to Send `CTS` |
| White  | `TX`                |
| Orange | `RX`                |

> Note: In instances where Orange is not avalible, Yellow may be used as a substitute.
---
