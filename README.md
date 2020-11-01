# ThermPDO
This Python 3 script for Raspberry Pi periodically reads temperature from a DS1621 thermometer via I<sup>2</sup>C, and emits a CAN packet containing the result as two four-byte floats.  The first float is in degrees Celsius, the second is in degrees Fahrenheit.  The script uses the [python-can](https://pypi.org/project/python-can/) library, which works with a wide variety of CAN bus [interfaces](https://python-can.readthedocs.io/en/stable/interfaces.html).  I've been using SocketCAN with the SK Pang [PiCAN2](http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-2-p-1475.html), and SLCAN with the Lawicel [CANUSB](http://www.can232.com/?m=201710) dongle for which this script is written.  The python-can library has the nice property that it allows multiple processes on the RasPi to interact with the CAN bus at the same time. Obviously the accesses are serialized behind the scenes.  So this script can run as a process emitting PDOs periodically, while other processes perform their functions on the same CAN bus.  Command line arguments may be used to set the I<sup>2</sup>C bus address for the DS1621, the measurement interval period in seconds, and the arbitration id (or COB id) of the emitted CAN packet.
