#!/usr/bin/env python3
# file ThermPDO.py

# https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/
# sudo ip link set can0 up type can bitrate 1000000

# https://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html
# sudo slcand -o -c -f -s8 /dev/ttyUSB0 slcan0
# sudo ifconfig slcan0 up

import sys, can, time, datetime, string, struct

canBus = can.interface.Bus(channel='slcan0', bustype = 'socketcan_native')

# ----------------------------------------------------------
# command line arguments
print ('ThermPDO.py reads temperature from Maxim/Dallas DS1621')
print ('thermometer connected by I2C to Raspberry Pi computer.')
print ('M. Williamsen, 10/30/2020, https://github.com/springleik')
print ('Usage: python3 ThermPDO.py [arg names and values]')
print ('  arg name | arg value')
print ('  ---------|----------')
print ('  -add     | DS1621 I2C address    (default is 0x48)')
print ('  -int     | Interval between PDOs (default is 10 sec)')
print ('  -cob     | COBid of PDO to emit  (default is 0x1AB)')

# set defaults
ds1621Addr = 0x48
period = 10.0
cobID = 0x1AB

# check for user inputs on command line
args = iter(sys.argv)
print ('Running script: "{0}"\n  in Python: {1}'.format(next(args), sys.version))
for arg in args:
	if '-add' == arg:
		ds1621Addr = int(next(args, ds1621Addr), 0)
	elif '-int' == arg:
		interval = float(next(args, period))
	elif '-cob' == arg:
		cobID = int(next(args, cobID), 0)
	else:
		print ('Unexpected argument: {0}'.format(arg))

# ----------------------------------------------------------
# i2c bus interface
# DS1621 register addresses
stopConv   = 0x22
accessTH   = 0xa1
accessTL   = 0xa2
readCount  = 0xa8
readSlope  = 0xa9
readTemp   = 0xaa
accessCfg  = 0xac
startConv  = 0xee

# enable interface in one-shot mode
try:
	import smbus
	i2cBus = smbus.SMBus(bus = 1)
	cfg = i2cBus.read_byte_data(ds1621Addr, accessCfg)
	if 0 == (cfg & 0x01):
		cfg |= 0x01	# set one-shot bit
		print ('Writing config register: {0}'.format(hex(cfg)))
		i2cBus.write_byte_data(ds1621Addr, accessCfg, cfg)
		time.sleep(0.01)
	print ('DS1621 intialized at addr: {0}'.format(hex(ds1621Addr)))

except (IOError, OSError, ImportError) as e:
	i2cBus = None
	print ('Failed to initialize hardware: {0}'.format(e))
	print ('   Running in simulation mode.')

# function to read and report temperature
def getDataPoint():
	if not i2cBus:
		message = {'message':'Simulation mode enabled.'}
		print(message)
		return message
		
	# start a temperature conversion
	i2cBus.write_byte_data(ds1621Addr, startConv, 0)

	# wait up to 1.5 sec. for completion
	done = False
	timeout = 15
	while (not done) and (timeout > 0):
		time.sleep(0.1)
		rslt = i2cBus.read_byte_data(ds1621Addr, accessCfg)
		if rslt & 0x80: done = True
		timeout -= 1
		
	if not timeout:
		error = {'error':'Conversion timed out.'}
		print (error)
		return error

	# read standard (1/2-deg) resolution
	therm = i2cBus.read_word_data(ds1621Addr, readTemp)
	loRes  = (therm << 1) & 0x1fe
	loRes |= (therm >> 15) & 0x01
	if loRes > 255: loRes -= 512
	loRes /= 2.0

	# read high (1/16-deg) resolution
	count = i2cBus.read_byte_data(ds1621Addr, readCount)
	slope = i2cBus.read_byte_data(ds1621Addr, readSlope)
	temp = therm & 0xff
	if temp > 127: temp -= 256
	hiRes = temp - 0.25 + (slope - count) / slope

	# build data point structure
	now = datetime.datetime.now()
	point = {'loResC': loRes,
		'hiResC': hiRes,
		'hiResF': 32.0 + hiRes * 9.0 / 5.0,
		'date': now.strftime('%m/%d/%Y'),
		'time': now.strftime('%H:%M:%S')
		}
	return point

# measure temperature, package two floats as eight bytes of data
def modifyDataPoint(msg):
	thePoint = getDataPoint()
	msg.data = struct.pack('<ff', thePoint['hiResC'], thePoint['hiResF'])

# make first measurement
theMsg = can.Message(arbitration_id = cobID, data = [0]*8, is_extended_id = False)
modifyDataPoint(theMsg)

# main loop emits periodic PDOs with temperature data
print ('Emitting PDO to cob ID {0}, every {1} seconds.'.format(hex(cobID), period))
task = canBus.send_periodic(theMsg, period)
done = False
while not done:
	try:
		modifyDataPoint(theMsg)
		task.modify_data(theMsg)
	except (KeyboardInterrupt):
		print ('  User exit request.')
		done = True
