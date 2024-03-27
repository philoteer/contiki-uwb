#!/usr/bin/env python3

#ref: https://wikidocs.net/188633
import serial
import threading
import time
import pylink

# Configurations
#################################################################
PORT_NAMES = {"one":"/dev/ttyACM0","two":"/dev/ttyACM1"}
SAVE_PATHS = {"one":"out1.txt","two":"out2.txt"}
INDICATOR = {"one":"*","two":"#"}
JLINK_DEV_LIST = [760126688, 760126516] #JLinkExe -> showemulist

ser = {}
f_out = {}
num_buf = {}
header_cnt = {}

GPIO_DEVICE = "/dev/ttyUSB0"
GPIO_UP_COMMAND=bytes("pin.value(1) \r\n", "ascii")
GPIO_DOWN_COMMAND=bytes("pin.value(0) \r\n", "ascii")

ser_GPIO_DEVICE = serial.Serial(
	port = GPIO_DEVICE, 
	baudrate=115200, 
	parity='N',
	stopbits=1,
	bytesize=8,
	timeout=8,
	xonxoff=0
)

#################################################################

# Functions
#################################################################
def rx_thread(PORT_NAME):
	while(True):
		try:
			input_data = ser[PORT_NAME].readline().decode("ASCII")
			if("CIR" in input_data[0:3]):
				f_out[PORT_NAME].write(f"{time.time() - start_time},{input_data.split(' ')[-1]}")
				f_out[PORT_NAME].flush()
				print(INDICATOR[PORT_NAME], end="",flush=True)
		except:
			if(f_out[PORT_NAME].closed):
				break
			else:
				print("?",end="",flush=True)
			
#Implement this to send the start signal (via GPIO or something)
def gpio_signal_start():
	print("Start signal")
	ser_GPIO_DEVICE.write(GPIO_UP_COMMAND)
	print(ser_GPIO_DEVICE.readline().decode("ASCII"))
	
#Implement this to send the end signal (via GPIO or something)
def gpio_signal_end():
	print("End signal")
	ser_GPIO_DEVICE.write(GPIO_DOWN_COMMAND)
	print(ser_GPIO_DEVICE.readline().decode("ASCII"))

#################################################################

# main()
#################################################################

gpio_signal_end()
print("press enter to continue.")
input()

for DEV in JLINK_DEV_LIST:
	jlink = pylink.JLink()
	jlink.open(DEV)
	jlink.connect("nrf52832_xxAA",speed=1000)
	print(f"Jlink status: {jlink.target_connected()}")
	jlink.reset()
	jlink.close()
	
start_time = time.time()
gpio_signal_start()

for PORT_NAME in PORT_NAMES:
	ser[PORT_NAME] = serial.Serial(
		port = PORT_NAMES[PORT_NAME], 
		baudrate=115200, 
		parity='N',
		stopbits=1,
		bytesize=8,
		timeout=8,
		xonxoff=1
		)

	ser[PORT_NAME].isOpen()

	f_out[PORT_NAME] = open(SAVE_PATHS[PORT_NAME], 'w')

	num_buf[PORT_NAME]=""

	header_cnt[PORT_NAME] = 3

threads = {}
for PORT_NAME in PORT_NAMES:
	threads[PORT_NAME] = (threading.Thread(target=rx_thread, args=(PORT_NAME,)))
	
for PORT_NAME in PORT_NAMES:
	threads[PORT_NAME].start()
	print(f"Thread Started. ({PORT_NAMES[PORT_NAME]})")

while (True):
	try:
		time.sleep(5)
	except KeyboardInterrupt:
		for PORT_NAME in PORT_NAMES:
			f_out[PORT_NAME].close()
		gpio_signal_end()
		break

print("done.")
input()
################################################################
