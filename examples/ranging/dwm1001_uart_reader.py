#!/usr/bin/env python3

#ref: https://wikidocs.net/188633
import serial
import threading
import time

PORT_NAMES = {"one":"/dev/ttyACM0","two":"/dev/ttyACM1"}
SAVE_PATHS = {"one":"out1.txt","two":"out2.txt"}
INDICATOR = {"one":"*","two":"#"}

ser = {}
f_out = {}
num_buf = {}
header_cnt = {}
start_time = time.time()

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

def rx_thread(PORT_NAME):
	while(True):
		try:
			input_data = ser[PORT_NAME].readline().decode("ASCII")
			if("CIR" in input_data[0:3]):
				f_out[PORT_NAME].write(f"{time.time() - start_time},{input_data.split(' ')[-1]}")
				f_out[PORT_NAME].flush()
				print(INDICATOR[PORT_NAME], end="",flush=True)
		except:
			print("?",end="",flush=True)


threads = {}
for PORT_NAME in PORT_NAMES:
	threads[PORT_NAME] = (threading.Thread(target=rx_thread, args=(PORT_NAME,)))
	
for PORT_NAME in PORT_NAMES:
	threads[PORT_NAME].start()
	print(f"Thread Started. ({PORT_NAMES[PORT_NAME]})")

while (True):
	time.sleep(5)
	
for PORT_NAME in PORT_NAMES:
	f_out[PORT_NAME].close()
