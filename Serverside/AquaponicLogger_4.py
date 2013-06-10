#! /usr/bin/python

'''
	AquaponicLogger.py by Miles Thorogood 2012

Program that requests pakets of sensor data from Shaun's sensors and logs the data to a sqlite db
switch relay on 
\x72 and \x61 - 64 (r and a-d)
switch relay off
\x73 and \x61 - 64 (r and a-d)

read sensors
\x61 (a) - returns 34 chars

dummy load
\x62 (b) - returns a b and 33 x chars

'''

from xbee import ZigBee
import sqlite3

import threading
import time
import serial
import time
from datetime import datetime


# delay between asking sensors for data in sec
delay_time = 5

# our list of radio addresses
class radio:
	def __init__(self, addr, name):
		self.addr = addr
		self.name = name
radios = []


# setup serial, which needs to be in the config as setup in the XBee
PORT = 'COM6'
BAUD_RATE = 9600

'''
* Copy and paste radios.append(radio(addr = '', name = ''))
* where addr is the 8 bit address found at the bottom line at the back of the xbee 
* and name is the human readable name you want to give it
'''

radios.append(radio(addr = '\x00\x13\xa2\x00\x40\x6F\x22\x5A', name = 'radio1'))

#radios.append(radio(addr = '\x00\x13\xa2\x00\x40\x6c\xbd\x01', name = 'radio1'))
#radios.append(radio(addr = '\x00\x13\xA2\x00\x40\x6F\x22\x44', name = 'radio2'))
#radios.append(radio(addr = '\x00\x13\xA2\x00\x40\x6F\x21\xE5', name = 'radio3'))
#radios.append(radio(addr = '\x00\x13\xa2\x00\x40\x6c\xbd\x01', name = 'radio1'))








ser = serial.Serial(PORT, BAUD_RATE)
zig = ZigBee(ser, escaped=True)
for radio in radios:
	print "added a radio " + radio.name


'''
XBEE COMMUNICATION
'''
# thread class to sit and wait for incoming xbee packets
class XBeeReceiveThread(threading.Thread):
	def __init__(self, group=None, target=None, name=None,
			args=(), kwargs=None, verbose=None):
		threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
		self.args = args
		self.store = args[0]
		
		return
	def run(self):
		while True:
			#try:
			response = zig.wait_read_frame()			
			#except TypeError:
			self.store.process_data(response) # in main thread


	
'''

SQL STUFF
requires sqlite3 
'''

class sql_store:
	def __init__(self):
		# start up a database connection
		# creates a db if not exists\\
		print "trying db connection"
		self.conn = sqlite3.connect('aquaponic.db', check_same_thread = False)
		# the cursor object
		self.c = self.conn.cursor()
		# Create table throws an error if table exists
		try:
			self.c.execute('''CREATE TABLE sensor_data
				(radio name, read_time, liquidLevel, waterFlow, temperature, 
				humidity, thermocouple, lightFreq, relay0, relay1, 
				relay2, relay3, waterTrip, current0, current1, 
				current2, pH, UV, dOxygen)''')
			print "db table created"
		except:
			sqlite3.OperationalError
			print "table already exists"
		
		
	# function to interpret xbee packets
	def process_data(self, data):
		
		print "got some data"
		print data['id']
		print data		
		# this is the data from radios
		if data['id'] == 'rx': # if rx
			print data
			if len(data['rf_data']) == 34: # if its the sensor data length
				print 'is sensor data length'
				conv = [ord(i) for i in data['rf_data']] # turn lead into gold
				sen_data = []
				for i in xrange(0, len(conv), 2): # repack data
					b = conv[i] << 8 | conv[i+1]
					sen_data.append(b)
				print conv # this is the muck
				print sen_data # this is the good stuff
				if len(sen_data) == 17:
					name =''
					for radio in radios:
						if radio.addr == data['source_addr_long']:
							name = radio.name
					self.log_data(name, sen_data) # log data into database
			
	# this function is for logging the 17 sensor values and the radio name
	def log_data(self, radio, d):
                dt = datetime.now()
                time_now = dt.strftime("%Y-%m-%d-%H:%M:%S.%f")
		# Insert a row of data
		t = (radio, time_now, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15], d[16],)
		self.c.execute("INSERT INTO sensor_data VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", t) 
		# Save (commit) the changes
		self.conn.commit()

	def close(self):
		self.conn.close()
		

'''

REST OF PROGRAM

'''	

print "start of day"

st = sql_store() # open the database connection
print "sql stuff done"
xt = XBeeReceiveThread(args=(st,))
print "created xbee receive thread"
xt.start() # start up a thread to listen for incoming xbee packets

print "xbee started"

zig.send("at", frame='A', command='ND') # look for radios
print "sent to zig"

# function to send request to radio
def send_request(addr):
	print 'sending request to radio ' + addr
	zig.send("tx", frame='A', dest_addr='\x19\xC4', dest_addr_long=addr, data='\x61', options='\x01')
	#zig.send("at", frame='A', command='DH')
	
# Do other stuff in the main thread
while True:
	try:
		for radio in radios:
			send_request(radio.addr)
		time.sleep(delay_time)

	except KeyboardInterrupt:
		break

'''

CLOSE THINGS DOWN BEFORE EXIT

'''
# halt() must be called before closing the serial
# port in order to ensure proper thread shutdown
zig.halt()
ser.close()
# We can also close the sql cursor
st.close()
