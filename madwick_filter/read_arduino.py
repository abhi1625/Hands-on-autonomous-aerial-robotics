import serial, time
import numpy as np
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle
arduino.write("Hello from Python!")
time.sleep(1)
i = 0
imu_data = []
while i<500:
	data = arduino.readline()
	print i
	if data:
		# print data.rstrip('\n') #strip out the new lines for now
		# (better to do .read() in the long run for this reason
		imu_data.append(data.rstrip('\n'))
		print imu_data[i]
		i+=1
np.save('imu_data.npz',imu_data)