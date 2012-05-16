 multiwii for linux - tools
=======

The user space tool are composed by :
 * mwi : read sensors values from the device driver and do all servos and motors mixing using simplified IMU computation
 * rc : read remote commandes from the pilote/autopilote
 * serial : link to ground station
 
start with :

	$./mwi
	
Ouput :
	
	Setup
	MOTOR :  1000, 1000, 1000, 1000,
	
	ok
	Go
	raw[0] =-89
	raw[1] =112
	raw[2] =-6
	raw[0] =-96
	raw[1] =114
	raw[2] =-2
	MOTOR :  1000, 1000, 1000, 1000,
	raw[0] =-91
	raw[1] =118
	raw[2] =-8
	raw[0] =-89
	raw[1] =111
	raw[2] =-16
	[...]
	more gyro init loop
	[...]
	raw[0] =-88
	raw[1] =123
	raw[2] =-6
	rcData[THROTTLE] = 1500
	raw[0] =-93
	raw[1] =107
	raw[2] =-6
	rcData[THROTTLE] = 1500
	MOTOR :  1488, 1530, 1388, 1434,
	raw[0] =-84
	raw[1] =107
	raw[2] =-10
	rcData[THROTTLE] = 1500
	raw[0] =-87
	raw[1] =113
	raw[2] =-12
	rcData[THROTTLE] = 1500
	MOTOR :  1456, 1534, 1390, 1460,
	raw[0] =-88
	raw[1] =111
	raw[2] =-14
	rcData[THROTTLE] = 1500
	raw[0] =-91
	raw[1] =111
	raw[2] =-10
	rcData[THROTTLE] = 1500
	MOTOR :  1411, 1537, 1391, 1501,
	raw[0] =-87
	raw[1] =109
	raw[2] =-7
	rcData[THROTTLE] = 1500
	raw[0] =-95
	raw[1] =109
	raw[2] =-9
	rcData[THROTTLE] = 1500
	MOTOR :  1438, 1528, 1396, 1478,
