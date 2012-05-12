 mi2c kernel module
=======


mi2c demonstrates how you can use i2c within another driver handling devices
of different types. It exposes a char dev interface for i2c devices.

The driver registers the devices it wants to handle when it loads. This way
kernel source modifications are not necessary. 

The driver declares the ability to handle two types of devices, "itg3200" and
"arduino" devices.


For the Arduino, there is a pde in the project directory to run the Arduino 
slave code. The Arduino uses the analog pins 4 (sda) and 5 (scl).


If you wanted to make this more generic, you might want to pass the 
i2c_board_info struct to mi2c_init_i2c() since the i2c driver part of the
code doesn't really care. 

