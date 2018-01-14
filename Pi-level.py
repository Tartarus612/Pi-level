#!/usr/bin/python
# This version uses the Complementary Filter output to drive the 2 servos

import smbus
import math
import time
import unicornhathd as unicorn


#set up Unicorn Hat
unicorn.brightness(1.0)

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
 
# Chip temperature register
temp = 0x41
celsius = (temp/340.00) + 36.53
print("Temp = ", "%.2f" % celsius, " deg C")  # Just for fun! (Hope it's right!)

gyro_scale = 131.0
accel_scale = 16384.0

address = 0x68  # This is the address value read via the i2cdetect command

def read_all():
    success = False
    while (success != True):
        try:
            raw_gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
            raw_accel_data = bus.read_i2c_block_data(address, 0x3b, 6)
            success = True
        except:
            time.sleep(.002)
            success = False

    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale

    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z)
    
def twos_compliment(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def get_y_rotation(x,z):
    radians = math.atan2(x, z)
    return -math.degrees(radians)

def get_x_rotation(y,z):
    radians = math.atan2(y, z)
    return math.degrees(radians)

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

now = time.time()

Tau = 0.5                     # accelerometer noise time constant (seconds)
Delta_t = 0.01                # sampling time (seconds)
Alpha = Tau/(Tau + Delta_t)   # apportionment coefficient

time_diff = 0.04

(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()

last_x = get_x_rotation(accel_scaled_y, accel_scaled_z)
last_y = get_y_rotation(accel_scaled_x, accel_scaled_z)

gyro_offset_x = gyro_scaled_x 
gyro_offset_y = gyro_scaled_y

gyro_total_x = (last_x) - gyro_offset_x
gyro_total_y = (last_y) - gyro_offset_y

#print "{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - now, (last_x), gyro_total_x, (last_x), (last_y), gyro_total_y, (last_y))

# Map servo rotation to MPU6050 orientation
slope = .15
middle = 8

while (True):
    time.sleep(time_diff - 0.001) 
    
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
    
    gyro_scaled_x -= gyro_offset_x
    gyro_scaled_y -= gyro_offset_y
    
    gyro_x_delta = (gyro_scaled_x * time_diff)
    gyro_y_delta = (gyro_scaled_y * time_diff)

    gyro_total_x += gyro_x_delta
    gyro_total_y += gyro_y_delta

    rotation_x = get_x_rotation(accel_scaled_y, accel_scaled_z)
    rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_z)

    last_x = Alpha * (last_x + gyro_x_delta) + ((1 - Alpha) * rotation_x)
    last_y = Alpha * (last_y + gyro_y_delta) + ((1 - Alpha) * rotation_y)

    roll = middle+int(slope*last_x)
    pitch = middle+int(slope*last_y)
    
    #print ("last_x = " + str(last_x) + " last_y = " + str(last_y) + " roll = " + str(roll) + " pitch = " + str(pitch))
    if(roll > 15):
        roll = 15
    if (roll < 0):
        roll = 0
    if (pitch > 15):
        pitch = 15
    if (pitch < 0):
        pitch = 0
        
    unicorn.clear()
    unicorn.set_pixel(roll, pitch, 255, 255, 255)
    unicorn.show()