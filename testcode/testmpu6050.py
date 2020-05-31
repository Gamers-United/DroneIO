import mpu6050

mpu = mpu6050.mpu6050(0x68)
while True:
    data = mpu.get_accel_data()
    print(data)

