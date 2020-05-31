import bme280

mpu = bme280.
while True:
    data = mpu.get_accel_data()
    print(data)

