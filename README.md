Welcome to DroneIO’s documentation!¶
DroneIO Is a library developed to provide high level drone control to drone’s featuring the MPU-6050 Accelerometer/Gyroscope, QMC5883L Magnotometer & BME280 Pressure Sensor. It’s goal is to make writing control software for a drone all about the actual physhics and basic control rather than about the back end I2C communication and processing algorithims such as the Kalman filter algoritihm.

This project uses the following other libraries:

https://github.com/Tijndagamer/mpu6050
https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050
https://github.com/RigacciOrg/py-qmc5883l
https://pypi.org/project/RPi.bme280/
sys
smbus2
RPI: python-smbus
https://github.com/adafruit/Adafruit_Python_PCA9685/tree/master/Adafruit_PCA9685
The Kalman filter algorithim used is extremly heavily based on the linked library above.
