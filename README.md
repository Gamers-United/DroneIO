Welcome to DroneIO's documentation!
===================================

    DroneIO Is a library developed to provide high level drone control to drone's featuring the
    MPU-6050 Accelerometer/Gyroscope, QMC5883L Magnotometer & BME280 Pressure Sensor. It's goal
    is to make writing control software for a drone all about the actual physhics and basic control
    rather than about the back end I2C communication and processing algorithims such as the Kalman
    filter algoritihm.

    This project uses the following other libraries:

    Libraries Used
    ==============
    1. https://github.com/Tijndagamer/mpu6050
    2. https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050
    3. https://github.com/RigacciOrg/py-qmc5883l
    4. https://pypi.org/project/RPi.bme280/
    5. sys
    6. smbus2
    7. RPI: python-smbus
    8. https://github.com/adafruit/Adafruit_Python_PCA9685/tree/master/Adafruit_PCA9685

    The Kalman filter algorithim used is extremly heavily based on the linked library above.
    
    Note that this is the runnable linux version of the code, while Mac898/DroneIO is the windows test enviroment/documentation correct repo.
