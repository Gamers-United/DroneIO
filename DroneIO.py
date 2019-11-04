import sys
import smbus2
import Adafruit_PCA9685
import bme280
from mpu6050 import mpu6050
import py_qmc5883l

class DroneIO:
    """The Drone IO class is the low level class that provides consolidated low level control methods for use in the higher level DroneControl & Filter classes. It is includes methods for control of all """
    def __init__(self, mpuadd, qmcadd, bmeadd, pwmadd, pwmfrequency):
        self.mpuaddress = mpuadd
        self.qmcaddress = qmcadd
        self.bmeaddress = bmeadd
        self.pwmaddress = pwmadd
        self.qmcpollingrate = py_qmc5883l.ODR_50HZ
        self.i2cport = 1
        #connect to IC's
        #adafruit pca9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=self.pwmaddress)
        self.pwm.set_pwm_freq(pwmfrequency)
        self.pwm.set_pwm(0, 0, 360)
        self.pwm.set_pwm(1, 0, 360)
        self.pwm.set_pwm(2, 0, 360)
        self.pwm.set_pwm(3, 0, 360)
        self.mpuaccelrange = mpu6050.ACCEL_RANGE_4G
        #bme280
        self.bus = smbus2.SMBus(self.i2cport)
        self.bmecalibration_params = bme280.load_calibration_params(self.bus, self.bmeaddress)
        #mpu6050
        self.mpu6050 = mpu6050(self.mpuaddress)
        #QMC5883L-TL
        self.qmc5833lcalibrationdata = [[1.0303, 0.0255, -227.7989],
                                        [0.0255, 1.0214, 1016.4415],
                                        [0.0, 0.0, 1.0]]
        self.qmc5883L = py_qmc5883l.QMC5883L(output_range=py_qmc5883l.RNG_2G, output_data_rate=self.qmcpollingrate)

    def setMagnoPollingRate(self, pollingrate):
        """Set the polling rate of the QMC-5883l magnetometer. Valid options are '10', '50', '100', '200'. The unit is in hz and the polling rate should arrive as a string."""
        if pollingrate == "10":
            self.qmcpollingrate = py_qmc5883l.ODR_10HZ
            return
        if pollingrate == "50":
            self.qmcpollingrate = py_qmc5883l.ODR_50HZ
            return
        if pollingrate == "100":
            self.qmcpollingrate = py_qmc5883l.ODR_100HZ
            return
        if pollingrate == "200":
            self.qmcpollingrate = py_qmc5883l.ODR_200HZ
            return
        else:
            self.qmcpollingrate = py_qmc5883l.ODR_50HZ
            return "ERROR"
    def setAccelRange(self, pollingrate):
        """Set the accelerometer range of the mpu6050.. Valid options are '2', '4', '8', '16'. The unit is in G's and the accelerometer range should arrive as a string."""
        if pollingrate == "2":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_2G)
            return
        if pollingrate == "4":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_4G)
            return
        if pollingrate == "8":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_8G)
            return
        if pollingrate == "16":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_16G)
            return
        else:
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_4G)
            return "ERROR"
    def readAccelerometer(self):
        accel_data = self.mpu6050.get_accel_data()
        return accel_data
    def readGyroscope(self):
        gyro_data = self.mpu6050.get_gyro_data()
        return gyro_data
    def readMagnetometer(self):
        magnodata = self.qmc5883L.get_magnet()
        return magnodata
    def readBarometer(self):
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.pressure
    def readTemperature(self):
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.temperature
    def readHumidity(self):
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.humidity
    def setPWM(self, gpio, time):
        maxNum = time / (1000000/4096/60)
        self.pwm.set_pwm(gpio, 0, maxNum)
        print("Motor at Pin "+gpio+" is at duty cycle 0 to "+maxNum)
        return

