import sys
import smbus2
import Adafruit_PCA9685
import bme280
from mpu6050 import mpu6050
import py_qmc5883l
import AngleMeterAlpha

class DroneIO:
    """The Drone IO class is the low level class that provides consolidated low level control methods for use in the higher level DroneControl & Filter classes. It is includes methods for control of all underlying drone componenets. 
       mpuadd is the mpu6050 address in hex format 0x**, same with qmcadd, bmeadd and pwmadd. pwmadd is the PCA9685 address. PWM frequency is equivalent to the pwm frequency in hz. Do not add hz to the input."""
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
        #init kalman filtered mpu
        self.kalmanfilter = AngleMeterAlpha.AngleMeterAlpha()
        self.kalmanfilter.MPU_Init()
        self.kalmanfilter.measure()
        

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
    def setAccelRange(self, accelmaxg):
        """Set the accelerometer range of the mpu6050.. Valid options are '2', '4', '8', '16'. The unit is in G's and the accelerometer range should arrive as a string."""
        if accelmaxg == "2":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_2G)
            return
        if accelmaxg == "4":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_4G)
            return
        if accelmaxg == "8":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_8G)
            return
        if accelmaxg == "16":
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_16G)
            return
        else:
            self.mpu6050.set_accel_range(self.mpu6050.ACCEL_RANGE_4G)
            return "ERROR"
    def setGyroRange(self, gyromaxdeg):
        """Set the maximum gyroscope rotation rate per second. Valid options are '250', '500', '1000', '2000'. The unit is degrees, and range should arrive as a string."""
        if gyromaxdeg == "250":
            self.mpu6050.set_gyro_range(self.mpu6050.GYRO_RANGE_250DEG)
            return
        if gyromaxdeg == "500":
            self.mpu6050.set_gyro_range(self.mpu6050.GYRO_RANGE_500DEG)
            return
        if gyromaxdeg == "1000":
            self.mpu6050.set_gyro_range(self.mpu6050.GYRO_RANGE_1000DEG)
            return
        if gyromaxdeg == "2000":
            self.mpu6050.set_gyro_range(self.mpu6050.GYRO_RANGE_2000DEG)
            return
        else:
            self.mpu6050.set_gyro_range(self.mpu6050.GYRO_RANGE_500DEG)
            return "ERROR"
    def setQMCCalibratioNData(self, calibrationarray):
        """Set a new QMC-5883L calibration data array from the output of the magno-calibration scripts."""
        self.qmc5833lcalibrationdata = calibrationarray
    def readAccelerometer(self):
        """Read raw accelerometer data from the drone."""
        accel_data = self.mpu6050.get_accel_data()
        return accel_data
    def readGyroscope(self):
        """Read raw gyroscope data from the drone."""
        gyro_data = self.mpu6050.get_gyro_data()
        return gyro_data
    def readKalmanPitch(self):
        """Read Kalman filtered pitch in degrees"""
        return self.kalmanfilter.pitch
    def readKalmanRoll(self):
        """Read Kalman filtered roll in degrees"""
        return self.kalmanfilter.roll
    def readMagnetometer(self):
        """Read raw magnetometer data from the QMC5883L."""
        magnodata = self.qmc5883L.get_magnet()
        return magnodata
    def readBarometer(self):
        """Read raw barometer data from the BME280."""
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.pressure
    def readTemperature(self):
        """Read raw temperature data from the BME280."""
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.temperature
    def readHumidity(self):
        """Read raw humidity data from the BME280."""
        data = bme280.sample(self.bus, self.bmeaddress, self.bmecalibration_params)
        return data.humidity
    def setPWM(self, gpio, time):
        "Set a PWM pin's on time in ms. gpio is the output pin of the PCA-9685 to use, 1-16 and the time is the ontime in ms."
        maxNum = time / (1000000/4096/60)
        self.pwm.set_pwm(gpio, 0, maxNum)
        print("Motor at Pin "+gpio+" is at duty cycle 0 to "+maxNum)
        return

class DroneControl:
    def __init__(self):
        """Drone Control Class. Provides high level methods to the low level methods contained in DroneIO, also provides output in the nicest possible fashion"""
        self.mpuadd = 0x68
        self.qmcadd = 0x0D
        self.bmeadd = 0x76
        self.pcaadd = 0x40
        self.pwmfrequency = 50
        self.droneio = DroneIO(self.mpuadd, self.qmcadd, self.bmeadd, self.pcaadd, self.pwmfrequency)

    def AccelX(self):
        """Returns Accelerometer X Value in m/s^2"""
        threeaxis = self.droneio.readAccelerometer()
        return threeaxis['x']
    def AccelY(self):
        """Returns Accelerometer Y Value in m/s^2"""
        threeaxis = self.droneio.readAccelerometer()
        return threeaxis['y']
    def AccelZ(self):
        """Returns Accelerometer Z Value in m/s^2"""
        threeaxis = self.droneio.readAccelerometer()
        return threeaxis['z']
    def GyroX(self):
        """Returns Gyroscope X Value in degrees/second"""
        threeaxis = self.droneio.readGyroscope()
        return threeaxis['x']
    def GyroY(self):
        """Returns Gyroscope Y Value in degrees/second"""
        threeaxis = self.droneio.readGyroscope()
        return threeaxis['y']
    def GyroZ(self):
        """Returns Gyroscope Z Value in degrees/second"""
        threeaxis = self.droneio.readGyroscope()
        return threeaxis['z']
    def BaroP(self):
        """Returns barometer pressure in hPa"""
        return self.droneio.readBarometer()
    def BaroT(self):
        """Returns barometer temperature in hPa"""
        return self.droneio.readTemperature()
    def BaroH(self):
        """Returns barometer humidity in hPa"""
        return self.droneio.readHumidity()      