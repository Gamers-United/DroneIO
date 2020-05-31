import pigpio

pi = pigpio.pi()

pi.set_mode(17, pigpio.OUTPUT)
pi.set_mode(27, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)
pi.set_mode(23, pigpio.OUTPUT)

pi.set_PWM_frequency(17, 50)
pi.set_PWM_frequency(27, 50)
pi.set_PWM_frequency(22, 50)
pi.set_PWM_frequency(23, 50)

pi.set_PWM_range(17, 10000)
pi.set_PWM_range(27, 10000)
pi.set_PWM_range(22, 10000)
pi.set_PWM_range(23, 10000)

while True:
    angcalc = int(input("Enter ESC % 0-1000"))
    pi.set_PWM_dutycycle(17, angcalc)
    pi.set_PWM_dutycycle(27, angcalc)
    pi.set_PWM_dutycycle(22, angcalc)
    pi.set_PWM_dutycycle(23, angcalc)