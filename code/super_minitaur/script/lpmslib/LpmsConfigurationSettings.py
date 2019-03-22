
from LpmsConfig import LPMS_STREAM_FREQ
class LpmsConfigurationSettings(object):

    def __init__(self):
        self.stream_frequency = 0
        self.reserved3_8 = 0
        self.pressure_enable = 0
        self.magnetometer_enable = 0
        self.accelerometer_enable = 0
        self.gyro_enable = 0
        self.temperature_enable = 0
        self.heave_motion_output_enable = 0
        self.gyro_calibration_enable = 0
        self.angular_velocity_enable = 0
        self.euler_enable = 0
        self.quaternion_enable = 0
        self.altitude_enable = 0
        self.dynamic_covariance_enable = 0
        self.linear_acceleration_enable = 0
        self.sixteen_bit_data_enable = 0
        self.gyro_threshold_enable = 0
        self.magnetometer_compensation_enable = 0
        self.accelerometer_compensation_enable = 0
        self.heave_motion_enable = 0
        self.reserved27 = 0
        self.reserved28 = 0
        self.reserved29 = 0
        self.gyro_autocalibration_enable = 0
        self.reserved31 = 0

    def __str__(self):
        j = 50
        d = '.'
        res = ""
        res += "# Configuration Register #" + "\n"
        res += "[ 0:2 ] Stream Frequency".ljust(j, d) + str(self.stream_frequency) + "\n"
        res += "[ 3:8 ] Reserved".ljust(j, d) + str(self.reserved3_8) + "\n"
        res += "[ 9   ] Pressure Output enabled".ljust(j,d) + str(self.pressure_enable) + "\n"
        res += "[ 10  ] Magnetometer raw enabled".ljust(j,d) + str(self.magnetometer_enable) + "\n"
        res += "[ 11  ] Accelerometer raw enabled".ljust(j,d) + str(self.accelerometer_enable) + "\n"
        res += "[ 12  ] Gyro raw enabled".ljust(j,d) + str(self.gyro_enable) + "\n"
        res += "[ 13  ] Temperature output enabled".ljust(j,d) + str(self.temperature_enable) + "\n"
        res += "[ 14  ] Heave motion output enabled".ljust(j,d) + str(self.heave_motion_output_enable) + "\n"
        res += "[ 15  ] Gyro calibration enabled".ljust(j,d) + str(self.gyro_calibration_enable) + "\n"
        res += "[ 16  ] Angular velocity enabled".ljust(j,d) + str(self.angular_velocity_enable) + "\n"
        res += "[ 17  ] Euler enabled".ljust(j,d) + str(self.euler_enable) + "\n"
        res += "[ 18  ] Quaternion raw enabled".ljust(j,d) + str(self.quaternion_enable) + "\n"
        res += "[ 19  ] Altitude raw enabled".ljust(j,d) + str(self.altitude_enable) + "\n"
        res += "[ 20  ] Dynamic covariance enabled".ljust(j,d) + str(self.dynamic_covariance_enable) + "\n"
        res += "[ 21  ] Linear acceleration enabled".ljust(j,d) + str(self.linear_acceleration_enable) + "\n"
        res += "[ 22  ] 16Bit data enabled".ljust(j,d) + str(self.sixteen_bit_data_enable) + "\n"
        res += "[ 24  ] Magnetometer compensation enabled".ljust(j,d) + str(self.magnetometer_compensation_enable) + "\n"
        res += "[ 25  ] Accelerometer compensation enabled".ljust(j,d) + str(self.accelerometer_compensation_enable) + "\n"
        res += "[ 26  ] Heave motion enabled".ljust(j,d) + str(self.heave_motion_enable) + "\n"
        res += "[ 27  ] Reserved".ljust(j,d) + str(self.reserved27) + "\n"
        res += "[ 28  ] Reserved".ljust(j,d) + str(self.reserved28) + "\n"
        res += "[ 29  ] Reserved".ljust(j,d) + str(self.reserved29) + "\n"
        res += "[ 30  ] Gyro auto-calibration enabled".ljust(j,d) + str(self.gyro_autocalibration_enable) + "\n"
        res += "[ 31  ] Reserved".ljust(j,d) + str(self.reserved31) + "\n"
        return res


    def pretty_print(self):
        print self.__str__()

    def parse(self, cr):
        l = ("{0:b}".format(cr)).zfill(32)
        self.stream_frequency = LPMS_STREAM_FREQ[int(l[-3:], 2)]
        self.reserved3_8 = l[-9:-3]
        self.pressure_enable = int(l[-10])
        self.magnetometer_enable = int(l[-11])
        self.accelerometer_enable = int(l[-12])
        self.gyro_enable = int(l[-13])
        self.temperature_enable = int(l[-14])
        self.heave_motion_output_enable = int(l[-15])
        self.gyro_calibration_enable = int(l[-16])
        self.angular_velocity_enable = int(l[-17])
        self.euler_enable = int(l[-18])
        self.quaternion_enable = int(l[-19])
        self.altitude_enable = int(l[-20])
        self.dynamic_covariance_enable = int(l[-21])
        self.linear_acceleration_enable = int(l[-22])
        self.sixteen_bit_data_enable = int(l[-23])
        self.gyro_threshold_enable = int(l[-24])
        self.magnetometer_compensation_enable = int(l[-25])
        self.accelerometer_compensation_enable = int(l[-26])
        self.heave_motion_enable = int(l[-27])
        self.reserved27 = int(l[-28])
        self.reserved28 = int(l[-29])
        self.reserved29 = int(l[-30])
        self.gyro_autocalibration_enable = int(l[-31])
        self.reserve31 = 0