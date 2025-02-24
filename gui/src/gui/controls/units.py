import math

class ConverterUnits():

    def __init__(self):
        pass

    def rpm_to_rad_per_sec(self, rpm):
        return round(rpm * (math.pi / 30), 3) #rad/s
    
    def rad_per_sec_to_rpm(self, rad_per_sec):
        return round(rad_per_sec * (30 / math.pi), 3)