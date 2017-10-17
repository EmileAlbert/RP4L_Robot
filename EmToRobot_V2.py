# coding: utf-8
import RPi.GPIO as GPIO
import sys
import time
import math

class EmToRobot(object):
    def __init__(self, PinsMotorLeft, PinsMotorRight, listPinSensor):
        self.motor = {"MotorLeft": PinsMotorLeft,
                      "MotorRight": PinsMotorRight}

        self.US_echo = listPinSensor[0]
        self.US_trigg = listPinSensor[1]
        self.OD_Right = listPinSensor[2]
        self.OD_Left = listPinSensor[3]

        self.pathLenght_Right = 0
        self.pathLenght_Left = 0

    def __enter__(self):
        GPIO.setmode(GPIO.BCM)
        for entry in self.motor:
            # Initialisation des deux couples de pins moteur
            GPIO.setup(self.motor[entry][0], GPIO.OUT)
            GPIO.setup(self.motor[entry][1], GPIO.OUT)

        self.motor["MLDuty+"] = GPIO.PWM(self.motor["MotorLeft"][0], 50)
        self.motor["MLDuty-"] = GPIO.PWM(self.motor["MotorLeft"][1], 50)
        self.motor["MRDuty+"] = GPIO.PWM(self.motor["MotorRight"][0], 50)
        self.motor["MRDuty-"] = GPIO.PWM(self.motor["MotorRight"][1], 50)

        GPIO.setup(self.US_echo, GPIO.IN)
        GPIO.setup(self.US_trigg, GPIO.OUT)


        GPIO.setup(self.OD_Right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.OD_Left, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Méthode executee a la sortie du with
        GPIO.cleanup()

    def LeftMotor(self, speed):
        if speed > 0:
            self.motor["MLDuty+"].start(speed)
            self.motor["MLDuty-"].start(0)

        if speed < 0:
            self.motor["MLDuty+"].start(0)
            self.motor["MLDuty-"].start(abs(speed))

    def RightMotor(self, speed):
            if speed > 0:
                self.motor["MRDuty+"].start(speed)
                self.motor["MRDuty-"].start(0)

            if speed < 0:
                self.motor["MRDuty+"].start(0)
                self.motor["MRDuty-"].start(abs(speed))

    def Move(self, speed, length):
        self.pathLenght_Right = 0;
        self.pathLenght_Left = 0;

        GPIO.add_event_detect(self.OD_Right, GPIO.RISING, self.PathLength_Right, bouncetime=20)
        GPIO.add_event_detect(self.OD_Left, GPIO.RISING, self.PathLength_Left, bouncetime=20)

        while self.pathLenght_Right < length:
            self.LeftMotor(speed)
            self.RightMotor(speed)
            if self.StopAlert(20):
               while self.StopAlert(20):
                   self.Stop()

        self.Stop()

    def Stop(self):
        self.motor["MRDuty+"].stop()
        self.motor["MRDuty-"].stop()
        self.motor["MLDuty+"].stop()
        self.motor["MLDuty-"].stop()

    def Turn(self, angle, bendRadius):
        """
        Robot turn of max 180°
        Turn of 90° is equals to turn of 90° on the left and -90° is equals to a turn of 90° on the right
        """

        wheelRatio = bendRadius/(bendRadius+16)  # wheelRatio < 1
        lenghtToDo = 2*math.pi()*bendRadius*(angle/360)

        if wheelRatio > 0:  # left turn -> right wheel faster than left one
            while self.pathLenght_Right < lenghtToDo:
                self.RightMotor(75)
                self.LeftMotor(wheelRatio*75)
                if self.StopAlert(20):
                    while self.StopAlert(20):
                        self.Stop()

        else:
            while self.pathLenght_Right < lenghtToDo:
                self.RightMotor(wheelRatio*75)
                self.LeftMotor(75)
                if self.StopAlert(20):
                    while self.StopAlert(20):
                        self.Stop()

        self.Stop()

    def PathLength_Right(self):
        # 20 rising edge corresponds to 7cm*pi = 21.99 cm
        self.pathLenght_Right += 21.99/20
        return self.pathLenght_Right

    def PathLength_Left(self):
        # 20 rising edge corresponds to 7cm*pi = 21.99 cm
        self.pathLenght_Left += 21.99/20
        return self.pathLenght_Left

    #ToDO : Algorithm to make several measure and have a more reliable distance value
    def StopAlert(self, trig):
        GPIO.output(self.US_trigg, False)
        time.sleep(0.1)
        GPIO.output(self.US_trigg, True)
        time.sleep(0.00001)
        GPIO.output(self.US_trigg, False)
        start = time.time()
        while GPIO.input(self.US_echo) == 0:
            start = time.time()
        while GPIO.input(self.US_echo) == 1:
            stop = time.time()

        elapsed = stop - start
        distance = (elapsed * 34000)/2

        if distance < trig:
            return True

        return False


# ListPin sensor = [US_echo, US_trig, OD_Right, OD_Left]

with EmToRobot((23, 22), (4, 17), [25, 24, 21, 18]) as robot:
    robot.Move(100, 1000)
    #print(robot.DetectSensor())
