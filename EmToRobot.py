"""
TODO
 - PWM motor
 - Distance (encodeur)
 - Arret (US)
"""

# coding: utf-8
import RPi.GPIO as GPIO
import sys
import time


class EmToRobot(object):
    def __init__(self, PinsMotorLeft, PinsMotorRight, listPinSensor):
        self.motor = {"MotorLeft": PinsMotorLeft,
                      "MotorRight": PinsMotorRight}
        self.listPinSensor = listPinSensor

    def __enter__(self):
        # Méthode executee à l'appel après le init
        GPIO.setmode(GPIO.BCM)

        for entry in self.motor:
            # Initialisation des deux couples de pins moteur
            GPIO.setup(self.motor[entry][0], GPIO.OUT)
            GPIO.setup(self.motor[entry][1], GPIO.OUT)

        self.motor["MLDuty+"] = GPIO.PWM(self.motor["MotorLeft"][0], 1)
        self.motor["MLDuty-"] = GPIO.PWM(self.motor["MotorLeft"][1], 1)
        self.motor["MRDuty+"] = GPIO.PWM(self.motor["MotorRight"][0], 1)
        self.motor["MRDuty-"] = GPIO.PWM(self.motor["MotorRight"][1], 1)

        for pin in self.listPinSensor:
            GPIO.setup(pin, GPIO.IN)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Méthode executee a la sortie du with
        GPIO.cleanup()

    def LeftMotor(self, duty):
        if duty > 0:
            self.motor["MLDuty+"].ChangeDutyCycle(duty)
            self.motor["MLDuty-"].ChangeDutyCycle(1 - duty)

        elif dir < 0:
            self.motor["MLDuty+"].ChangeDutyCycle(1 - duty)
            self.motor["MLDuty-"].ChangeDutyCycle(duty)

        elif duty == 0:
            GPIO.output(self.motorPins["MotorLeft"][0], GPIO.LOW)
            GPIO.output(self.motorPins["MotorLeft"][1], GPIO.LOW)

    def RightMotor(self, duty):
        if duty > 0:
            self.motor["MRDuty+"].ChangeDutyCycle(duty)
            self.motor["MRDuty-"].ChangeDutyCycle(1 - duty)

        elif dir < 0:
            self.motor["MRDuty+"].ChangeDutyCycle(1 - duty)
            self.motor["MRDuty-"].ChangeDutyCycle(duty)

        elif duty == 0:
            GPIO.output(self.motorPins["MotorRight"][0], GPIO.LOW)
            GPIO.output(self.motorPins["MotorRight"][1], GPIO.LOW)

    def MoveForward(self, duration):
        self.LeftMotor(1)
        self.RightMotor(1)
        time.sleep(duration)

    """
    def MoveBackward(self, duration):
        self.LeftMotor("B")
        self.RightMotor("B")
        time.sleep(duration)

    def Turn(self, direction, duration):
        if direction == "R":
            self.LeftMotor("F")
            self.RightMotor("B")

        elif direction == "L":
            self.LeftMotor("B")
            self.RightMotor("F")

        time.sleep(duration)

    """

print("test")
with EmToRobot((22, 23), (25, 24), []) as robot:
    robot.MoveForward(5)

