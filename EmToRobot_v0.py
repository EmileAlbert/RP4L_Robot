# coding: utf-8
import RPi.GPIO as GPIO
import sys
import time


class EmToRobot(object):
    def __init__(self, PinsMotorLeft, PinsMotorRight, listPinSensor):
        self.motorPins = {"MotorLeft": PinsMotorLeft, "MotorRight": PinsMotorRight}
        self.listPinSensor = listPinSensor

    def __enter__(self):
        # Méthode executee à l'appel après le init
        GPIO.setmode(GPIO.BCM)

        for entry in self.motorPins:
            # Initialisation des deux couples de pins moteur
            GPIO.setup(self.motorPins[entry][0], GPIO.OUT)
            GPIO.setup(self.motorPins[entry][1], GPIO.OUT)

        for pin in self.listPinSensor:
            GPIO.setup(pin, GPIO.IN)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Méthode executee a la sortie du with
        GPIO.cleanup()

    def LeftMotor(self, dir):
        if (dir == "F"):
            GPIO.output(self.motorPins["MotorLeft"][0], GPIO.LOW)
            GPIO.output(self.motorPins["MotorLeft"][1], GPIO.HIGH)
        elif (dir == "B") :
            GPIO.output(self.motorPins["MotorLeft"][0], GPIO.HIGH)
            GPIO.output(self.motorPins["MotorLeft"][1], GPIO.LOW)

        else :
            GPIO.output(self.motorPins["MotorLeft"][0], GPIO.HIGH)
            GPIO.output(self.motorPins["MotorLeft"][1], GPIO.LOW)

    def RightMotor(self, dir):
        if (dir == "F"):
            GPIO.output(self.motorPins["MotorRight"][0], GPIO.LOW)
            GPIO.output(self.motorPins["MotorRight"][1], GPIO.HIGH)
        elif (dir == "B"):
            GPIO.output(self.motorPins["MotorRight"][0], GPIO.HIGH)
            GPIO.output(self.motorPins["MotorRight"][1], GPIO.LOW)

        else:
            GPIO.output(self.motorPins["MotorRight"][0], GPIO.HIGH)
            GPIO.output(self.motorPins["MotorRight"][1], GPIO.LOW)

    def MoveForward(self, duration):
        print("moveF")
        self.LeftMotor("F")
        self.RightMotor("F")


        time.sleep(duration)

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