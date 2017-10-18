# coding: utf-8
import RPi.GPIO as GPIO
import sys
import time
import yaml
import math

#Graphics Method
def ok():
    print("["+'\033[92m'+"  OK  "+'\033[0m'+"]", end=' ')

def fail():
    print("[" + '\033[91m' + " FAIL " + '\033[0m' + "]", end=' ')


class EmToRobot(object):
    def __init__(self, configfile):
        try :
            self.config = yaml.load(open(configfile))
            ok()
        except Exception:
            fail()
        print("Configuration file loaded")
        self.config['Sensors']['Odometers']['Distances'] = {}
        self.config['Sensors']['Odometers']['Distances']['Left_wheel'] = 0
        self.config['Sensors']['Odometers']['Distances']['Right_wheel'] = 0

    def __enter__(self):
        try :
            GPIO.setmode(GPIO.BCM)
            ok()
        except Exception:
            fail()
        print("Set GPIO Mode to BCM")

        # Setup Motors
        self.config['Motor']['PWM'] = {}
        for MotorConnection, PinNumber in self.config['Motor']['Connections'].items():
            try :
                GPIO.setup(PinNumber, GPIO.OUT)
                ok()
            except Exception as e:
                fail()
            print("Set up Motor-" + MotorConnection + " on GPIO " + str(PinNumber))
            try:
                self.config['Motor']['PWM'][MotorConnection] = GPIO.PWM(PinNumber, self.config['Motor']['PWMFrequency'])
                ok()
            except Exception as e:
                fail()
            print("Set up PWM of " + MotorConnection + " at " + str(self.config['Motor']['PWMFrequency']) + "Hz")

        #Setup Odometers
        for OdoConnections, PinNumber in self.config['Sensors']['Odometers']['Connections'].items():
            try :
                GPIO.setup(PinNumber, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                ok()
            except Exception as e:
                fail()
            print("Set up Odo-" + OdoConnections + " on GPIO " + str(PinNumber))

        GPIO.add_event_detect(self.config['Sensors']['Odometers']['Connections']['LeftPin'],
                              GPIO.RISING,callback=self.PathLength_Left,bouncetime=self.config['Sensors']['Odometers']['Bounce_time'])
                             #Todo Bounce time function of speed
        GPIO.add_event_detect(self.config['Sensors']['Odometers']['Connections']['RightPin'],
                              GPIO.RISING,callback=self.PathLength_Right,bouncetime=self.config['Sensors']['Odometers']['Bounce_time'])
        #Setup US Sensor
        try :
            GPIO.setup(self.config['Sensors']['Ultrasound']['Connections']['Trigg'], GPIO.OUT)
            ok()
        except Exception as e:
            fail()
        print("Set up US-Trigg on GPIO " + str(self.config['Sensors']['Ultrasound']['Connections']['Trigg']))

        try :
            GPIO.setup(self.config['Sensors']['Ultrasound']['Connections']['Echo'], GPIO.IN)
            ok()
        except Exception as e:
            fail()
        print("Set up US-ECHO on GPIO " + str(self.config['Sensors']['Ultrasound']['Connections']['Echo']))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Méthode executee a la sortie du with
        try :
            GPIO.cleanup()
            ok()
        except Exception as e:
            fail()
        print("Clean Up the GPIO")


    #Basic Move...
    def LeftMotor(self, speed):
        self.config['Motor']['PWM']['LeftPin+'].start((abs(speed) + speed) /2)
        self.config['Motor']['PWM']['LeftPin-'].start((abs(speed) - speed) / 2)

    def RightMotor(self, speed):
        self.config['Motor']['PWM']['RightPin+'].start((abs(speed) + speed)/2)
        self.config['Motor']['PWM']['RightPin-'].start((abs(speed) - speed) / 2)

    def Stop(self):
        for MotorPWM in self.config['Motor']['PWM']:
            self.config['Motor']['PWM'][MotorPWM].stop()


    #Complex Move
    def move(self, speed, length):
        for side in self.config['Sensors']['Odometers']['Distances']:
            self.config['Sensors']['Odometers']['Distances'][side] = 0
        right_speed = speed*0.85
        while self.config['Sensors']['Odometers']['Distances']['Left_wheel'] < length:
            regulation = (self.config['Sensors']['Odometers']['Distances']['Left_wheel'] -
                            self.config['Sensors']['Odometers']['Distances']['Right_wheel']*0.91) / self.config['Motor'][
                               'PwmRegulationFactor']
            if (right_speed+regulation) < 100.0:
                  right_speed  -= regulation
                  right_speed = round(right_speed,2)
            #print(regulation)
            self.LeftMotor(speed)
            self.RightMotor(right_speed)
            #print("speed :"+str(speed))
            #print("right speed:"+str(right_speed))

            if self.StopAlert(30):
               while self.StopAlert(30):
                   self.Stop()
        self.Stop()
        #time.sleep(1)




    def Turn(self, angle, bendRadius):
        """
        Robot turn of max 180°
        Turn of 90° is equals to turn of 90° on the left and -90° is equals to a turn of 90° on the right
        """
        for side in self.config['Sensors']['Odometers']['Distances']:
            self.config['Sensors']['Odometers']['Distances'][side] = 0
            print(str(side)+":"+str(self.config['Sensors']['Odometers']['Distances'][side]))
        wheelRatio = bendRadius/(bendRadius+16)  # wheelRatio < 1
        lenghtToDo = 6.28*bendRadius*(abs(angle)/360)
        print("lgtdo:"+str(lenghtToDo))

        if angle > 0:  # left turn -> right wheel faster than left one
            while self.config['Sensors']['Odometers']['Distances']['Right_wheel'] < lenghtToDo*0.85:
                print("Right")
                self.RightMotor(60*0.91)
                self.LeftMotor(wheelRatio*60)
                if self.StopAlert(20):
                    while self.StopAlert(20):
                        self.Stop()

        else:
            while self.config['Sensors']['Odometers']['Distances']['Left_wheel'] < lenghtToDo*1.25:
                print("Left")
                self.RightMotor(wheelRatio*60*0.91)
                self.LeftMotor(60)
                if self.StopAlert(30):
                    while self.StopAlert(30):
                        self.Stop()

        self.Stop()
        #time.sleep(1)

    def PathLength_Right(self,pin):
        # 20 rising edge corresponds to 7cm*pi = 21.99 cm
        self.config['Sensors']['Odometers']['Distances']['Right_wheel'] += 21.99/20
        #print("Right: "+str(self.config['Sensors']['Odometers']['Distances']['Right_wheel']))
        return self.config['Sensors']['Odometers']['Distances']['Right_wheel']

    def PathLength_Left(self,pin):
        # 20 rising edge corresponds to 7cm*pi = 21.99 cm
        self.config['Sensors']['Odometers']['Distances']['Left_wheel']  += 21.99/20
        #print("Left: " + str(self.config['Sensors']['Odometers']['Distances']['Left_wheel']))
        return self.config['Sensors']['Odometers']['Distances']['Left_wheel']

    #ToDO : Algorithm to make several and have a more reliable distance value
    def StopAlert(self, trig):
        GPIO.output(self.config['Sensors']['Ultrasound']['Connections']['Trigg'], False)
        time.sleep(0.1)
        GPIO.output(self.config['Sensors']['Ultrasound']['Connections']['Trigg'], True)
        time.sleep(0.00001)
        GPIO.output(self.config['Sensors']['Ultrasound']['Connections']['Trigg'], False)
        start = time.time()
        stop = time.time()
        while GPIO.input(self.config['Sensors']['Ultrasound']['Connections']['Echo']) == 0:
            start = time.time()
        while GPIO.input(self.config['Sensors']['Ultrasound']['Connections']['Echo']) == 1:
            stop = time.time()
        elapsed = stop - start
        distance = (elapsed * 34000)/2

        if distance < trig:
            return True

        return False
