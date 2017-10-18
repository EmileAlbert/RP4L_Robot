from EmToRobot_V2 import EmToRobot

with EmToRobot("config.yaml") as robot:
    robot.move(100, 100)
    print("robot.move(100, 100)")
    robot.Turn(90, 30)
    print("robot.Turn(-90, 30)")
    robot.move(60, 50)
    print("robot.move(100, 50)")
    robot.Turn(-90, 30)
    print("robot.Turn(-90, 30)")
    robot.move(-60, 100)
    print("robot.move(-100, 100)")
    print("test setup")