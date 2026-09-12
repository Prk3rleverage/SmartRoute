def move_forward(left_motor, right_motor, speed):
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)


def stop(left_motor, right_motor):
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def turn_left(left_motor, right_motor, speed):
    left_motor.setVelocity(-speed)
    right_motor.setVelocity(speed)


def turn_right(left_motor, right_motor, speed):
    left_motor.setVelocity(speed)
    right_motor.setVelocity(-speed)

def correct_left(left_motor, right_motor):
    left_motor.setVelocity(2.5)
    right_motor.setVelocity(3.0)

def correct_right(left_motor, right_motor):
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(2.5)