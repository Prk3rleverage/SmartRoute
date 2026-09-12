def setup_sensors(robot, timestep):
    sensors = []

    for i in range(8):
        sensor = robot.getDevice("ps" + str(i))
        sensor.enable(timestep)
        sensors.append(sensor)

    return sensors


def read_sensors(sensors):
    sensor_values = []

    for sensor in sensors:
        sensor_values.append(sensor.getValue())

    return sensor_values