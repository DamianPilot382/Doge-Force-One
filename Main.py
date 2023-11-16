import board
import csv
import digitalio
import time

import adafruit_gps # GPS
import adafruit_lsm9ds1 # IMU
import adafruit_mcp9808 # Temperature
import adafruit_mpl3115a2 # Altitude


# Sensor Initialization
i2c = board.I2C()

tempSensor = adafruit_mcp9808.MCP9808(i2c)

altSensor = adafruit_mpl3115a2.MPL3115A2(i2c)
altSensor.sealevel_pressure = 1022.5

imuSensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

# Set update rate to once a second (1hz)
gps.send_command(b"PMTK220,1000")

csvFile = open('test.csv', 'w')
csvWriter = csv.writer(csvFile)

headers = ['Time', 'Temperature', 'Pressure', 'Altitude',
           'AccelX', 'AccelY', 'AccelZ',
           'MagX', 'MagY', 'MagZ',
           'GyroX', 'GyroY', 'GyroZ',
           'GPSFix', 'GPSLat', 'GPSLong']

csvWriter.writerow(headers)

startTime = time.time()

timestamp = time.time() - startTime

while timestamp < 60:

    print("=" * 40)

    gps.update()

    timestamp = time.time() - startTime

    print("Time: ", timestamp)

    # Temperature
    tempC = tempSensor.temperature
    tempF = tempC * 9 / 5 + 32
    print("Temperature: {} C {} F ".format(tempC, tempF))
    
    # Altimeter
    pressure = altSensor.pressure
    print("Pressure: {0:0.3f} hectopascals".format(pressure))
    altitude = altSensor.altitude
    print("Altitude: {0:0.3f} meters".format(altitude))
    
    
    # IMU
    print("IMU:")
    accel_x, accel_y, accel_z = imuSensor.acceleration
    mag_x, mag_y, mag_z = imuSensor.magnetic
    gyro_x, gyro_y, gyro_z = imuSensor.gyro
    print("    Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})".format(accel_x, accel_y, accel_z))
    print("    Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})".format(mag_x, mag_y, mag_z))
    print("    Gyroscope (rad/sec): ({0:0.3f},{1:0.3f},{2:0.3f})".format(gyro_x, gyro_y, gyro_z))
    
    print("GPS:")
    if gps.has_fix:
        lat = gps.latitude
        long = gps.longitude
        # Print out details about the fix like location, date, etc.
        print("    Latitude: {0:.6f} degrees".format(lat))
        print("    Longitude: {0:.6f} degrees".format(long))
        print("    Fix quality: {}".format(gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        if gps.satellites is not None:
            print("# satellites: {}".format(gps.satellites))
        if gps.altitude_m is not None:
            print("Altitude: {} meters".format(gps.altitude_m))
        if gps.speed_knots is not None:
            print("Speed: {} knots".format(gps.speed_knots))
        if gps.track_angle_deg is not None:
            print("Track angle: {} degrees".format(gps.track_angle_deg))
        if gps.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(gps.horizontal_dilution))
        if gps.height_geoid is not None:
            print("Height geoid: {} meters".format(gps.height_geoid))

    else:
        print("    No fix")
        lat = 0
        long = 0


    gps.update()

    csvWriter.writerow([timestamp, tempC, pressure, altitude,
                       accel_x, accel_y, accel_z,
                       mag_x, mag_y, mag_z,
                       gyro_x, gyro_y, gyro_z,
                       gps.has_fix, lat, long])

    time.sleep(0.5)