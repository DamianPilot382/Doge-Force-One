import board
import csv
import digitalio
import os
import time

import adafruit_gps # GPS
import adafruit_lsm9ds1 # IMU
import adafruit_mcp9808 # Temperature
import adafruit_mpl3115a2 # Altitude


# Sensor Initialization
i2c = board.I2C()

# Temperature
tempSensor = adafruit_mcp9808.MCP9808(i2c)

# Altitude
altSensor = adafruit_mpl3115a2.MPL3115A2(i2c)
altSensor.sealevel_pressure = 1022.5

# IMU
imuSensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# GPS
gpsSensor = adafruit_gps.GPS_GtopI2C(i2c, debug=False)

# Turn on the basic GGA and RMC info
gpsSensor.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Set update rate to once a second (1hz)
gpsSensor.send_command(b"PMTK220,1000")

# Create a new numbered results CSV file
inc = 0
while os.path.exists('Results/Results_%s.csv' % inc):
    inc += 1

# Open the CSV file
csvFile = open('Results/Results_%s.csv' % inc, 'w')
csvWriter = csv.writer(csvFile)

# Create headers for CSV File
csvWriter.writerow(['Time',
                    'Temperature', 'Pressure', 'Altitude',
                    'AccelX', 'AccelY', 'AccelZ',
                    'MagX', 'MagY', 'MagZ',
                    'GyroX', 'GyroY', 'GyroZ',
                    'GPSFix', 'GPSLat', 'GPSLong'])

startTime = time.time()

# Loop forever or until interrupted
while True:

    # Print separator
    print("=" * 40)

    # Update the GPS (done twice per loop)
    gpsSensor.update()

    # Get the new timestamp and print it
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
    
    # GPS
    print("GPS:")

    # Check if the GPS has a fix
    if gpsSensor.has_fix:
        lat = gpsSensor.latitude
        long = gpsSensor.longitude
        print("    Latitude: {0:.6f} degrees".format(lat))
        print("    Longitude: {0:.6f} degrees".format(long))
        print("    Fix quality: {}".format(gpsSensor.fix_quality))

        # Print extra information
        if gpsSensor.satellites is not None:
            print("# satellites: {}".format(gpsSensor.satellites))
        if gpsSensor.altitude_m is not None:
            print("Altitude: {} meters".format(gpsSensor.altitude_m))
        if gpsSensor.speed_knots is not None:
            print("Speed: {} knots".format(gpsSensor.speed_knots))
        if gpsSensor.track_angle_deg is not None:
            print("Track angle: {} degrees".format(gpsSensor.track_angle_deg))
        if gpsSensor.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(gpsSensor.horizontal_dilution))
        if gpsSensor.height_geoid is not None:
            print("Height geoid: {} meters".format(gpsSensor.height_geoid))
    
    # If no GPS fix
    else:
        print("    No fix")
        lat = 0
        long = 0

    # Update the GPS Sensor again
    gpsSensor.update()

    # Write the results for this iteration to the CSV file
    csvWriter.writerow([timestamp, tempC, pressure, altitude,
                       accel_x, accel_y, accel_z,
                       mag_x, mag_y, mag_z,
                       gyro_x, gyro_y, gyro_z,
                       gpsSensor.has_fix, lat, long])