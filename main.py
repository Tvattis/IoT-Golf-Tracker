import machine
from machine import Pin
import time
from micropyGPS import MicropyGPS
from bno055 import *
import lcd_4bit_mode
from mqtt import MQTTClient
import keys
import wifiConnection
import ujson
import math

# On board LED setup------------------------------------------------------------------------------------
led = Pin("LED", Pin.OUT) 

# LCD setup ------------------------------------------------------------------------------------
RS = machine.Pin(2,machine.Pin.OUT)
ENABLE = machine.Pin(3,machine.Pin.OUT)
BACK_LIGHT = machine.Pin(0,machine.Pin.OUT) #hoppas att det fungerar utan detta
D4 = machine.Pin(4,machine.Pin.OUT)
D5 = machine.Pin(5,machine.Pin.OUT)
D6 = machine.Pin(6,machine.Pin.OUT)
D7 = machine.Pin(7,machine.Pin.OUT)
display = lcd_4bit_mode.LCD16x2(RS,ENABLE,BACK_LIGHT,D4,D5,D6,D7)
lcd_lock = False
lcd_lock_until = 0
lcd_lock_time = 4000 # 4 seconds lock

def lock_lcd():
    global lcd_lock, lcd_lock_until, lcd_lock_time
    lcd_lock = True
    lcd_lock_until = time.ticks_add(time.ticks_ms(), lcd_lock_time)


# button and interrupt setup and functions------------------------------------------------------------------------------------
button = Pin(13, Pin.IN, Pin.PULL_DOWN)
button_pressed = False
button_state = button.value()
button_released = False

def button_INT(pin):
    global button_state, button_pressed, button_released
    button.irq(handler=None)

    if (button.value() == 1) and (button_state == 0):
        button_state = 1
        button_pressed = True # dont do heavy lifting in the interruption handler
        #print("Button pressed - saving GPS data")
        #led.toggle()
        # save_gps_to_file()

    elif (button.value() == 0) and (button_state == 1):
        button_state = 0
        button_released = True

    button.irq(handler=button_INT)

button.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=button_INT)


# MQTT setup and functions -------------------------------------------------------------------------------------
#client = MQTTClient(keys.AIO_CLIENT_ID, keys.AIO_SERVER, keys.AIO_PORT, keys.AIO_USER, keys.AIO_KEY)

def connect_mqtt():
    try:
        ip = wifiConnection.connect()
        print("WiFi connected, IP:", ip)
        global client
        client = MQTTClient(keys.AIO_CLIENT_ID, keys.AIO_SERVER, keys.AIO_PORT, keys.AIO_USER, keys.AIO_KEY)
        client.connect()
        print("Connected to MQTT broker")
        return True
    except Exception as e:
        print("Connection error:", e)
        return False
    
def publish_calculations():
    button.irq(handler=None) # No interupts while reading or writing to file
    try:
        with open("computed.txt", "r") as f:
            lines = f.readlines()
    except OSError:
        print("No computed.txt file found.")
        display.ClearScreenCursorHome()
        display.WriteLine("MQTT Upload fail", 1)
        display.WriteLine("No data file", 2)
        lock_lcd()
        button.irq(handler=button_INT)
        return

    if not lines:
        print("No calculations to send.")
        display.ClearScreenCursorHome()
        display.WriteLine("MQTT Upload fail", 1)
        display.WriteLine("No data to send", 2)
        lock_lcd()
        button.irq(handler=button_INT)
        return

    unsent_lines = []
    sent_lines = []
    total_lines = len(lines)
    display.ClearScreenCursorHome()
    display.WriteLine("MQTT Upload", 1)
    lock_lcd()

    for idx, line in enumerate(lines, start=1):
        try:
            
            # Convert CSV to JSON of right format
            parts = line.strip().split(',')
            try:
                locationA = ujson.dumps({
                    "value": 1337,
                    "lat": parts[1],
                    "lon": parts[2],
                    "ele": 0
                })
                locationB = ujson.dumps({
                    "value": 1337,
                    "lat": parts[3],
                    "lon": parts[4],
                    "ele": 0
                })
                locationC = ujson.dumps({
                    "value": 1337,
                    "lat": parts[5],
                    "lon": parts[6],
                    "ele": 0
                })

                shot = "Shot Number {}".format(parts[0])
                distance = "Shot distance: {} m".format(parts[7])
                pei_procent = float(parts[10]) * 100
                pei = "Shot PEI: {}%".format(pei_procent)

                client.publish(keys.AIO_GPS_POSITION_FEED, locationA)
                time.sleep(0.1)
                client.publish(keys.AIO_GPS_POSITION_FEED, locationB)
                time.sleep(0.1)
                client.publish(keys.AIO_GPS_POSITION_FEED, locationC)
                time.sleep(0.1)
                client.publish(keys.AIO_GOLF_STATISTICS_FEED, pei)
                time.sleep(0.1)
                client.publish(keys.AIO_GOLF_STATISTICS_FEED, distance)
                time.sleep(0.1)
                client.publish(keys.AIO_GOLF_STATISTICS_FEED, shot)
                

                print("Published JSONs:", locationA, locationB, locationC)
                print("Published further data: ", shot, distance, pei)
                sent_lines.append(line)
                display.WriteLine("Sent {}/{}".format(idx, total_lines), 2)
                lock_lcd()
                time.sleep(0.1)
            except ValueError as ve:
                print("Invalid GPS data format:", ve)
                continue

        except Exception as e:
            print("Failed to publish:", e)
            unsent_lines.extend(lines[idx-1:])  # Save current + remaining lines
            display.WriteLine("Upload Failed!", 2)
            lock_lcd()
            button.irq(handler=button_INT)
            break
    else:
        # All lines sent successfully
        display.WriteLine("Upload Complete", 2)
        lock_lcd()

    # Write unsent lines back to gps_log.txt
    try:
        with open("computed.txt", "w") as f:
            for l in unsent_lines:
                f.write(l)
    except Exception as e:
        print("unsent lines not saved:", e)
        display.WriteLine("sent, save error", 2)
        lock_lcd()
        button.irq(handler=button_INT)

    # Append sent lines to computed_sent.txt
    if sent_lines:
        try:
            with open("computed_sent.txt", "a") as f_sent:
                for l in sent_lines:
                    f_sent.write(l)
        except Exception as e:
            print("Failed to write sent lines:", e)
            display.WriteLine("File Save Error", 2)
            lock_lcd()
            button.irq(handler=button_INT)
    button.irq(handler=button_INT)

# GPS setup and functions ------------------------------------------------------------------------------------
gps_uart = machine.UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
gps = MicropyGPS(location_formatting='dd')

def read_GPS():
    sentence_type = None
    while gps_uart.any():
        char = gps_uart.read(1)
        if char:
            try:
                decoded = char.decode('utf-8')
                sentence_type = gps.update(decoded)
            except UnicodeError:
                pass  # Skip non-UTF8 data
            except Exception as e:
                print("GPS parsing error:", e)
    return sentence_type

def save_gps_to_file(angle):
    if gps.fix_type < 2:
        display.ClearScreenCursorHome()
        display.WriteLine("No GPS Fix", 1)
        display.WriteLine("Save Failed!", 2)
        print("Save failed: No GPS fix")
        
        lock_lcd()
        return False

    lat_str = gps.latitude_string()
    lon_str = gps.longitude_string()
    fix_type = gps.fix_type
    sats = gps.satellites_in_use
    time_utc = gps.timestamp
    hdop_str = gps.hdop
    altitude = gps.altitude

    try:
        # Clean and convert latitude to float
        if "° N" in lat_str:
            lat = float(lat_str.replace("° N", "").strip())
        elif "° S" in lat_str:
            lat = -float(lat_str.replace("° S", "").strip())
        else:
            raise ValueError("Invalid latitude format")

        # Clean and convert longitude to float
        if "° E" in lon_str:
            lon = float(lon_str.replace("° E", "").strip())
        elif "° W" in lon_str:
            lon = -float(lon_str.replace("° W", "").strip())
        else:
            raise ValueError("Invalid longitude format")
    except ValueError as ve:
        print("Failed to parse GPS coordinates:", ve)
        display.ClearScreenCursorHome()
        display.WriteLine("GPS Format Err", 1)
        display.WriteLine("Save Failed!", 2)
        lock_lcd()
        return False

    # make the timestamp into a readable format without any commas
    if time_utc and len(time_utc) == 3:
        time_str = "{:02}:{:02}:{:02}".format(*time_utc)
    else:
        time_str = "00:00:00"  # fallback if GPS hasn't locked time

    line = "{},{},{},{},{},{},{},{}\n".format(time_str, lat, lon, altitude, angle, fix_type, sats, hdop_str)

    button.irq(handler=None) # no interrupt while writing to file
    try:
        with open("gps_log.txt", "a") as f:
            f.write(line)
        display.ClearScreenCursorHome()
        display.WriteLine("GPS & ANGL SAVED", 1)
        line2 = "ANGLE: {}".format(angle)
        display.WriteLine(line2, 2)
        lock_lcd()
        print("GPS position saved:", line.strip())

        button.irq(handler=button_INT)
        return True
    except Exception as e:
        display.ClearScreenCursorHome()
        display.WriteLine("File Error", 1)
        display.WriteLine("Save Failed!", 2)
        print("Failed to save GPS data:", e)
        lock_lcd()

        button.irq(handler=button_INT)
        return False


# BNO055 setup and related functions------------------------------------------------------------------------------------
i2c = machine.I2C(1, sda=machine.Pin(26), scl=machine.Pin(27))
imu = BNO055(i2c, address=0x29)

def calibrate_imu(imu):
    calibrated = False
    while not calibrated:
        time.sleep(1)
        calibrated = imu.calibrated()
        sys, gyro, accel, mag = imu.cal_status()
        
        line1 = 'CALIBRATING IMU'
        line2 = 'S:{} G:{} A:{} M:{}'.format(sys, gyro, accel, mag)

        # Ensure lines are max 16 characters (pad or trim if needed)
        display.ClearScreenCursorHome()
        display.WriteLine(line1, 1)
        display.WriteLine(line2, 2)

def save_imu_calibration(filename="/calibration.dat"):
    global imu
    offsets = imu.sensor_offsets()
    with open(filename, "wb") as f:
        f.write(offsets)
    display.ClearScreenCursorHome()
    display.WriteLine("Calibration", 1)
    display.WriteLine("Saved!", 2)

def load_imu_calibration(filename="/calibration.dat"):
    global imu
    try:
        with open(filename, "rb") as f:
            offsets = f.read()
            if len(offsets) == 22:  # Check correct size
                imu.set_offsets(offsets)
                display.ClearScreenCursorHome()
                display.WriteLine("Calibration", 1)
                display.WriteLine("loaded", 2)
                return True
            else:
                print("Invalid offset file size.")
                return False
    except OSError:
        print("Calibration file not found.")
        return False

def get_heading():
    global imu
    heading, roll, pitch = imu.euler()
    return heading

# Main Loop funcions ------------------------------------------------------------------------------------
load_imu_calibration()
last_LCD_print = time.ticks_ms()
LAST_MQTT_UPLOAD = 0
MQTT_UPLOAD_INTERVAL = 60000  # 60 seconds
initial_heading = 400
GPS_count = 0
shot_count = 0

def print_gps_status_LCD():
    # Get and clean GPS data
    lat_dd = gps.latitude_string().replace("°", "").replace("'", "").replace(" ", "")
    lon_dd = gps.longitude_string().replace("°", "").replace("'", "").replace(" ", "")
    sats = gps.satellites_in_use
    
    # time_utc = gps.timestamp  # (hh, mm, ss)
    # time_str = "{:02}:{:02}:{:02}".format(*time_utc) if time_utc and len(time_utc) == 3 else "--:--:--"

    # Fix string conversion
    fix_map = {1: "0D", 2: "2D", 3: "3D"}
    fix_str = fix_map.get(gps.fix_type, "?")

    hdop_str = "{:.1f}".format(gps.hdop) if gps.hdop else "?"

    # Display lines
    gpsline1 = "{} {}".format(lat_dd[:7], lon_dd[:7])        # Line 1: Lat Lon
    gpsline2 = "{} S{} HD{}".format(fix_str, sats, hdop_str)  # Fix type, Sats, HDOP

    # Show on LCD
    display.ClearScreenCursorHome()
    display.WriteLine(gpsline1, 1)
    display.WriteLine(gpsline2, 2)

def compute_third_point(lat1, lon1, angle1, lat2, lon2, angle2):
    """
    Given two GPS points (lat1, lon1) and (lat2, lon2), and angles from each point
    toward a third point, compute the third point's latitude/longitude and distances.
    This is not the most accurate algorithm, but it will get the job done within atleast
    1 m accuracy

    Args:
        lat1, lon1: Latitude and longitude of point A
        angle1: Angle from A toward unknown point C (in degrees, left positive)
        lat2, lon2: Latitude and longitude of point B
        angle2: Angle from B toward unknown point C (in degrees, right negative)

    Returns:
        (latC, lonC, distanceAB, distanceAC, distanceBC): Coordinates of point C and distances of triangle sides
    """

    def haversine_distance(lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        rad_lat1 = math.radians(lat1)
        rad_lat2 = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        a = math.sin(delta_lat / 2)**2 + math.cos(rad_lat1) * math.cos(rad_lat2) * math.sin(delta_lon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def destination_point(lat, lon, distance, bearing_deg):
        R = 6371000
        bearing_rad = math.radians(bearing_deg)
        lat_rad = math.radians(lat)
        delta_x = distance * math.sin(bearing_rad)
        delta_y = distance * math.cos(bearing_rad)
        new_lat = lat + math.degrees(delta_y / R)
        new_lon = lon + math.degrees(delta_x / (R * math.cos(lat_rad)))
        return new_lat, new_lon

    def initial_bearing(lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        x = math.sin(delta_lon) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon)
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360

    def triangle_distances(ABdist, Adeg, Bdeg):
        A = math.radians(Adeg)
        B = math.radians(Bdeg)
        triangle_angle = math.radians(180) - (abs(A) + abs(B))
        BC = math.sin(A) / math.sin(triangle_angle) * ABdist
        AC = math.sin(B) / math.sin(triangle_angle) * ABdist
        return abs(AC), abs(BC)

    # Step 1: Distance between A and B
    distanceAB = haversine_distance(lat1, lon1, lat2, lon2)

    # Step 2: Compute triangle side distances
    distanceAC, distanceBC = triangle_distances(distanceAB, angle1, angle2)

    # Step 3: Calculate bearing from A to B
    bearingAB = initial_bearing(lat1, lon1, lat2, lon2)
    bearingAC = (bearingAB + angle1) % 360

    # Step 4: Project from A to C
    latC, lonC = destination_point(lat1, lon1, distanceAC, bearingAC)

    return latC, lonC, distanceAB, distanceAC, distanceBC

def Calc_stats():
    # Perhaps add return true/false if it succeeds or fails
    # Take two lines of raw data and make calculations that then will be stored for later publishing
    global shot_count
    shot_count += 1
    button.irq(handler=None) # No interupts while reading or writing to file
    
    try:
        with open("gps_log.txt", "r") as f:
            lines = f.readlines()
    except OSError:
        print("No GPS log file found for calculation.")
        button.irq(handler=button_INT)
        return
    
    if len(lines) < 2:
        print("Not enough lines in the file for calculation")
        button.irq(handler=button_INT)
        return
    
    button.irq(handler=button_INT) # reenable after file handling done

    # Parse the first two lines as lists by splitting on commas
    location1 = lines[0].strip().split(",")
    location2 = lines[1].strip().split(",")

    # negative angle beacause the calculation and recording of data uses different directions
    latC, lonC, shotDistance, targetDistance, errorDistance = compute_third_point(float(location1[1]), float(location1[2]), -float(location1[4]),
                                                                                  float(location2[1]), float(location2[2]), -float(location2[4]))
    
    Pei = errorDistance / targetDistance

    line = "{},{},{},{},{},{},{},{},{},{},{}\n".format(shot_count, location1[1], location1[2], location2[1], location2[2], latC, lonC, shotDistance, targetDistance, errorDistance, Pei)

    print("From A:", location1[1], location1[2], "angle:", location1[3])
    print("From B:", location2[1], location2[2], "angle:", location2[3])
    print("Result C:", latC, lonC)

    button.irq(handler=None) # no interrupt while writing to file
    
    # Append used lines to gps_used.txt
    try:
        with open("gps_used.txt", "a") as f_used:
            f_used.write(lines[0])
            f_used.write(lines[1])
    except Exception as e:
        print("Failed to write used gps data after calculation:", e)
        display.WriteLine("File Save Error", 1)
        display.WriteLine("used gps calcDat", 2)
        lock_lcd()
        button.irq(handler=button_INT)


    # clear the gps log of the used lines:
    try:
        with open("gps_log.txt", "w") as f:
            for line in lines[2:]:  # Keep remaining lines, if any
                f.write(line) 
    except Exception as e:
        print("gps log not cleared:", e)
        display.WriteLine("sent, save error", 2)
        lock_lcd()
        button.irq(handler=button_INT)

    # write the calculated numbers to file
    try:
        with open("computed.txt", "a") as f:
            f.write(line)
        display.ClearScreenCursorHome()
        display.WriteLine("Calcs saved", 1)
        line2 = "Shot: {} Pei:{}".format(shot_count, Pei * 100)
        display.WriteLine(line2, 2)
        lock_lcd()
        print("calculated data saved:", line.strip())

        button.irq(handler=button_INT)
        return True
    except Exception as e:
        display.ClearScreenCursorHome()
        display.WriteLine("File Error Calc", 1)
        display.WriteLine("Save Failed!", 2)
        print("Failed to save calculated data:", e)
        lock_lcd()

        button.irq(handler=button_INT)
        return False
    

    

# Main Loop  ------------------------------------------------------------------------------------
while True:
    # idle display, until interrupted by button
    read_GPS()
    now = time.ticks_ms()

    # LCD update
    if time.ticks_diff(now, last_LCD_print) > 1000:
        # Only update LCD if not locked or lock expired
        if not lcd_lock or time.ticks_diff(now, lcd_lock_until) > 0:
            lcd_lock = False  # Unlock LCD updates
            print_gps_status_LCD()
            # print("Latitude:", gps.latitude_string())
            # print("Longitude:", gps.longitude_string())
            # print("Fix Type:", gps.fix_type)
            # print("Satellites in use:", gps.satellites_in_use)
            # print("-" * 40)
            last_LCD_print = now

    if GPS_count > 1:
        Calc_stats() #we have not handled the case where there are more than 2 lines of data
        GPS_count = 0

    # Periodic MQTT upload
    if time.ticks_diff(now, LAST_MQTT_UPLOAD) > MQTT_UPLOAD_INTERVAL:
        display.ClearScreenCursorHome()
        display.WriteLine("Connecting...", 1)
        lock_lcd()
        if connect_mqtt():
            publish_calculations()
            client.disconnect()
            wifiConnection.disconnect()
        else:
            print("Skipping MQTT upload: no connection")
            display.WriteLine("failed to connec", 2)
            lock_lcd()
        LAST_MQTT_UPLOAD = now

    # Action that should take place when the button is pressed
    if button_pressed:
        button_pressed = False
        led.toggle()
        print("Button pressed - recording heading")
        initial_heading = get_heading() # record the heading currently pointing to
        # Start looking for the most accurate gps-positoin (perhaps average?)
        
        display.ClearScreenCursorHome()
        display.WriteLine("measurment start", 1)
        line2 = "in.head: {}".format(initial_heading)
        display.WriteLine(line2, 2)
        lock_lcd()

    # Actions that should take place when the button is released
    if button_released:
        button_released = False
        led.toggle()
        print("the button was released")
        new_heading = get_heading()        # record the new heading
        angle = (new_heading - initial_heading + 180) % 360 - 180 # Positive = clockwise (right turn), Negative = counter-clockwise (left turn).
        initial_heading = 4000 # the heading should be over 360 so that we know that it is wrong (not implemented correctly now)
        # Stop searching for the best GPS-data, choose the best one (not happening in this verison)
        if angle == 0:
            display.ClearScreenCursorHome()
            display.WriteLine("angle = 0", 1)
            display.WriteLine("Measure again!", 2)
        else:
            save_gps_to_file(angle) # Record gps and angle for future broadcast to MQTT (or calculation)(some sort of function simmilar to "save_gps_to_file()")
            GPS_count += 1

    time.sleep(0.01)