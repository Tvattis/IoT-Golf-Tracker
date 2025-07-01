import ubinascii              # Conversions between binary data and various encodings
import machine                # To Generate a unique id from processor

# Wireless network
WIFI_SSID = "wifi name"
WIFI_PASS = "wifi password"

# Adafruit IO (AIO) configuration
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_USER = ""
AIO_KEY = ""
AIO_CLIENT_ID = ubinascii.hexlify(machine.unique_id())  # Can be anything
AIO_GPS_POSITION_FEED = "USERNAME/feeds/gps-position/json"
AIO_GOLF_STATISTICS_FEED = "USERNAME/feeds/golf-statistics"