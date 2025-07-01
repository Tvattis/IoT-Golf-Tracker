#WifiConnections.py
import keys
import network
from time import sleep

def connect():
    wlan = network.WLAN(network.STA_IF)

    print("Waiting before enabling WiFi...")
    sleep(0.5)  # Give time after boot

    if not wlan.active():
        wlan.active(True)

    # Wait for chip to fully power up
    for _ in range(10):  # max 1s
        if wlan.active():
            break
        sleep(0.1)

    if not wlan.active():
        raise RuntimeError("WiFi chip failed to activate")

    print('Connecting to network...')
    try:
        wlan.connect(keys.WIFI_SSID, keys.WIFI_PASS)
    except OSError as e:
        print("Failed to start WiFi connection:", e)
        raise

    # Wait for connection
    print('Waiting for connection', end='')
    timeout = 20
    while not wlan.isconnected() and wlan.status() >= 0 and timeout > 0:
        print('.', end='')
        sleep(1)
        timeout -= 1

    if not wlan.isconnected():
        raise RuntimeError("WiFi connection timeout or failed")

    ip = wlan.ifconfig()[0]
    print('\nConnected on {}'.format(ip))
    return ip



def disconnect():
    wlan = network.WLAN(network.STA_IF)         # Put modem on Station mode
    wlan.disconnect()
    wlan = None 

    