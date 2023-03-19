import time
import json
import socket
from pymavlink import mavutil

adsbRxSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
adsbTx = mavutil.mavlink_connection('udpout:localhost:30006', source_system=254, source_component=1)


try:
    print("Opening Rx Socket")
    adsbRxSocket.bind(('',30005))
    lastHeartbeat = -999
    while True:
        # Send a MAVLink Heartbeat at Xs Intervals
        clockTime = time.monotonic()
        if clockTime - lastHeartbeat >= 1:
            adsbTx.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ADSB, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
            lastHeartbeat = clockTime
            print("HEARTBEAT: Heartbeat at time "+str(clockTime))
        message = adsbRxSocket.recvfrom(2048)
        #print(message[0])
        jsonObject = json.loads(message[0])
        if "aircraft" in jsonObject:
            try:
                icaoAddress = int(jsonObject["aircraft"][0]["icaoAddress"],16)
                latitude = int(jsonObject["aircraft"][0]["latDD"] * 1e7)
                longitude = int(jsonObject["aircraft"][0]["lonDD"] * 1e7)
                altitude_type = int(jsonObject["aircraft"][0]["altitudeType"])
                altitude = int(jsonObject["aircraft"][0]["altitudeMM"])
                heading = int(jsonObject["aircraft"][0]["headingDE2"])
                horVel = int(jsonObject["aircraft"][0]["horVelocityCMS"])
                verVel = int(jsonObject["aircraft"][0]["verVelocityCMS"])
                squawk = int(jsonObject["aircraft"][0]["squawk"])
                callsign = bytes(jsonObject["aircraft"][0]["callsign"], 'utf-8')
                emitterType = int(jsonObject["aircraft"][0]["emitterType"])
                adsbTx.mav.adsb_vehicle_send(icaoAddress, latitude, longitude, altitude_type, altitude, heading, horVel, verVel, callsign, emitterType, 0, 32768, squawk)
                print("Aircraft "+str(callsign)+" with Squawk "+str(squawk)+" passed to MAVLink")
            except:
                print("ERROR: Aircraft "+jsonObject["aircraft"][0]["icaoAddress"])
                
except KeyboardInterrupt:
    pass
finally:
    adsbRxSocket.close()
    print("Rx Socket Closed")
    
