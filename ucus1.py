from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

dron = connect("127.0.0.1:14550", wait_ready=True)

def takeoff(hundurluk):
    while dron.is_armable is not True:
        print("dron heleki xodlana bilmez")
        time.sleep(1)


    print("dron xodlana biler")

    dron.mode = VehicleMode("GUIDED")

    dron.armed = True

    while dron.armed is not True:
        print("dron xodlanir...")
        time.sleep(0.5)

    print("dron xodlandi.")

    dron.simple_takeoff(hundurluk)
    
    while dron.location.global_relative_frame.alt < hundurluk * 0.9:
        print("dron hedefe qalxir...")
        time.sleep(1)

def gedis():
    global emr
    emr = dron.commands

    emr.clear()
    time.sleep(1)

    # TAKEOFF
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    # WAYPOINT
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36301483, 149.16518417, 20))
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36304696, 149.36304696, 30))
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36299825, 149.36299825, 40))
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36293503, 149.36293503, 50))
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36292674, 149.36292674, 60))
    # RTL
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
   #check
    emr.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    emr.upload()
    print("emrler uploading...")


takeoff(10)

gedis()

emr.next = 0

dron.mode = VehicleMode("AUTO")

while True:
    next_waypoint = emr.next

    print(f"sirdaki yer {next_waypoint}")
    time.sleep(1)

    if next_waypoint is 8:
        print("gedis bitdi,")
        break


print("bitdi.")