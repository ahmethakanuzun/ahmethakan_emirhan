from pymavlink import mavutil 

address = 'udpin:localhost:14551' #simulasyon
# address= '/dev/ttyACM0' #pixhawk usb
# address= '/dev/ttyTHS1' #pixhawk telem2 baudrate= 115200
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

# while True:
#     message= vehicle.recv_match(blocking= True)
#     print(message)


def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt
def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
print("arac arm edildi")
takeoff(10)

from pymavlink import mavutil 

address = 'udpin:localhost:14551'
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt
def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
print("arac arm edildi")
takeoff(10)

def move(y,x,z):
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,vehicle.target_system,vehicle.target_component,9,
        int(0b0000011111111000),
        y,x,z,
        0,0,0,
        0,0,0,
        0,0
    ))
def go_to(lat,lon,alt):
    vehicle.mav.mission_item_send(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,2,0,0,0,0,0,lat,lon,alt)

    from pymavlink import mavutil 

address = 'udpin:localhost:14551'
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt
def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
print("arac arm edildi")
takeoff(10)

def move(y,x,z):
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,vehicle.target_system,vehicle.target_component,9,
        int(0b0000011111111000),
        y,x,z,
        0,0,0,
        0,0,0,
        0,0
    ))
def go_to(lat,lon,alt):
    vehicle.mav.mission_item_send(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,2,0,0,0,0,0,lat,lon,alt)

    from pymavlink import mavutil, mavwp

address = 'udpin:localhost:14551'
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

wp= mavwp.MAVWPLoader()

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt
def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

def add_mission(seq,lat,lon,alt):
    frame= mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(vehicle.target_system, vehicle.target_component,
    seq,
    frame,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,lat,lon,alt))

    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wp.count())
    for i in range (wp.count()):
        msg= vehicle.recv_match(type=["MISSION_REQUEST"], blocking= True)
        vehicle.mav.send(wp.wp(msg.seq))
        print("Sending waypoints {0}".format(msg.seq))

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
takeoff(10)
add_mission(0, -35.36319320, 149.16543250, 25)
add_mission(1, -35.36296790, 149.16540030, 35)
add_mission(2, -35.36292410, 149.16519640, 40)
add_mission(3, -35.36306190, 149.16500060, 20)
add_mission(4, -35.36319970, 149.16513470, 10)
vehicle.set_mode("AUTO")