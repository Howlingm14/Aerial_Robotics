import time
from pymavlink import mavutil

def set_position(lat,lng,alt,master):
    master.mav.set_position_target_global_int_send(
        time_pair[1]+int(round((time.time()-time_pair[0])*1000)),
        1,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        (ignore_velocity | ignore_accel),
        int(lat*(10**7)), # Lat (degE7)
        int(lng*(10**7)), # Long (degE7)
        alt, # Altitude
        0,0,0, # Velocities
        0,0,0, # Accels
        4.939, # Yaw 
        0 # Yaw rate
        )

#Fenswood
#--home=51.4235413,-2.6708488,50,250
#Ol Pejeta
#--home=0.07690274,36.87651257,1780,0
#Put this command into the mission planner simulation

#path coordinates in longitude and latitude
# [[ 0.07690274 36.87651257]
#  [ 0.10459048 36.75547052]
#  [ 0.10732521 36.74321559]
#  [ 0.11189748 36.71861897]] # this is how it comes out of the path finder code
shortestpath=[[0.07690274, 36.87651257, 110], [0.093515384, 36.80388734, 110], [0.096284158, 36.791783135, 155], [0.10459048, 36.75547052, 155], [0.10732521,36.74321559, 155], [0.11189748,36.71861897, 155], [0.10732521,36.74321559, 155], [0.10459048, 36.75547052, 155], [0.096284158, 36.791783135, 155], [0.07690274, 36.87651257, 110]]

#connect to local device mission planner
master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")

#confirm connection
master.wait_heartbeat()
print("Got heartbeat")

#get some information from the drone
#Time
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    2,
    1e5,
    0,0,0,0,
    0,
    )

#typical VFR HUD information
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    74,
    1e5,
    0,0,0,0,
    0,
    )

#global position
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    33,
    1e5,
    0,0,0,0,
    0,
    )

system_time = master.recv_match(type="SYSTEM_TIME",blocking=True)
time_pair = (time.time(), system_time.time_boot_ms)
print("Time synced")

#change mode to guided
master.mav.send(mavutil.mavlink.MAVLink_command_long_message(
    1,
    1,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4, # 4 for guided
    0, 
    0, 
    0,
    0, 
    0  
))

#verify mode change
while master.flightmode != "GUIDED":
    print("Waiting for flightmode change")
    master.wait_heartbeat()
print(master.flightmode)
print("mode changed")

master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to arm
        0, 0, 0, 0, 0, 0
        )

print("Drone armed")

#disable any geofences
master.param_set_send('FENCE_ENABLE', 0)

#take off
takeoffalt=61 #200ft
master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        takeoffalt
        )  

# Confirm take off command
msg = master.recv_match(type="COMMAND_ACK",blocking=True)
print(msg)
if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Error sending takeoff command")
    exit()

# ignore velocity and accelerations
ignore_velocity = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

ignore_accel = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    )


#check take off is complete
alt_home=1780
while True:

    msg = master.recv_match()

    if not msg:
        continue

    if msg.get_type()== 'VFR_HUD':
        print(msg.alt-alt_home)
        if round(msg.alt-alt_home,0)==takeoffalt:
            print("Take off complete")
            break
        
msg_filter=1
for i in range(len(shortestpath)):

    print("Moving to",shortestpath[i] )
    set_position(shortestpath[i][0],shortestpath[i][1],shortestpath[i][2],master)
    while True:

        msg = master.recv_match()

        if not msg:
            continue

        if msg.get_type()== 'VFR_HUD':
            msg_filter=msg_filter+1
            if msg_filter%10==0:
                print(msg)
        
        if msg.get_type()=="GLOBAL_POSITION_INT":
            if msg_filter%10==0:
                print(msg.lat/1e7, ", ", msg.lon/1e7)
            if round(msg.lat/1e7,5)==round(shortestpath[i][0], 5) and round(msg.lon/1e7,5)==round(shortestpath[i][1], 5):
                print("Arrived at",shortestpath[i])
                break



print("Mission Completed")
exit()