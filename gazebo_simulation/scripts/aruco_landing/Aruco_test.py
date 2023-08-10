#!/usr/bin/env python3

import pymavlink.mavutil as mavutil
import asyncio
from mavsdk import System
from mavsdk.offboard import (
    OffboardError, PositionNedYaw, VelocityBodyYawspeed)
import landing
import numpy as np
import cv2

mavlink_addr = "127.0.0.1"
mavlink_port = 14750
src_sys = 201

mavsdk_port = 14540
#front_offset = 0.3
#side_offset = -0.2
#rotation_offset = 70
altitude = 7
descent_speed = -0.5
aruco_visualization = True

async def print_altitude(drone):
    """ Prints the altitude when it changes """

    previous_altitude = None

    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != previous_altitude:
            previous_altitude = altitude
            print(f"Altitude: {altitude}")

async def get_altitude(drone):
    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        return altitude


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return


async def run():

    loop = asyncio.get_event_loop()

    #  PyMavlink connection for cover openning and closing.
    mav = mavutil.mavlink_connection(
        'udpout:' + mavlink_addr + ":" + str(mavlink_port), source_system=src_sys)
    # MAVSDK connection for drone control.
    drone = System()
    lander = landing.Landing(drone, descent_speed)

    await drone.connect(system_address="udp://:" + str(mavsdk_port))

    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)
    print("-- Starting offboard")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    
    print("-- Move to start--")
    await drone.offboard.set_position_ned(PositionNedYaw(0, -2, -2.0, 0))
    await asyncio.sleep(20)        

    print("-- Open Video Capture --")
    cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert ! appsink emit-signals=true sync=false max-buffers=8 drop=true", cv2.CAP_GSTREAMER)


    if not cap.isOpened():
        print('VideoCapture not opened')
        exit(-1)

    while True:
        ret, frame = cap.read()
        
        if not ret:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            await asyncio.sleep(1)   
            cap.release()
            cv2.destroyAllWindows()
            break    

        corners, centers, ids = lander.process_frame(frame)
        if corners is None or len(corners) == 0:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            await asyncio.sleep(1)   
            cap.release()
            cv2.destroyAllWindows()
            break
        altitude = 0
        async for position in drone.telemetry.position():
            altitude = position.relative_altitude_m
            break   
   
        print(altitude)    
        s_id = 2
        index = ids.tolist().index(s_id)
        (topLeft, topRight, bottomRight, bottomLeft) = corners[index]
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))


        vel = 0
        async for position_vel in drone.telemetry.position_velocity_ned():
            vel = position_vel.velocity
            break 

        print(vel)

        moveX, moveY, rotation = lander.compute_move(
            frame.shape, corners[index], centers[index])
        #print(moveX, moveY,rotation)
        await lander.apply_move(moveX, moveY, rotation)
        
        if (aruco_visualization):
            frame = lander.draw_aruco(frame, corners[index])
            cv2.imshow('aruco_visualization', cv2.resize(frame, dsize=(640,480)))
            if cv2.waitKey(1) & 0XFF == ord('q'):
                break

          
        print("waiting")  
        
        await asyncio.sleep(0.1)   


    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(
            f"Stopping offboard mode failed with error code: {error._result.result}")
    print ("-- Landing --")
    lander = landing.Landing(drone, 0)
    #print(moveX, moveY,rotation)
    #await drone.action.kill()
    await asyncio.sleep(5)


    

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
