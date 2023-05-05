#!/usr/bin/env python3

import rospy
import numpy  as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import cv2
from mavsdk  import System
from mavsdk.offboard import OffboardError,PositionNedYaw,VelocityBodyYawspeed
import numpy as np
import asyncio
from ultralytics import YOLO
from PIL import Image
import cv2
drone = System()
model = YOLO("v8n21k_15apr.pt")
cap=cv2.VideoCapture(0)
mx=320
my=240
count=0
angle = 400
drone_detected = False


def callback(data):
    global angle, drone_detected
    rang = data.ranges
    for i in range(0, 360):
        if (1/rang[i]) == 0:
            angle = 400
            drone_detected=False
        else:
            angle = i
            drone_detected = True
            break


       
       
async def start():
    print("hello")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)
    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.land()
        await asyncio.sleep(5)
        await drone.action.disarm()
        
        return
    await asyncio.sleep(1)
    
    
   
def srch():
     rospy.init_node('distance_check')

     rospy.Subscriber('/scan', LaserScan, callback)
     rate = rospy.Rate(10)  # 10 Hz
     while not rospy.is_shutdown():
        if drone_detected:
            print(f'{angle}@@@@@@@@@@@@@@@@ drone dectect @@@@@@@@@@@@@@@@@@@@@@@@@@')
            return angle,drone_detected
        
            
        rate.sleep()    

        
   
async def follow():
 m=0

 while cap.isOpened():
     
   
            # if angle is not None:

    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame, conf=0.3)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
      
       # print(results[0])
        box=results[0].boxes
        b=box.xyxy
        print(b.numel())
        if b.numel()!=0:
          
         x1=b[0][0]
         x2=b[0][2]
         y1=b[0][1]
         y2=b[0][3]
         cx=(x2+x1)/2
         cy=(y1+y2)/2
         area=(x2-x1)*(y2-y1)
         tol=[mx-cx,my-cy]
         print(b)
         #cv2.circle(annotated_frame,(cx,cy),2,(0,255,255),4)
         m=1 
        else:
        # count+=1
         #if count%5==0:
         print("no drone detect")
         # print(count)    
         m=0
        # Display the annotated frame
        cv2.circle(annotated_frame,(320,240),2,(0,0,255),4)
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
       
    if m==1:   
     if tol[0]>10 :
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,-150))
         await asyncio.sleep(0.1)
         print("-30..........................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...-30")
         
        
        
     elif tol[0]<-10:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,150))
         await asyncio.sleep(0.1)
         print("-30.........................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...30")
         
      
     if tol[1]<-40:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.5,0))
         await asyncio.sleep(0.1) 
         print("up 1......................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...up 1")
         
     elif tol[1]>40:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,-0.5,0))
         await asyncio.sleep(0.1)
         print("down 1.........................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...down 1")
         
        
        
      
    
     print(f'{area}...................................................')    
     if area<50000:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0,0.0,0.0,0))
         await asyncio.sleep(0.05)
         print("front 1..................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...front 1")
        
     elif area >90000:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0,0.0,0.0,0))
         await asyncio.sleep(0.05)
         print("back 1...................................................")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...back 1")
         
     
     await asyncio.sleep(0.1)    
    elif m==0: 
        ag,detect=srch()
        if not detect:
         try:
          await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
          await asyncio.sleep(0.3)
         except:
          print("Drone not detected, vel 0 failed")
        elif detect:
            print(f'drone detected at {ag}@@@@@@@@@@@@@@@@@@ ')
            try:
             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,ag))
             await asyncio.sleep(1)
             print(f"{angle}@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            except RuntimeError:
             print(f"Caught a RuntimeError exception. Retrying...{ag}")
            
            
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break
   


        
async def stop():
   try:
             await drone.offboard.stop()
  
   except OffboardError as error:
             print("unable to stop offboard")
             print(f'{error._result.result}')
   try:

      await drone.action.land()
      print("landing...")
      await drone.action.disarm()

   except RuntimeError:
      print("Caught a RuntimeError exception. Retrying...land")
      
async def ma():
    
    await follow()


      
if __name__ == "__main__":
    
    
   
    asyncio.run(start())
   
 
    asyncio.run(ma())
   
    asyncio.run(stop())
    
    cap.release()
    cv2.destroyAllWindows()
