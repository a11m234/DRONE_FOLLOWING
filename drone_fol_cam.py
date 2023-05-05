#!/usr/bin/env python3
import cv2
from mavsdk  import System
from mavsdk.offboard import OffboardError,PositionNedYaw,VelocityBodyYawspeed
import numpy as np
import asyncio
from ultralytics import YOLO
from PIL import Image
import cv2
drone = System()
model = YOLO("/home/taskforce/catkin_ws/src/dron/v8n21k_15apr.pt")
cap=cv2.VideoCapture(2)
mx=320
my=320




    
async def start():
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
    try:
     await drone.action.arm()
     print("armed")
    except:
       print("not armed")
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    print("takeoff")
    await drone.action.takeoff()
    await asyncio.sleep(5)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    print("taking off")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
    await asyncio.sleep(5)
    
    
   
async def follow():
 m=0
 while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame, conf=0.5)

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
         cv2.circle(annotated_frame,(cx,cy),2,(0,255,255),4)
         m=1 
        else:
         print("no drone detect")    
         m=0
        # Display the annotated frame
        cv2.circle(annotated_frame,(320,320),2,(0,0,255),4)
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
       
    if m==1:   
     if tol[0]>10 :
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,-30))
         print("-30")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...-30")
         await asyncio.sleep(1)
        
        
     elif tol[0]<-10:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,30))
         print("-30")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...30")
         await asyncio.sleep(1)
      
     if tol[1]<-10:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,-0.1,0))
         print("up 1")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...up 1")
         await asyncio.sleep(1) 
     elif tol[1]>10:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.1,0))
         print("down 1")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...down 1")
         await asyncio.sleep(1)
        
        
      
     await asyncio.sleep(3) 
     print(area)    
     if area<8000:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0,0.0,0.0,0))
         print("front 1")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...front 1")
         await asyncio.sleep(1)
     elif area >200000:
        try:
         await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0,0.0,0.0,0))
         print("back 1")
        except RuntimeError:
         print("Caught a RuntimeError exception. Retrying...back 1")
         await asyncio.sleep(1)
     
    #  await asyncio.sleep(3)    

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

