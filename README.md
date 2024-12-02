# Solio-S186

## Table of Contents
- [Introduction](#introduction)
- [TiHAN Testbed Route](#tihan-testbed-route)
- [FAQ](#FAQ)

## Introduction

**Project Name:** ADAS for Point-to-Point Navigation System for Autonomous Car Adaptable for Indian Scenarios  <br/>
**Project Duration:** 3 years <br/>
**Principal Investigator:** Prof. P Rajalakshmi <br/>

<p align="center">
  <img src="https://cdn.wheel-size.com/automobile/body/suzuki-solio-bandit-2020-2022-1626347862.1116588.jpg" alt="Solio" width="48%" />
  <img src="https://github.com/nineRishav/Solio-S186/blob/master/media/IITHLogo_V.png" alt="IITH" width="30%" />
</p>

📺 [Watch the Demo Video](https://github.com/nineRishav/Solio-S186/blob/master/media/GUI-demo.webm)

<div style="text-align: center;">
  <img src="media/Demo-PC-view.gif" alt="Alt Text"/>
</div>




## TiHAN Testbed Route

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/RouteMap.png" alt="TestBed Map" height="600">

Check point:
- 1 : Pedestrian Crossing
- 2 : Car coming in our lane
- 3 : Bicycle coming in our lane
- 4 : Speed Bump
- 5 : Traffic Sign(Stop)
- 6 : Traffic Light
- 7 : Car coming from the front
  
## FAQ

### Q: All hardware connections are fine, but why isn't the vehicle actuating when running the navigation code?
**A:** Ensure the ORIN is blinking fast, the MABX does not show any errors, there are no ESP errors on the vehicle, and the Private CAN is ON. Additionally, start the perception system before running the navigation file to avoid any errors.

### Q: Why is the vehicle not following the designated path and moving erratically?
**A:** The vehicle's GNSS system likely needs calibration. Check the [pos_type](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Position_VelocityType) status in `bestpos` topic of GNSS system. POS_TYPE should be 50 (RTK working with solution working) or 53(RTK enabled but solution is last calculated).

### Q: Vehicle suddenly in the middle of path, not able to actuate properly on scenerio or going on the right path ? 
**A:** Check the `Roslaunch` on the terminal, possibly the wire was loose and rosclaunch terminal screen was terminated.

### Q: Steps for turning the manual mode to autonomous mode.
**A:** Following are the steps, before doing anything, make sure Seat Belts are ON for front passengers, front doors are closed.
- Step 1: Turn the Engine ON
- Step 2: Turn ON the MABX circuit breaker switch
- Step 3: Switch the CAN switch from Auto to Manual
- Step 4: On the Steering Mounted Controls, towards the right side, switch on cruise control switch and press down the bottom to start the Autonomous mode. 
