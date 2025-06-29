---
title: Sysiphus Landing Rocket
author: Braxton
description: 3D-printed model rocket that uses thrust vector control (TVC) and lands like SpaceX.
created_at: 2024-06-06
---

**NOTE:** Starting this journal late, already designed prototype, haven't launched yet though.

**6/6/25** - 2hr - Minor improvements to launchpad, tightented pad bolts using nut-like piece and PVC pipe segment as spacers.
<img src="https://github.com/user-attachments/assets/2bd1a0e5-71d6-4519-9927-9c3140087db3" width="30%" height="30%">




**6/7/25** - 1hr - Tested parachute ejection and landing leg deployment, both using a 2S lipo and thin nichrome wire to sever respective rubber bands. Both work exceedingly well.

<img src="https://github.com/user-attachments/assets/f7a25402-e031-462b-b53e-2e56d85a7688" width="30%" height="30%">
<img src="https://github.com/user-attachments/assets/d4dd71c1-8764-4b15-b5a8-0d080f379321" width="30%" height="30%">



**6/9/25** - 2hr - Began reaction wheel assembly, I'm not planning on using it for roll stability in the first launch, but I will use it on the next one if necessary.

<img src="https://github.com/user-attachments/assets/88af9159-2e43-4491-b572-5a71f75d89d5" width="30%" height="30%">



**6/10/25** - 3hr - Tested pyro and continuity check circuit module PCB. It seems to be partially malfunctional. When the pyro is turned off, a very small current is supposed to pass through the pyro output to check continuity and turn on an LED, but this voltage is way too high, very close to the output voltage when turned on.

<img src="https://github.com/user-attachments/assets/d827f7b4-b00c-4794-9ffd-46f2ca29dd15" width="30%" height="30%">



**6/12/25** - 4hr - Troubleshot pyro and continuity check circuit. I found a new circuit that should work better for checking continuity and is also much simpler, so I will probably order another prototype pyro module with these improvements soon.

<img src="https://github.com/user-attachments/assets/7ddfb0d9-fad5-4999-8aa9-6adba38b4ef7" width="30%" height="30%">



**6/17/25** - 2hr - The next prototype for the pyro and continuity check circuit arrived today. I think my circuit is right, but the MOSFETs don't seem to be working. This is the stuff nightmares are made of. These prototypes are pretty expensive so failures are very not good.
<img src="https://github.com/user-attachments/assets/867bc351-a2e9-4a82-ac6c-391210421bcd" width="30%" height="30%">



**6/19/25** - 2hr - I just found the issue that's been holding me up for a week. The drain and source on my pyro MOSFETs were backwards. The problem is that EasyEDA had the drain and source bindings from schematic to PCB view switched up. I did some more testing though with some trace cutting and jumper wires to flip them back and it sort of worked, but I think my solder connections were too bad to be sure. I ordered another test PCB and it should arrive in a few days.



**6/24/25** - 1hr - I am now revisiting the code for the rocket. I haven't worked on it in a few months, so it's a little foreign right now. I'm trying to combine the sensor data from the IMU and baro to get a more accurate speed/altitude reading. I also transitioned to VScode from Arduino IDE.



**6/25/25** - 1hr - the new pyro circuit prototype arrived today. It seems like the mosfet part works fine, but the low voltage was still too high (around 6.4V), so I replaced the 220 ohm resistor in front of the LED with a 10K and it seems to work pretty well. I'm hoping to order the full PCB very soon so I can launch ASAP.

<img src="https://github.com/user-attachments/assets/fd43574b-a637-48a5-9388-11d8cc4a2abc" width="30%" height="30%">


**6/26/25** - 4hr - I'm working on the sensor fusion. The main challenge is dealing with drift, inaccuracies, and also I need to do some relatively hard math to find the total upward acceleration from the gyro angles and accelerometer readings for the rocket axes.


**6/27/25** - 5hr - I decided not to do any fancy sensor fusion because I can't get the accelerometer to accurately integrate to altitude. I'm just going to use the raw values. However, I think it will be smart to incorporate some buffer for the descent motor ignition. I need to time how long it take my flight computer to ignite F15 motors and I also think my altitude reading will be lagging behind a little. I will roughly time the delay that both of these introduce and light the motor pyro a little sooner. I also simulated the aerodynamics of the rocket using CFD to find the center of pressure, which will be essential in determining how to balance the rocket.

<img src="https://github.com/user-attachments/assets/0804b68b-354a-4e7e-98b7-e11e0881dcdd" width="20%" height="20%">
<img src="https://github.com/user-attachments/assets/7f806c7c-56e4-4350-b77d-54a038641192" width="30%" height="30%">

**6/29/25** - 1hr - My sensor testing module PCB has been taking significantly longer than I had expected, and I'm not sure if I'm going to get it before I launch. By the way I'm planning on launching for the first time within a week or two so I have at least a single launch to show at Open Sauce. I decided to modify the main Impulse 2.0 board so it is compatible with the little sensor module that may or may not arrive. I also added an MPU-6050 and BMP-280 module in case the other sensors don't work. I also updated the pyro and continuity check circuit on the main PCB to use the simpler 10K resistor and LED designs in parallel to the MOSFETs. I really hope this board works, but if some parts don't I do have ways to fix the most likely problems. For example, if the continuity circuit is keeping the pyro LOW states to high, I can just remove to LEDs because they are non-essential. I think the only real places my board might fail are the peripherals like the button, buzzer, and RGB LED, which I didn't test. I'm also going to review the design to make sure I'm not making any integration errors.
