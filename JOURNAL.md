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

**6/29/25** - 5hr - My sensor testing module PCB has been taking significantly longer than I had expected, and I'm not sure if I'm going to get it before I launch. By the way I'm planning on launching for the first time within a week or two so I have at least a single launch to show at Open Sauce. I decided to modify the main Impulse 2.0 board so it is compatible with the little sensor module that may or may not arrive. I also added an MPU-6050 and BMP-280 module in case the other sensors don't work. I also updated the pyro and continuity check circuit on the main PCB to use the simpler 10K resistor and LED designs in parallel to the MOSFETs. I really hope this board works, but if some parts don't I do have ways to fix the most likely problems. For example, if the continuity circuit is keeping the pyro LOW states to high, I can just remove to LEDs because they are non-essential. I think the only real places my board might fail are the peripherals like the button, buzzer, and RGB LED, which I didn't test. I'm also going to review the design to make sure I'm not making any integration errors. You know I'm sort of starting to like this journaling; it's like therapy.


**6/30/25** - 2hr - I want to to a test launch that only does the TVC for the rocket and then deploys a parachute on the way down. This should be a good test for general integration of the systems. I figure since I have to wait for the new electronics anyway, I might as well try to do what I can with the old PCB. I don't think the old PCB is good enough to do a full propulsive landing, but it sould be enough for just the basic thrust vectoring and parachute and landing leg deployment.

**7/3/25** - 7hr - I've been forgetting to journal daily, but a lot has been going on with the electronics. First of all, I think my pyro circuits have all been bottle-necking the output current due to the traces only being 10 mil. This makes  lot of sense because, while the pyros would output very close to the battery's voltage, they seemed to be less powerful in heating the nichrome wire. With this in mind, I changed all high-power pyro traces to 90 mil, which should be plenty for battery output. I also reliazed my current voltage regulation design only permits up to 500mA, which is enough for powering the teensy and logic components, but not enough for the servos, so I added a bigger buck converter to power the servos. Also, the sensor board arrived in the mail finally! And it works very well! Actually I was only able to get the DPS310 and BNO055 working, but they both seem much more accurate than the MPU6050 and BMP280 I have been using. Finally, since I want to launch before Open Sauce, I ordered the final integrated PCB. I added some compromises and redundant systems to it as well. For power management, I have my 500mA buck converter that I designed, but I also have a DROK 3A buck converter to power the servos, there are also tap-offs for both regulators to wire an external regulator to either one, and ways to easiler use the DROK converter exclusively. In terms of sensors, I have connections for the good sensor board that just arrived, as well as a redundant bmp280 and mpu6050. For the pyros I have tap-offs to all the essentialls if I need to wire a new circuit externally. With all these back-up systems I con't conceive many plausible ways for the board to have any fatal errors, and I am very excited to launch this thing!

<img src="https://github.com/user-attachments/assets/a250e04d-6709-49b0-804b-33b8258f60d2" width="40%" height="40%">

**7/10/25** - 3hr - The PCBs i've been waiting for came today. The two protoboards I don't really care about right now, but the final board that's going to launch seems to be working almost perfectly. I hope to launch to test my TVC stability in a few days.

**7/11/25** - 4hr - I'm just changing the code for the new board. I hope to launch tomorrow and also on monday.

**7/13/25** - 12hr - This log covers yesterday and today. I finally launched! Most of the parts worked, but I think the rocket was aerodynamically unstable, meaning even with TVC correction it pitched over quickly and spun out of control. A lot of the rocket was destroyed, but the landing legs, and upper section is okay. I am currently wiring up a new PCB and speed-printing new rocket parts. I'm not satisfied with today's launch, so I want to get in one more (hopefully tomorrow) before presenting at Open Sauce at the end of the week. I leave in two days though, so I need to be very quick.
