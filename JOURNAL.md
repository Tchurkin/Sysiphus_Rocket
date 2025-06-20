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

**6/19/25** - 2hr - I just found the issue that's been holding me up for a week. The drain and source on my pyro MOSFETs were backwards. The problem is that EasyEDA had the drain and source bindings from schematic to PCB view switched up. I did some more testing though with some trace cutting and jumper wires to flip them back and it sort of worked, but I think my solder connections were too bad. I ordered another test PCB and it should arrive in a few days.

