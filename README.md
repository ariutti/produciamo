# README

Preliminary Arduino code for an interactive installation using:

* 1x  Arduino UNO
* 13x capacitive pads
* 4x  MPR121 breakout boards

## Connections

Connect Arduino to the 4 MPR via the same bus used for the capacitive wall. The cable is a shielded cable where the shield is connected to the same voltage of the Arduino GND.

Then connect the MPR in a way that:

+ 1st MPR to PADs 1 and 2 (2x pads)
+ 2nd MPR to PADs 3 and 4 (2x pads)
+ 3rd MPR to PADs 5 and 13 (2x pads)
+ 4th MPR to PADs 6,7,8,9,10,11,12 (7x pads)

## MPR121 initial settings

It will be wise to give all these MPR different settings cause of the fact PADs all have different dimensions and size.

PADs 1,2,3,4,5,13 seems to behave in a similar way (similar to the capacitive wall behaviour), something different happens with PADs 6,7,8,9,10,11,12 which are smaller and have particular form factors.

So why not set the 4th MPR with different settings respect the others?

### CASE STUDY

