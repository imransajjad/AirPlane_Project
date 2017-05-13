## added 13-03-2014
the airplane state variables and environment variables act as initial conditions. you can alter these to input different initial conditions.

'air_u1' and 'air_u2' are the angles of the ailerons. alter these to roll or pitch the aircraft. note that even in straight flight, there has to be some positive angle on both to counter gravity.

'air_t0' is the thrust. the more thrust, the more the acceleration, and more speed is gained. with more speed, less alpha (angle of attack) and hence less 'air_u1' and 'air_u2' is required.

the vpython environment frame on the screen is x axis-> right, y axis-> up, z axis-> out of the screen. the airplane frame is z-> up, x-> right, y-> forward. variables with the prefix 'a_' are in world (vpython) frame and with 'air_' are in airplane frame.

the model chosen is the Newtonian lift model with a compressible, continuous flow of air. forces of lift, drag, gravity, torques are simulated. frictional forces have also been 'added' to this model. vortex lift is a very complicated (graduate school level) problem and has not been included.

if you want to alter the airplane parameters, all of them are in the airplane frame.

## added 26-03-2014
'air_u3' is the rudder angle. It is used to control yaw. +ve 'air_u3' means the rudder turns towards the right and the aircraft turns right.

The variables displayed are as follows:

position: 3x1 vector in world frame, (meters)
velocity: magnitude, 3x1 vector in world frame (meters/sec)
altitude: (meters)

force: 3x1 vector in airplane frame (Newtons)
torque: 3x1 vector in airplane frame (Newton-meters)
omega: turn-rate 3x1 vector in airplane frame (radians/sec)
aoa: angle of attack, alpha (degrees)
beta: yaw angle (degrees)

actuators: values of air_u1, air_u2, air_u3, air_t0 (degrees, degrees, degrees, Newtons)
stick: values of keyboard input pilot_u1, pilot_u2, pilot_u3, pilot_t0 ([-1,1],[-1,1],[-1,1],[0,1])

frame correctness: a measure of the perpendicularness of the airplane frame. should be zero or close to zero under normal conditions.



keyboard controls are explained as follows:
	pilot_u1	pilot_u2	result
	+ve		+ve		air_u1,air_u2 +ve, aircraft pitches up
	+ve		-ve		air_u1 -ve, air_u2 +ve, aircraft rolls left
	-ve		+ve		air_u1 +ve, air_u2 -ve, aircraft rolls right
	-ve		-ve		air_u1,air_u2 -ve, aircraft pitches down

	pilot_u3	result
	+ve		air_u3 +ve, aircraft yaws right
	-ve		air_u3 -ve, aircraft yaws left

	pilot_t0
	range between 0 and 1
	multiplied by air_t0_max (maximum thrust) to give total thrust

key bindings:
Key		motion
up arrow	pitch down
down arrow	pitch up
right arrow	roll right
left arrow	roll left
w		thrust up
s		thrust down
a		rudder left
d		rudder right

Also use the mouse to alter the view:
hold middle mouse button and move mouse forward and back to zoom in and out
hold right mouse button and move mouse to change camera direction


note: with these controls, I have had to include a tail and a rudder to cater for beta as well. the equations for this force are similar, they are just in a different direction. as a result, when the aircraft turns, the nose does not start to point away from the velocity direction when the aircraft is sideways.
So now there are five force elements acting on the aircraft: the wing area, the tail area, ailerons 1 and 2 and the rudder.

note: there is also a stick decay function which returns the stick to its center position if none of the keys are pressed. the throttle is not decayed.

## added 04-04-2014
added crash prediction and conditions, some graphics and control improvements.
added pause and camera views.

press 'e' to pause and resume.
press 'c' to cycle through views. mouse zoom enabled in all views, rotation only in 4.
1: view attached to velocity frame
2: view attached to velocity frame facing back
3: view attached to airplane frame (pilot view)
4: free view

control:
added a term to make plane fly straight (maintain required angle of attack for a speed by using actuators). as a consequence, even when there is no stick input, there should be some positive actuator values on air_u1 and air_u2.

crash prediction:

printed on screen: [crash message, caution/correction message]

the red forward trail is the current trajectory of the plane.
if this trail intesects the ground, 'on crash course' is printed, otherwise 'safe' is printed as a crash message.

the cyan trails are the trajectories of the plane with maximum input in up down, left, right directions. (the right and left trails are 'roll, then pull up', not yaw).
if a horn or megaphone-like shape was fit in the cyan trails, it would constitute the all the possible directions the plane can take. call this region 'X'.

if a portion of the ground is in X, a caution message is printed with the position of the ground in X. ('crash if up, crash if down, etc'). means if plane goes in that direction, it will crash.

if the crash message is 'on crash course', a correction is found by seeing where in X can the plane go to avoid a crash. this correction is printed in place of the caution message now.

if all the trails lead into the ground, (no place to go for plane to avoid a crash), the correction message is to speed up and move in the direction where the ground is farthest away from the plane.

if all the trails lead into the ground, crash point is too close to the plane (not enough room to speed up), the correction message is to eject.

note: speeding up increases the radius of X. it makes a wider range of directions available for the plane to move in. from a control systems perspective, speeding up increases the controllability (the amount of directions the plane can take).
