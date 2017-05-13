####################################################################
# pilot and control parameters: do not touch anything below this
####################################################################

## pilot input
global stick_state;

global pilot_u1;
global pilot_u2;
global pilot_u3;
global pilot_t0;
global pilot_s0;
global pilot_t1;

global pause;
global camera_view;
pause = 1;
camera_view = 2;

# pilot input variables
stick_state = [0, 0, 0, 0, 0, 0, 0, 0];

pilot_u1 = 0;
pilot_u2 = 0;
pilot_u3 = 0;
pilot_t0 = 0.7;
pilot_s0 = 0;
pilot_t1 = 1;


K_stick = 0.75;
K_yaw = 0.25;
K_thrust = 0.001;

# define functions for keyboard control
def keyPress(event):
    global stick_state;
    global camera_view;
    global pause;

    global pilot_u1;
    global pilot_u2;
    global pilot_u3;
    global pilot_t0;
    global pilot_s0;
    global pilot_t1;

    global pitch_e1;
    global pitch_i;
    global roll_e1;
    global roll_i;

    if event.key == 'c':
        camera_view = (camera_view+1)%4;
    if event.key == 'e':
        pause = (pause+1)%2;
    if event.key == 'up':
        stick_state[0] = 1;
    if event.key == 'down':
        stick_state[1] = 1;
    if event.key == 'left':
        stick_state[2] = 1;
    if event.key == 'right':
        stick_state[3] = 1;
    if event.key == 'a':
        stick_state[4] = 1;
    if event.key == 'd':
        stick_state[5] = 1;
    if event.key == 'w':
        stick_state[6] = 1;
    if event.key == 's':
        stick_state[7] = 1;
    if event.key == 'x':
        pilot_s0 = 1-pilot_s0;
    if event.key == 'z':
        pilot_t1 = (pilot_t1+1)%2;
        if pilot_t1 == 2 :
            pitch_e1 = 0;
            pitch_i = 0;
            roll_e1 = 0;
            roll_i = 0;
            
        
    if pilot_t0 > 1:
        pilot_t0 = 1;
    if pilot_t0 < 0:
        pilot_t0 = 0;
        
def keyRelease(event):
    global stick_state;

    if event.key == 'up':
        stick_state[0] = 0;
    if event.key == 'down':
        stick_state[1] = 0;
    if event.key == 'left':
        stick_state[2] = 0;
    if event.key == 'right':
        stick_state[3] = 0;
    if event.key == 'a':
        stick_state[4] = 0;
    if event.key == 'd':
        stick_state[5] = 0;
    if event.key == 'w':
        stick_state[6] = 0;
    if event.key == 's':
        stick_state[7] = 0;
        
def stick_iter(stick_state, scale):
    global camera_view;
    global pause;
    global pilot_u1;
    global pilot_u2;
    global pilot_u3;
    global pilot_t0;
    global pilot_s0;
    global pilot_t1;
    
    global K_stick;
    global K_yaw;
    global K_thrust;
    if stick_state[0] == 1 :
        pilot_u1 = (1-K_stick*scale)*pilot_u1 - K_stick*scale;
        pilot_u2 = (1-K_stick*scale)*pilot_u2 - K_stick*scale;
    if stick_state[1] == 1 :
        pilot_u1 = (1-K_stick*scale)*pilot_u1 + K_stick*scale;
        pilot_u2 = (1-K_stick*scale)*pilot_u2 + K_stick*scale;
    if stick_state[2] == 1 :
        pilot_u1 = (1-K_stick*scale)*pilot_u1 - K_stick*scale;
        pilot_u2 = (1-K_stick*scale)*pilot_u2 + K_stick*scale;
    if stick_state[3] == 1 :
        pilot_u1 = (1-K_stick*scale)*pilot_u1 + K_stick*scale;
        pilot_u2 = (1-K_stick*scale)*pilot_u2 - K_stick*scale;
    if stick_state[4] == 1 :
        pilot_u3 = (1-K_yaw)*pilot_u3 - K_yaw;
    if stick_state[5] == 1 :
        pilot_u3 = (1-K_yaw)*pilot_u3 + K_yaw;
    if stick_state[6] == 1 :
        pilot_t0 = pilot_t0 + K_thrust;
    if stick_state[7] == 1 :
        pilot_t0 = pilot_t0 - K_thrust;
        
    if pilot_u1 > 1:
        pilot_u1 = 1;
    if pilot_u1 < -1:
        pilot_u1 = -1;

    if pilot_u2 > 1:
        pilot_u2 = 1;
    if pilot_u2 < -1:
        pilot_u2 = -1;

    if pilot_u3 > 1:
        pilot_u3 = 1;
    if pilot_u3 < -1:
        pilot_u3 = -1;
    if pilot_t0 > 1:
        pilot_t0 = 1;
    if pilot_t0 < 0:
        pilot_t0 = 0;


def stick_decay():
    global pilot_u1;
    global pilot_u2;
    global pilot_u3;
    global K_stick;
    global K_yaw;
    mu = 0;
    sigma = 0.0000005;
    pilot_u1 = (1-K_stick)*pilot_u1 + random.gauss(mu, sigma); 
    pilot_u2 = (1-K_stick)*pilot_u2 + random.gauss(mu, sigma);
    pilot_u3 = (1-K_yaw)*pilot_u3 + random.gauss(mu, sigma);
