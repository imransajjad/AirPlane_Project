## initial airplane state variables
a_pos = vector(-10,5000,0);
a_vel = vector(300,0,0);


# initial values
init_alpha = 15.070*pi/180; # initial angle of attack
init_beta = 0;
air_u1 = 22*pi/180; # angle of actuator 1 in radians
air_u2 = 22*pi/180; # angle of actuator 2 in radians
air_u3 = 0; # angle of actuator 3 in degrees

air_wga = vector(0,0,0);
air_alpha = 0;
air_beta = 0;

## actuator parameters
# limits
air_t0_max = 127000;
air_t0_min = 0;
air_u_max = pi/6;
air_u_min = -pi/6;