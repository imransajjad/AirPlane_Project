##########################################################
# airplane parameters: do not touch anything below this
##########################################################
a_x = vector(0,0,1);
a_y = rotate(vector(1,0,0), angle=0, axis=a_x);
a_z = rotate(vector(0,1,0), angle=0, axis=a_x);

# airframe geometry
air_area = 28; 
air_area_ac1 = 5;
air_area_ac2 = 5;
air_area_ac3 = 3.5;
air_area_ac4 = 2.5;
air_rc0 = vector(0,-0.15,0);
air_rc1 = vector(-2,-2,0);
air_rc2 = vector(2,-2,0);
air_rc3 = vector(0,-3,2);
air_rc4 = vector(0,-2,2);

rho_r = 5;

# airframe inertia
air_m = 15000;
air_I = 100000;

# air forces
air_acc_ang = vector(0,0,0);
air_tau = vector(0,0,0);
air_force = vector(0,0,0);
air_acc = vector(0,0,0);

air_fc0 = vector(0,0,0);
air_fc1 = vector(0,0,0);
air_fc2 = vector(0,0,0);
air_fc3 = vector(0,0,0);
air_fc4 = vector(0,0,0);

# environment variables
rho = 1.225; # air density
b = 1.983; # air viscosity
g = 9.81; # gravity