from visual import *
import random

# Airplane Simulation
# Imran Sajjad

##
# all variables with the prefix a_ are in the world frame
# when looking at the screen at the start,
# world x-axis is right
# world y-axis is up
# world z-axis is out of the screen

# all variables with the prefix air_ are in the airplane's frame
# world x-axis is lateral (right +ve)
# world y-axis is longitudinal (forward +ve)
# world z-axis is up

# there are four camera views to choose from
# explained in readme
##

execfile("pilot_input.py");
execfile("air_param.py");
execfile("air_init.py");
#import visual_init.py
execfile("visual_init.py");

# controller gains
K_roll = 11;
K_pitch = 2;
K_yaw = 0.7;
K_trim = 1361;

##########################################################
# main loop of simulation
##########################################################

# store old variables
a_vel1 = a_vel;
air_wga1 = air_wga;
a_acc1 = vector(0,0,0);
air_acc_ang1 = vector(0,0,0);

t = 0;
dt = 0.02*0.2;
while True:
    rate(1/dt)

    air_vel = a_vel.mag;
    air_rho = rho*pow(1 - 0.0065*a_pos.y/288.15, 4.258);
    air_alpha = diff_angle(vector(dot(a_vel,a_x),dot(a_vel,a_y),0),\
                           vector(dot(a_vel,a_x),dot(a_vel,a_y),dot(a_vel,a_z)));
    if dot(a_vel,a_z) >= 0:
        air_alpha = -air_alpha;
    air_beta = diff_angle(vector(0,dot(a_vel,a_y),dot(a_vel,a_z)),\
                vector(dot(a_vel,a_x),dot(a_vel,a_y),dot(a_vel,a_z)));    
    if dot(a_vel,a_x) >= 0:
        air_beta = -air_beta;

    
    # controller
    if (pilot_t1 == 0) | (pilot_t1 == 1) :
    	stick_iter(stick_state,1);
    	if pilot_t1 == 0 :
    		trim_com = 0;
    	elif pilot_t1 == 1 :
    		trim_com = asin(min(1,K_trim/air_rho/pow(air_vel,2)));

    	pitch_com = min(13750/air_rho/pow(air_vel,2), max(-6000/air_rho/pow(air_vel,2), \
                trim_com + K_pitch*(pilot_u1+pilot_u2) ));
    	roll_com = min(15000/air_rho/pow(air_vel,2), max(-15000/air_rho/pow(air_vel,2), \
                K_roll*(pilot_u1-pilot_u2) ));

    	stick_decay();
    elif pilot_t1 == 2 :
    	stick_iter(stick_state,0.001);

    	a_vel_dir = vector(a_vel.x,0,a_vel.z);
    	a_vel_dir = a_vel_dir/mag(a_vel_dir);

    	pitch_e = atan2(a_vel.y,mag(a_vel_dir))  - (pilot_u1 + pilot_u2);
    	pitch_i = pitch_i + (pitch_e + pitch_e1)*dt/2;
    	pitch_d = (pitch_e - pitch_e1)/dt;
    	pitch_com = asin(min(1,-K_trim*(0.75*pitch_e + 0.15*pitch_i + 0.01*pitch_d)/pow(air_vel,2) ));
    	pitch_e1 = pitch_e;

    	print("%10.3f" % pitch_e + "%10.3f" % (100*pitch_i)  + "%10.3f" % (pilot_u1 + pilot_u2))

    	a_z_dir = vector(a_z.x,0,a_z.z);
    	
    	print(dot(a_z_dir,a_vel/mag(a_vel)))

    	roll_e = -atan2(dot(a_z_dir,a_vel/mag(a_vel)), 1) - (pilot_u1 - pilot_u2);
    	roll_i = roll_i + (roll_e + roll_e1)*dt/2;
    	roll_d = (roll_e - roll_e1)/dt;
    	roll_com = 0*asin(min(1,-K_trim*(7.5*roll_e + 1.5*roll_i + 0.1*roll_d)/pow(air_vel,2) ));
    	roll_e1 = roll_e;

    	print("%10.3f" % roll_e + "%10.3f" % (100*roll_i)  + "%10.3f" % (pilot_u1 - pilot_u2))

    

    air_u1 = pitch_com - roll_com;
    air_u2 = pitch_com + roll_com;
    air_u3 = K_yaw*pilot_u3;
    air_t0 = air_t0_max*air_rho/rho*pilot_t0;
    
    if air_u1 > air_u_max:
        air_u1 = air_u_max;
    if air_u1 < air_u_min:
        air_u1 = air_u_min;
    if air_u2 > air_u_max:
        air_u2 = air_u_max;
    if air_u2 < air_u_min:
        air_u2 = air_u_min;
    if air_u3 > air_u_max:
        air_u3 = air_u_max;
    if air_u3 < air_u_min:
        air_u3 = air_u_min;
    
    
    ## iterate airplane in airplane frame
    # find forces acting on plane
    air_fc0 = rho_r*air_area*air_rho*pow(air_vel,2)*sin(air_alpha)*\
              vector(0,0,1);
    air_fc1 = rho_r*air_area_ac1*air_rho*pow(air_vel,2)*sin(air_alpha-air_u1)*\
              vector(0, sin(air_u1), cos(air_u1));
    air_fc2 = rho_r*air_area_ac2*air_rho*pow(air_vel,2)*sin(air_alpha-air_u2)*\
              vector(0, sin(air_u2), cos(air_u2));
    air_fc3 = rho_r*air_area_ac3*air_rho*pow(air_vel,2)*sin(air_beta-air_u3)*\
              vector(cos(air_u3), sin(air_u3),0);
    air_fc4 = rho_r*air_area_ac4*air_rho*pow(air_vel,2)*sin(air_beta)*\
              vector(1,0,0);

    air_force =  air_fc0 + air_fc1 + air_fc2 + air_fc3 + air_fc4 \
                + air_t0*vector(0,1,0) - 0.2*air_rho*pow(air_vel,2)*vector(0,1,0) - \
                4*pilot_s0*air_rho*pow(air_vel,2)*vector(0,1,0);
    air_tau = cross(air_rc0, air_fc0) + cross(air_rc1, air_fc1)\
              + cross(air_rc2, air_fc2) + cross(air_rc3, air_fc3)\
               + cross(air_rc4, air_fc4)\
               - 2200*air_rho*pow(air_vel,1)*air_wga;

    air_acc = air_force/air_m;
    air_acc_ang = air_tau/air_I;

    # all forces evaluated at this point
    ## finding velocities and distance, integration, ode solver

    # velocity from acceleration, zero order
    air_wga = air_wga + (air_acc_ang + air_acc_ang1)/2*dt;
    
    a_acc = vector(\
        dot(air_acc,vector(a_x.x,a_y.x,a_z.x)),\
        dot(air_acc,vector(a_x.y,a_y.y,a_z.y)),\
        dot(air_acc,vector(a_x.z,a_y.z,a_z.z)));
    
    a_vel = a_vel + (a_acc+ a_acc1)/2*dt + \
        + vector(0,-g,0)*dt;

    # position from velocity, 1st order trapezoidal
    a_pos = a_pos + (a_vel+a_vel1)/2*dt;

    a_y = rotate(a_y, axis=norm(a_x), angle=(air_wga.x+air_wga1.x)/2*dt);
    a_z = rotate(a_z, axis=norm(a_x), angle=(air_wga.x+air_wga1.x)/2*dt);
    a_x = rotate(a_x, axis=norm(a_z), angle=(air_wga.z+air_wga1.z)/2*dt);
    a_y = rotate(a_y, axis=norm(a_z), angle=(air_wga.z+air_wga1.z)/2*dt);
    a_x = rotate(a_x, axis=norm(a_y), angle=(air_wga.y+air_wga1.y)/2*dt);
    a_z = rotate(a_z, axis=norm(a_y), angle=(air_wga.y+air_wga1.y)/2*dt);
        
    # store old variables
    a_vel1 = a_vel;
    air_wga1 = air_wga;
    a_acc1 = a_acc;
    air_acc_ang1 = air_acc_ang;

    # check for crash
    if (a_pos.y <= 0):
        a_pos.y = 0;
        a_vel = vector(0,0,0);
        air_wga = vector(0,0,0);
        air_t0_max = 0;
        rho = 0;
    
    # draw airplane
    a_plane_fus.pos = a_pos;
    a_plane_fus.axis = a_y;
    a_plane_fus.up = a_z;
    a_plane_wing_l.pos = a_pos + 2*a_x;
    a_plane_wing_l.axis = a_y;
    a_plane_wing_r.pos = a_pos - 2*a_x;
    a_plane_wing_r.axis = a_y;
    a_plane_wing_l.up = a_z;
    a_plane_wing_r.up = a_z;
    a_plane_tail.pos = a_pos+0.1*a_z;
    a_plane_tail.axis = a_y;
    a_plane_tail.up = a_z;

    # update data_readout
    data_text =  "position:(%10.3f"%a_pos.x+"%10.3f"%a_pos.y+" %10.3f"%a_pos.z+ ")\n"\
                "velocity: %10.3f" % air_vel+ "(%10.3f"%norm(a_vel).x+\
            " %10.3f"%norm(a_vel).y+ " %10.3f"%norm(a_vel).z+ ")\n"\
                +"altitude: %10.3f" % a_pos.y+"\n"\
             +"g-force: %10.3f"%(air_acc.x/g)\
             + " %10.3f"%(air_acc.y/g) + " %10.3f"%(air_acc.z/g) +"\n"\
             +"g-torque: %10.3f"%(air_acc_ang.x/g)\
             + " %10.3f"%(air_acc_ang.y/g) + "% 10.3f"%(air_acc_ang.z/g) +"\n"\
             +"omega: %10.3f"%air_wga.x\
             + " %10.3f"%air_wga.y + " %10.3f"%air_wga.z +"\n"\
            + "aoa: %10.3f" %(air_alpha*180/pi) +"\n"\
            + "beta: %10.3f" %(air_beta*180/pi) +"\n"\
            + "actuators: %10.3f"%(air_u1*180/pi) + " % 10.3f"%(air_u2*180/pi)\
            + " % 10.3f"%(air_u3*180/pi) + " %10.3f"%air_t0 +"\n"\
            + "stick: %10.3f"%pilot_u1 + " % 10.3f"%pilot_u2\
            + " % 10.3f"%pilot_u3 + " %10.3f"%pilot_t0+"\n"\
            + "camera view: %i"%camera_view +"\n\r\n" ;
    #print "\r\n" * 100

    if (a_pos.y <= 0):
        break;

    #################################################################
    ## find horizon
    a_vel_dir = vector(a_vel.x,0,a_vel.z);
    a_vel_dir = a_vel_dir/mag(a_vel_dir);
    
    a_vel_dir_left = 30*a_vel_dir + 10*rotate(a_vel_dir, angle=pi/2, axis=(0,1,0));
    a_vel_dir_right = 30*a_vel_dir + 10*rotate(a_vel_dir, angle=-pi/2, axis=(0,1,0));

    
    for i in range(len(hor)):
        if (hor[i] == 0) :
            hor_obj[i].pos=[(a_pos + 30*a_vel_dir + 20*rotate(a_vel_dir, angle=pi/2, axis=(0,1,0))), \
                 (a_pos + 30*a_vel_dir + 20*rotate(a_vel_dir, angle=-pi/2, axis=(0,1,0)))];
        else :
            hor_obj[i].pos=[(a_pos + rotate(a_vel_dir_left, angle=(pi/180*hor[i]), axis=(-a_vel.z,0,a_vel.x))), \
                 (a_pos + rotate(a_vel_dir_right, angle=(pi/180*hor[i]), axis=(-a_vel.z,0,a_vel.x)))];
            hor_lab[i].pos=a_pos + rotate(a_vel_dir_left, angle=(pi/180*hor[i]), axis=(-a_vel.z,0,a_vel.x));
    #################################################################
    ## set up HUD

    heading = 180/pi*atan2(a_vel.z,a_vel.x) % 360;

    j = 0;
    for i in range(len(hud_h)):
        if abs(diff_angle( a_vel_dir,vector(cos(pi/180*hud_h[i]),0,sin(pi/180*hud_h[i])) )) < pi/8 :
            hud_head_obj[j].text = "%10.0f" % (hud_h[i]);
            hud_head_obj[j].pos = a_pos + 30*vector(cos(pi/180*hud_h[i]),0,sin(pi/180*hud_h[i]));
            j = j+1;
    for i in range(j,3) :
        hud_head_obj[i].text = "";

    hud1.pos = a_pos + 30*scene.forward;
    hud1.text = "%10.0f" % (1.94384*air_vel) +"\n %10.0f" % (3.28084*a_pos.y) \
                 + "(%1.0f" %pilot_t1 +")"+ "\n%10.2f" % pilot_t0 + "(%1.0f" %pilot_s0 +")";
    hud2.pos = a_pos + 30*scene.forward;
    hud2.text = "%10.1f" % (air_acc.z/g) +"\n %10.1f" % (air_alpha*180/pi) \
                +"\n %10.1f" % (air_beta*180/pi);
    hud_aoa.pos = a_pos + 30*a_y ;
    hud_aoa.axis = a_y;

    hud_velo1.pos = a_pos + 30*a_vel/mag(a_vel) ;
    hud_velo1.axis = a_vel;
    hud_velo2.pos = [a_pos + 30*a_vel/mag(a_vel) + a_x, a_pos + 30*a_vel/mag(a_vel) - a_x ];
    hud_velo2.axis = a_vel;
    
    #################################################################
    ## find limit trajectories
#    K_z = 0.00093187*pow(air_vel,2);
    K_z = 0.00129*pow(air_vel,2);
    K_y = -0.00017*pow(air_vel,2)+air_t0/air_m\
           - 0.05*air_rho*air_vel;
    K_t = air_t0_max/air_m/g;

    crash_pos = vector(0,0,0);
    c0_pos = a_pos;
    c0_vel = a_vel;
    c0_acc = a_acc+vector(0,-g,0);
    c1_pos = a_pos;
    c1_vel = a_vel;
    c1_acc = K_z*a_z+vector(0,-g,0)+K_y*a_y;
    c2_pos = a_pos;
    c2_vel = a_vel;
    c2_acc = -K_z*a_z+vector(0,-g,0)+K_y*a_y;
    c3_pos = a_pos;
    c3_vel = a_vel;
    c3_acc = -K_z*a_x+vector(0,-g,0)+K_y*a_y;
    c4_pos = a_pos;
    c4_vel = a_vel;
    c4_acc = K_z*a_x+vector(0,-g,0)+K_y*a_y;

    traj.pos = a_pos;
    traj1.pos = a_pos; # up
    traj2.pos = a_pos; # down
    traj3.pos = a_pos; # port
    traj4.pos = a_pos; # starboard

    crash_0 = False; # 0 true, 1 false
    crash_1 = False;
    crash_2 = False;
    crash_3 = False;
    crash_4 = False;

    for i in range(0, min(int(20*t_amt/air_vel),50) ):
        c0_pos = c0_pos + (c0_vel)*t_scale*dt;
        c0_vel = c0_vel + (c0_acc)*t_scale*dt;
        c0_acc = rotate(c0_acc, axis=a_x, angle=air_wga.x*dt);
        c1_pos = c1_pos + (c1_vel)*t_scale*dt;
        c1_vel = c1_vel + (c1_acc)*t_scale*dt;
        c1_acc = rotate(c1_acc, axis=a_x, angle= -air_wga.x*dt);
        c2_pos = c2_pos + (c2_vel)*t_scale*dt;
        c2_vel = c2_vel + (c2_acc)*t_scale*dt;
        c2_acc = rotate(c2_acc, axis=a_x, angle= air_wga.x*dt);
        c3_pos = c3_pos + (c3_vel)*t_scale*dt;
        c3_vel = c3_vel + (c3_acc)*t_scale*dt;
        c3_acc = rotate(c3_acc, axis=a_x, angle= air_wga.x*dt);
        c4_pos = c4_pos + (c4_vel)*t_scale*dt;
        c4_vel = c4_vel + (c4_acc)*t_scale*dt;
        c4_acc = rotate(c4_acc, axis=a_x, angle= air_wga.x*dt);
            
        traj.append( pos= c0_pos);
        traj1.append( pos= c1_pos);
        traj2.append( pos= c2_pos);
        traj3.append( pos= c3_pos);
        traj4.append( pos= c4_pos);

        if (crash_0 == False)&(c0_pos.y <= 0):
            crash0_pos = c0_pos;
            crash_0 = True;
        if (crash_4 == False)&(c4_pos.y <= 0):
            crash_4 = True;
            crash4_pos = c4_pos;
        if (crash_3 == False)&(c3_pos.y <= 0):
            crash_3 = True;
            crash3_pos = c3_pos;
        if (crash_2 == False)&(c2_pos.y <= 0):
            crash_2 = True;
            crash2_pos = c2_pos;
        if (crash_1 == False)&(c1_pos.y <= 0):
            crash_1 = True;
            crash1_pos = c1_pos;

    # crash decision
    if (crash_0==True):
        crash_warning = "on crash course, "
        if (crash_4==True)&(crash_3==False):
            crash_correction = "roll left, pull up";
        if (crash_4==False)&(crash_3==True):
            crash_correction = "roll right, pull up";
        if (crash_1==True)&(crash_2==False):
            crash_correction = "roll full, pull up";
        if (crash_1==False)&(crash_2==True):
            crash_correction = "pull up";
        if (crash_4==True)&(crash_3==True)&(crash_1==True)&(crash_2==True):
            if mag(a_pos-crash0_pos) > \
               1000*(K_t-g/air_m*(a_vel.y)/mag(a_vel)): # min safe altitude
                if mag(a_pos-crash3_pos) > mag(a_pos-crash4_pos):
                    crash_correction = "speed up and roll left, pull up";
                if mag(a_pos-crash4_pos) > mag(a_pos-crash3_pos):
                    crash_correction = "speed up and roll left, pull up";
                if mag(a_pos-crash2_pos) > mag(a_pos-crash1_pos):
                    crash_correction = "speed up and roll full, pull up";
                if mag(a_pos-crash1_pos) > mag(a_pos-crash2_pos):
                    crash_correction = "speed up and pull up";
                if (mag(a_pos-crash1_pos) == mag(a_pos-crash2_pos))&\
                    (mag(a_pos-crash3_pos) == mag(a_pos-crash4_pos)):
                    crash_correction = "speed up";
            else:
                crash_correction = "eject";
    else:
        if (crash_4==True):
            crash_warning = "safe, crash if right, ";
            crash_correction = "";
        if (crash_3==True):
            crash_warning = "safe, crash if left, ";
            crash_correction = "";
        if (crash_2==True):
            crash_warning = "safe, crash if down, ";
            crash_correction = "";
        if (crash_1==True):
            crash_warning = "safe, crash if up, ";
            crash_correction = "";
        if (crash_4==False)&(crash_3==False)&(crash_1==False)&(crash_2==False):
            crash_warning = "safe";
            crash_correction = "";


    #################################################################
    if mod(t,0.5) < dt:
        t = 0;
        print(data_text);
        print(crash_warning+crash_correction+"\r\n");
        

    # update scene
    scene.center = a_plane_fus.pos + 0.2*a_z;

    # set camera view based on value in camera_view
    if camera_view == 0:
        scene.forward = 0.1*a_y + 1*norm(a_vel);
        scene.up = a_z;
  #      scene.autoscale = False;  
    if camera_view == 1:
        scene.forward = -0.1*a_y - 1*norm(a_vel);
        scene.up = a_z;
 #       scene.autoscale = False;
    if camera_view == 2:
        scene.forward = 1*a_y + 0*norm(a_vel) - 0.2*a_z;
        scene.up = a_z;
#        scene.autoscale = False;
    if camera_view == 3:
#        scene.autoscale = False;
        scene.up = vector(0,1,0);

    # pause
    while (pause == 0):
        rate(1/dt);
        if pause == 1:
            break;
    
    #if t == 0.: # make sure everything is set up before first visible display
    scene.visible = 1;
    t = t+dt;
