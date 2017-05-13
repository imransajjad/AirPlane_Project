##########################################################
# environment and graphics
##########################################################

# scene create
scene = display(title='Airplane Simulation',
     x=0, y=0, width=1024, height=768,
     center=(0,10,0), background=(0,0.5,1),range=scene_size );
scene.visisble = 0;
scene.autoscale = 1;
scene.center = a_pos;
scene.bind('keydown', keyPress);
scene.bind('keyup', keyRelease);

# ground
ground_box = box(size=(300*scene_size,0.1,300*scene_size),\
                 material=materials.wood);
ground_box.pos = (a_pos.x,ground_box.y,a_pos.z);

### sun
##scene.lights = [];
##sun_sphere = sphere(pos=2000*scene_size*vector(1,1,1), radius=30000,\
##                    color=(1,1,0.5), material=materials.emissive);
##lamp = local_light(pos=2000*scene_size*vector(1,1,1), color=(1,1,0.5));

# draw airplane
a_plane_fus = cone(make_trail=True, radius=0.2, color=color.red);
a_plane_wing = box(size=(3,0.1,2), color=color.red);
a_plane_tail = box(make_trail=True, size=(2,0.5,0.01) , color=color.blue);

a_plane_fus.pos = a_pos;
a_plane_fus.axis = a_y;
a_plane_fus.up = a_z;
a_plane_wing.pos = a_pos;
a_plane_wing.axis = a_y;
a_plane_wing.up = a_z;
a_plane_tail.pos = a_pos+0.1*a_z;
a_plane_tail.axis = a_y;
a_plane_tail.up = a_z;


# data readout
data_text =  "";
crash_warning = "";
crash_correction = "";


# draw crash curve
global traj;
traj = curve( color = color.red );
traj_point = a_pos;
traj1 = curve( color = color.cyan );
traj1_point = a_pos;
traj2 = curve( color = color.cyan );
traj2_point = a_pos;
traj3 = curve( color = color.cyan );
traj3_point = a_pos;
traj4 = curve( color = color.cyan );
traj4_point = a_pos;

t_amt = 300;
t_scale = 20;

# draw horizon angles
global hor;
global hor_obj;
hor = [-90, -45, -25, -10, 0, 10, 25, 45, 90];

hor_obj = [-90, -45, -25, -10, 0, 10, 25, 45, 90];
hor_lab = [-90, -45, -25, -10, 0, 10, 25, 45, 90];
for i in range(len(hor)):
    hor_obj[i] = curve( color = color.green, pos=[a_pos ,a_pos] );
    if hor[i] != 0 :
        hor_lab[i] = label(pos=a_pos, text='%10.0f'% hor[i], height=10, color=color.green, \
                            linecolor=color.green, line=False, box=False, font='sans', opacity=0);
    else :
        hor_lab[i] = label(pos=a_pos, text="", height=10, color=color.green, \
                            linecolor=color.green, line=False, box=False, font='sans', opacity=0);

# draw hud
hud1 = label(pos=a_pos, text='', xoffset=-200,
    yoffset=12,  height=15, color=color.green, linecolor=color.green, line=False, font='sans', opacity=0);
hud2 = label(pos=a_pos, text='', xoffset=200,
    yoffset=12,  height=15, color=color.green, linecolor=color.green, line=False, font='sans', opacity=0);

hud_h = [x * 15 for x in range(0, 24)];
hud_head_obj = [0,0,0,0];

for i in range(0,3):
    hud_head_obj[i] = label(pos=a_pos, text='%10.0f'% hud_h[i], height=10, color=color.green, \
                            linecolor=color.green, line=False, box=False, font='sans', opacity=0);

hud_aoa = ring(pos=(a_pos), axis=a_vel, radius=0.15, thickness=0.02, \
                material=materials.unshaded, color=color.green, linecolor=color.green)

hud_velo1 = ring(pos=(a_pos), axis=a_vel, radius=0.3, thickness=0.02, \
                material=materials.unshaded, color=color.green, linecolor=color.green)
hud_velo2 = curve(pos=(a_pos), axis=a_vel, thickness=0.05, \
                material=materials.unshaded, color=color.green, linecolor=color.green)