cam_x = 0.34;
cam_y = 0.39;
cam_z = 1.47;
cam_point = [cam_x
             cam_y
             cam_z
              1];

cam_x_offset = 0.03;
cam_y_offset = 0.05;
cam_z_offset = 0.036;
platform2camera = [-1   0   0   cam_x_offset
                   0  -1   0   cam_y_offset
                   0   0   1   cam_z_offset
                   0   0   0        1];

dir = -0.5;
tilt = 0.5;
nx = cos(dir)*sin(tilt);
ny = sin(dir)*sin(tilt);
nz = cos(tilt);
alpha = atan2(-1*ny,nz);
beta = asin(nx);
origin2platform = [      cos(beta)              0               sin(beta)          0
                    sin(alpha)*sin(beta)    cos(alpha)    -sin(alpha)*cos(beta)    0
                   -cos(alpha)*sin(beta)    sin(alpha)     cos(alpha)*cos(beta)    0
                             0                  0                    0             1   ];


aim_point = origin2platform*platform2camera * cam_point;
disp('Final Target Coordinates:')
disp(aim_point(1:3))