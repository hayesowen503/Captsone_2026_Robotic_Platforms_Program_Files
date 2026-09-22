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
alpha = atan2(-1*ny, nz);
beta = asin(nx);
R_xy = [      cos(beta)              0               sin(beta)
        sin(alpha)*sin(beta)    cos(alpha)    -sin(alpha)*cos(beta)
       -cos(alpha)*sin(beta)    sin(alpha)     cos(alpha)*cos(beta) ];

% Parasitic Translation Solver
r = 0.07823;      % Radius from platform center to joint mounts (meters)
gamma = [pi/6, 5*pi/6, 3*pi/2]; % Base leg angles in radians

A = zeros(3, 2);
B = zeros(3, 1);

for i = 1:3
    % Local platform joint point
    p_i = [r * cos(gamma(i)); r * sin(gamma(i)); 0];
    
    % Rotated joint point relative to center
    v_i = R_xy * p_i;
    
    % Setup linear equation coefficients based on radial constraint
    sin_g = sin(gamma(i));
    cos_g = cos(gamma(i));
    
    % Left side of equation [xp_coeff, yp_coeff]
    A(i, :) = [-sin_g, cos_g];
    
    % Right side of equation (known constants)
    B(i) = v_i(1) * sin_g - v_i(2) * cos_g;
end

% Solve the overdetermined system for [par_x; par_y] via least squares
translation_xy = A \ B; 

par_x = translation_xy(1);
par_y = translation_xy(2);
par_z = 0; % Z is not parasitic
origin2platform = [ R_xy(1,1)  R_xy(1,2)  R_xy(1,3)  par_x
                    R_xy(2,1)  R_xy(2,2)  R_xy(2,3)  par_y
                    R_xy(3,1)  R_xy(3,2)  R_xy(3,3)  par_z
                        0          0          0        1   ];

aim_point = origin2platform * platform2camera * cam_point;

disp('Final Target Coordinates:')
disp(aim_point(1:3))