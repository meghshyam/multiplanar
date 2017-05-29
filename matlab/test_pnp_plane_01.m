clear all;clc;close all;

plot_planes_normal
[obj_points] = read_object_points(1);
[rot_cam_points, cam_points] = read_camera_points(1);
yaw = 0.0767829;
% yaw = -0.994881;
rot_guess = eye(3,3);
rot_guess(1, 1) = cos(-yaw);
rot_guess(1, 3) = -sin(-yaw);
rot_guess(3, 1) = sin(-yaw);
rot_guess(3, 3) = cos(-yaw);
rot_guess_inv = inv(rot_guess);

rot_obj_points = rot_guess_inv*obj_points';
rot_obj_points = rot_obj_points';

% scatter3(rot_obj_points(:,1), rot_obj_points(:,3), -rot_obj_points(:,2), 'b*'); hold on;
scatter3(cam_points(:,1), cam_points(:,3), -cam_points(:,2), 'k*'); hold on;
% scatter3(rot_cam_points(:,1), rot_cam_points(:,3), -rot_cam_points(:,2), 'g*'); hold on;
% scatter3(obj_points(:,1), obj_points(:,3), -obj_points(:,2), 'r*'); hold on;
for index=1:size(rot_cam_points, 1)
    text(rot_cam_points(index,1), rot_cam_points(index,3), -rot_cam_points(index,2), num2str(index));
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');