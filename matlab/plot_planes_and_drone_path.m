clear all;
clc;
close all;

[plane_bounding_box_points, plane_parameters] = read_params_from_file('test.txt', 0);
x = plane_bounding_box_points(:,1);
y = plane_bounding_box_points(:,2);
z = plane_bounding_box_points(:,3);
num_points = size(x);
num_planes = num_points(1,1)/5;
start_index = 1;
end_index = start_index+4;
for plane_index=1:num_planes
    fprintf(1, 'Plotting plane %d\n', plane_index);
    plot_x = x(start_index:end_index);
    plot_y = y(start_index:end_index);
    plot_z = z(start_index:end_index);
    plot3(plot_x, plot_y, plot_z); hold on;
    center_x = sum(plot_x(1:4))/4;
    center_y = sum(plot_y(1:4))/4;
    center_z = sum(plot_z(1:4))/4;
    normal_x = [center_x, center_x+plane_parameters(plane_index,1)];
    normal_y = [center_y, center_y+plane_parameters(plane_index,2)];
    normal_z = [center_z, center_z+plane_parameters(plane_index,3)];
    plot3(normal_x, normal_y, normal_z); hold on;
    % plot3(normal_x(1,plane_index), normal_y(1,plane_index), normal_z(1,plane_index)); hold on;
    start_index = start_index+4;
    end_index = end_index+4;
end
% plot3(normal_x, normal_y, normal_z);
