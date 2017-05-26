[plane_bounding_box_points, plane_parameters] = read_params_from_file('Plane_Info.txt', 0);
[num_planes, other] = size(plane_parameters);
x = plane_bounding_box_points(:,1);
y = plane_bounding_box_points(:,2);
z = plane_bounding_box_points(:,3);
num_points = size(x);
start_index = 1;
end_index = start_index+4;
distance = -1.0;
for plane_index=1:num_planes
    fprintf(1, 'Plotting plane %d\n', plane_index);
    curr_plot_x = x(start_index:end_index, :);
    curr_plot_y = y(start_index:end_index, :);
    curr_plot_z = z(start_index:end_index, :);
    plot3(curr_plot_x, curr_plot_y, curr_plot_z); hold on;
    curr_center_x = sum(curr_plot_x(1:4))/4;
    curr_center_y = sum(curr_plot_y(1:4))/4;
    curr_center_z = sum(curr_plot_z(1:4))/4;
    curr_right_center_x = sum(curr_plot_x(2:3))/2;
    curr_right_center_y = sum(curr_plot_y(2:3))/2;
    curr_right_center_z = sum(curr_plot_z(2:3))/2;
    curr_normal_x = [curr_center_x, curr_center_x+distance*plane_parameters(plane_index,1)];
    curr_normal_y = [curr_center_y, curr_center_y+distance*plane_parameters(plane_index,2)];
    curr_normal_z = [curr_center_z, curr_center_z+distance*plane_parameters(plane_index,3)];
    plot3(curr_normal_x, curr_normal_y, curr_normal_z); hold on;
    start_index = start_index+5;
    end_index = end_index+5;
    if plane_index < num_planes
        curr_plane_midpoint = [curr_center_x, curr_center_y, curr_center_z]; 
        next_plot_x = x(start_index:end_index, :);
        next_plot_y = y(start_index:end_index, :);
        next_plot_z = z(start_index:end_index, :);
        next_center_x = sum(next_plot_x(1:4))/4;
        next_center_y = sum(next_plot_y(1:4))/4;
        next_center_z = sum(next_plot_z(1:4))/4;
        next_normal_x = [next_center_x, next_center_x+distance*plane_parameters(plane_index+1,1)];
        next_normal_y = [next_center_y, next_center_y+distance*plane_parameters(plane_index+1,2)];
        next_normal_z = [next_center_z, next_center_z+distance*plane_parameters(plane_index+1,3)];
        next_plane_midpoint = [next_center_x, next_center_y, next_center_z]; 
        lambda = (next_plane_midpoint(1)-curr_plane_midpoint(1))*plane_parameters(plane_index,2) ...
                    - (next_plane_midpoint(2)-curr_plane_midpoint(2))*plane_parameters(plane_index,1);
        denominator = (plane_parameters(plane_index,1)*plane_parameters(plane_index+1,2)) - ...
                        (plane_parameters(plane_index+1,1)*plane_parameters(plane_index,2));
        lambda = lambda / denominator;
        t = ( plane_parameters(plane_index+1,1)*lambda + next_plane_midpoint(1)...
                - curr_plane_midpoint(1) )/ plane_parameters(plane_index,1);
        int_point_x = plane_parameters(plane_index,1)*t+curr_plane_midpoint(1);
        % int_point_x = plane_parameters(plane_index+1,1)*lambda+next_plane_midpoint(1)
        int_point_y = plane_parameters(plane_index,2)*t+curr_plane_midpoint(2);
        % int_point_y = plane_parameters(plane_index+1,2)*lambda+next_plane_midpoint(2)
        int_point_z = plane_parameters(plane_index,3)*t+curr_plane_midpoint(3);
        % int_point_z = plane_parameters(plane_index+1,3)*lambda+next_plane_midpoint(3)
        new_curr_normal_x = [curr_normal_x(1), int_point_x];
        new_curr_normal_y = [curr_normal_y(1), int_point_y];
        new_curr_normal_z = [curr_normal_z(1), int_point_z];
        new_next_normal_x = [next_normal_x(1), int_point_x];
        new_next_normal_y = [next_normal_y(1), int_point_y];
        new_next_normal_z = [next_normal_z(1), int_point_z];
        plot3(new_curr_normal_x, new_curr_normal_y, new_curr_normal_z);
        plot3(new_next_normal_x, new_next_normal_y, new_next_normal_z);
        angle = findAngle(plane_parameters(plane_index, 1:3), plane_parameters(plane_index+1, 1:3))
        angle = angle/2
        % new_distance = distance/cos((angle*180)/3.14159)
        new_distance = distance/cos((angle*3.14159)/180)
        dir = -[curr_right_center_x-int_point_x, curr_right_center_y-int_point_y, curr_right_center_z-int_point_z];
        dest_point_x = curr_right_center_x+new_distance*dir(1);
        dest_point_y = curr_right_center_y+new_distance*dir(2);
        dest_point_z = curr_right_center_z+new_distance*dir(3);
        mid_normal_x = [curr_right_center_x, dest_point_x];
        mid_normal_y = [curr_right_center_y, dest_point_y];
        mid_normal_z = [curr_right_center_z, dest_point_z];
        plot3(mid_normal_x, mid_normal_y, mid_normal_z);
    end
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');