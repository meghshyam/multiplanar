clear all;
clc;
close all;

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
points = [...
    [ 1.03342; 1.11459; -0.184419; 4.39934], ...
    [ 1.03342; 0.618535; -0.184419; 4.39934], ...
    [ 1.10695; 0.618535; -0.184419; 4.39934], ...
    [ 1.10695; 0.86554; -0.184419; 4.39934], ...
    [ 1.18049; 0.86554; -0.184419; 4.39934], ...
    [ 1.18049; 1.11255; -0.184419; 4.39934], ...
    [ 1.18049; 1.11255; 0.246269; 4.39934], ...
    [ 1.18049; 1.11255; 0.676957; 4.39934], ...
    [ 1.70117; 0.741016; 0.676957; 4.39934], ...
    [ 1.70117; -0.0536219; 0.676957; 4.39934], ...
    [ 2.22185; -0.0536219; 0.676957; 4.39934], ...
    [ 2.22185; -0.354249; 0.676957; 4.39934], ...
    [ 2.74252; -0.354249; 0.676957; 4.39934], ...
    [ 2.74252; -0.654876; 0.676957; 4.39934], ...
    [ 2.74252; -0.654876; 0.655443; 4.39934], ...
    [ 2.74252; -0.654876; 0.633928; 4.39934], ...
    [ 2.7307; -0.756836; 0.639431; 0.5617], ...
    [ 2.71889; -0.611791; 0.644934; -3.27594], ...
    [ 2.70707; -0.466746; 0.650437; -7.11358], ...
    [ 2.69525; -0.3217; 0.65594; -10.9512], ...
    [ 2.68343; -0.176655; 0.661443; -14.7889], ...
    [ 2.67161; -0.0316094; 0.666946; -18.6265], ...
    [ 2.65979; 0.113436; 0.672449; -22.4641], ...
    [ 2.64797; 0.258481; 0.677952; -26.3018], ...
    [ 2.63615; 0.403527; 0.683455; -30.1394], ...
    [ 2.62433; 0.548572; 0.688958; -33.9771], ...
    [ 2.61251; 0.693618; 0.694461; -37.8147], ...
    [ 2.60069; 0.838663; 0.699964; -41.6523], ...
    [ 2.58887; 0.983708; 0.705467; -45.49], ...
    [ 2.57706; 1.12875; 0.710971; -49.3276], ...
    [ 2.56524; 1.2738; 0.716474; -53.1652], ...
    [ 2.55342; 1.41884; 0.721977; -57.0029] ...
];
plot3(points(1,:), points(2,:), points(3,:));
points = [...
    [ 2.35259; 1.41884; 0.721977; -57.0029], ...
    [ 2.35259; 1.67157; 0.721977; -57.0028], ...
    [ 2.15177; 1.67157; 0.721977; -57.0027], ...
    [ 2.15177; 1.9243; 0.721977; -57.0027], ...
    [ 1.95094; 1.9243; 0.721977; -57.0026], ...
    [ 1.95094; 2.17703; 0.721977; -57.0025], ...
    [ 1.95094; 2.17703; 1.05195; -57.0025], ...
    [ 1.95094; 2.17703; 1.38193; -57.0025] ...
];
plot3(points(1,:), points(2,:), points(3,:));
points = [...
    [ 1.95094; 2.77703; 1.38193], ...
    [ 3.36898; 2.98546; 1.26546], ...
    [ 3.92507; 3.81901; 1.39821], ...
    [ 3.91809; 3.80242; 0.992204] ...
%     [ 3.39456; 3.15149; 0.576543], ...
%     [ 7.25674; 0.989851; 0.919431], ...
%     [ 1.38772; 2.48003; 0.610491], ...
%     [ 9.05682; 0.216265; 0.363832], ...
%     [ 2.30483; 3.72421; -0.0425368], ...
%     [ 3.94527; 3.86726; 0.602609], ...
%     [ 3.90833; 3.8029; 0.154573], ...
%     [ 2.3422; 3.72009; -1.07158], ...
%     [ 4.67477; 1.59195; 0.0936673], ...
%     [ 1.84069; 2.79725; 0.25489], ...
%     [ 1.27922; 3.01785; -0.549877], ...
%     [ 3.05322; 2.41052; -0.173221], ...
%     [ 3.44202; 3.04978; -0.191181], ...
%     [ 3.88356; 4.20891; -0.604003] ...
];
plot3(points(1,:), points(2,:), points(3,:));