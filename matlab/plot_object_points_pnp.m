clear all;clc; close all;

camera_matrix = [565.710890694431, 0, 329.70046366652;
  0, 565.110297594854, 169.873085097623;
  0, 0, 1];

image_points = [0, 0; 
    320, 0; 
    640, 0; 
    640, 180;
    640, 360; 
    320, 360; 
    0, 360; 
    0, 180;
    320, 180];

radial_distortion_matrix = [-0.516089772391501; 0.285181914111246; -0.000466469917823537]; %0.000864792975814983; 0];

camera_object = cameraParameters('IntrinsicMatrix', camera_matrix, ...
                                 'WorldUnits', 'm');
                                 %'RadialDistortion', radial_distortion_matrix, ...


[object_points] = read_object_points();

startObjLocation = 1;
endObjLocation = 9;
[num_points, test] = size(object_points);
proj_points = zeros(num_points/9, 3);
grid_points = zeros(num_points, 1);
index = 1;
while endObjLocation <= num_points
    obj_points = [object_points(startObjLocation:endObjLocation, 1), ... 
                  object_points(startObjLocation:endObjLocation, 2), ...
                  object_points(startObjLocation:endObjLocation, 3)];
    [worldOrientation,worldLocation,inlierIndex,status] = ...
    estimateWorldCameraPose(image_points, obj_points, camera_object, ...
                            'Confidence', 99, 'MaxReprojectionError', 1.75, 'MaxNumTrials', 1000);
                        
    worldOrientation
    inv(worldOrientation)
    worldLocation
    inlierIndex
    status

    proj_points(index, :) = -inv(worldOrientation)*worldLocation';
    grid_points(startObjLocation:endObjLocation, 1) = index;
    
    startObjLocation = startObjLocation+9;
    endObjLocation = endObjLocation+9;
    index = index+1;
end
scatter3(object_points(:,1), -object_points(:,3), object_points(:,2), '*'); hold on;
scatter3(proj_points(:,1), -proj_points(:,3), proj_points(:,2), '*'); hold on;
for index=1:num_points
    text(object_points(index,1), -object_points(index,3), object_points(index,2), num2str(grid_points(index)));
end
for index=1:num_points/9
    text(proj_points(index,1), -proj_points(index,3), proj_points(index,2), num2str(index));
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');