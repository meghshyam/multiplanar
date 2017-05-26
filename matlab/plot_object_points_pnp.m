clear all;clc; close all;

object_points = [1.320249438285828, -1.194388747215271, 2.270143985748291; 
    1.538088262081146, -1.194388747215271, 2.605618476867676; 
    1.755927085876465, -1.194388747215271, 2.941092729568481; 
    1.755070090293884, -0.9693932831287384, 2.941648960113525; 
    1.754213094711304, -0.7443978190422058, 2.942205667495728; 
    1.536374270915985, -0.7443978190422058, 2.606731176376343; 
    1.318535447120667, -0.7443978190422058, 2.271256923675537; 
    1.319392442703247, -0.9693932831287384, 2.270700216293335; 
    1.537231266498566, -0.9693932831287384, 2.60617470741272];

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

% [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(image_points, object_points);
camera_object = cameraParameters('IntrinsicMatrix', camera_matrix, ...
                                 'RadialDistortion', radial_distortion_matrix, ...
                                 'WorldUnits', 'm');
% camera_object.IntrinsicMatrix = camera_matrix;
[worldOrientation,worldLocation,inlierIndex,status] = estimateWorldCameraPose(image_points, object_points, camera_object, ...
                                    'Confidence', 99, 'MaxReprojectionError', 1.75)