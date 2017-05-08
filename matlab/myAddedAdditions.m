%% myChangedAdditions
%
% *Date:* 2016-03-14
% *Author:* Sona Praneeth Akula
% *Input*
%
% 	_*data:*_ Data matrix of 3D world co-ordinates. 
% 	*Dimension:* 3*numberOfPoints
%
% 	_*T:*_ 	Indices of points denoting to which plane they belong.
%	*Dimension:* numberOfPoints*1 ie., Point having index i belongs to plane T[i]
%
% *Output*
%
% 

function myAddedAdditions(data1, planeIndices1)

startTime = tic;

minPointsPerPlane = 50; % Minimum number of data points to be present in a plane
numberOfPointsInData = size(planeIndices1,1); % Number of points in 'data'
a = unique(planeIndices1); % Plane indices
pointsPerPlane1 = [a, histc(planeIndices1(:),a)]; % Number of points in each plane. Format: Plane Index - No. of points

%% Step 1: Consider the planes which are having 'minPointsPerPlane' points in thier plane
planesInConsideration = [];
for i=1:size(pointsPerPlane1,1)
	if (pointsPerPlane1(i, 2) > minPointsPerPlane)
		planeIndex = pointsPerPlane1(i,1); % Index of the selected plane satisfying the criteria
		planesInConsideration = [planesInConsideration; planeIndex];
	end
end

%% Step 2: Extract points for the planes selected
numberOfPlanes = size(planesInConsideration,1); % Number of planes selected
data2 = []; % New set of points after removing unnecessary planes
data2PlaneIndices = [];
for i=1:size(data1,2)
	for j=1:size(planesInConsideration,1)
		if(planeIndices1(i,1)==planesInConsideration(j,1))
			%fprintf(1, 'Adding %d point to data2\n', i);
			data2 = [data2, data1(:,i)];
			data2PlaneIndices = [data2PlaneIndices, planeIndices1(i,1)];
			break;
		end
	end
end

% uniquePlaneIndices = unique(data2PlaneIndices);
% newData2 = [];
% newData2PlaneIndices = [];
% newData2Parameters = [];
% for i=1:length(uniquePlaneIndices)
% 	data2New = data2(:,find(data2PlaneIndices==uniquePlaneIndices(i)));
% 	newData2 = [newData2, data2New];
% 	newData2PlaneIndices = [newData2PlaneIndices, i*ones(1,length(data2New))];
% 	newData2Parameters = [newData2Parameters, fittingfn_plane(data2New)'];
% end

[paramsData2] = ShowResults(data2, @visualfn_plane, @fittingfn_plane, 50, data2PlaneIndices);

% savedPoints = [newData2];
% savedLabels = [newData2PlaneIndices];
% savedPlaneParameters = [newData2Parameters];
% save('pointsAndLabelsWithoutKmeans.mat','savedPoints','savedLabels','savedPlaneParameters');

%% Step 3: Performing K-means for re-clustering of points
planeIndices2 = kmeans(data2', numberOfPlanes);
% Plotting Figure
figure;
[params2] = ShowResults(data2, @visualfn_plane, @fittingfn_plane, 50, planeIndices2);
xlabel('x axis');ylabel('y axis');zlabel('z axis');axis equal;
title('Plotting data points after removing points from unncessary planes and re-clustering using K-means');


%% Step 4: Calculating the distances of all points to their particular planes
% Dimension: (numberOfPoints in data2)*(2*numberOfPlanes)
% size(data2,2) = numberOfPoints in data2
distanceMatrix = zeros(size(data2,2), 2*numberOfPlanes);
planeParameters = []; %  For a, b, c, d in ax+by+cz+d=0
for i=1:size(data2,2)
	% 3D co-ordinates of point
	x0 = data2(1, i);
	y0 = data2(2, i);
	z0 = data2(3, i);
	% Plane Parameters for the point x_0, y_0, z_0
	a = params2(planeIndices2(i,1),1);
	b = params2(planeIndices2(i,1),2);
	c = params2(planeIndices2(i,1),3);
	d = params2(planeIndices2(i,1),4);
	planeParameters = [planeParameters, [a;b;c;d;]];
	value = (a*x0)+(b*y0)+(c*z0)+d;
	if(value>=0)
		% Towards +ve normal
		distance = abs(((a*x0)+(b*y0)+(c*z0)+d)/sqrt(a^2+b^2+c^2));
	else
		% If it is on other side of plane. Towards -ve normal
		distance = abs(((-a*x0)+(-b*y0)+(-c*z0)-d)/sqrt(a^2+b^2+c^2));
	end
	% Column 1:numberOfPlanes = distance
	% Column 1+numberOfPlanes:end = true if belongs to planeNo (columnNo-numberOfPlanes)
	distanceMatrix(i,planeIndices2(i,1)) = distance;
	distanceMatrix(i,planeIndices2(i,1)+numberOfPlanes) = i;
end


%% Step 5: Removing points which are much farther from the plane
dataThree = []; % new data after k-means and and removing points > threshold
newDataAfterKmeans = [];
planeIndices3 = [];
plane3DProjections = [];
abcd = [];
savedPoints = [];
savedLabels = [];
savedPlaneParameters = [];

for i=1:numberOfPlanes
	% Indices of points for the plane i in distanceMatrix(:,i)
	planePointIndices = distanceMatrix(:,i+numberOfPlanes);
	planePointIndices = planePointIndices(planePointIndices>0);
	planePoints = data2(:,planePointIndices);
	planeDistances = distanceMatrix(:,i);
	planeDistances = planeDistances(planePointIndices);
	% Threshold for removing points farther from plane. We're using 95 percentile
	threshold = prctile(planeDistances,95);
	% Considering points which are within 95 percentile of distances
	planeDistancesAfterThreshold = planeDistances(planeDistances<=threshold);
	planePointIndicesAfterThreshold = planePointIndices(planeDistances<=threshold);
	newDataAfterKmeans = [newDataAfterKmeans, data2(:,planePointIndicesAfterThreshold)];
	planePoints = data2(:,planePointIndicesAfterThreshold);
	% Creating new plane indices
	planeIndices3 = [planeIndices3; i*ones(size(planePoints,2),1)];
	% Performing projection of the points onto plane i
	planeParametersForThisPlane = planeParameters(:,planePointIndicesAfterThreshold);
	abc = planeParametersForThisPlane(1:3,1);
	d = planeParametersForThisPlane(4,1);
	t = (sum(bsxfun(@times,planePoints,(-1.0).*abc),1)+(-1.0).*d)./(sum(abc.^2));
	result = planePoints+(abc*t);
	plane3DProjections = [plane3DProjections, result];
	% Testing data
	if i<=10
		points = result; % Dimesion: 3*someNoOfPoints
		labels = i*ones(1,size(points,2));
		plane_parameters = planeParametersForThisPlane(1:4,1);
		savedPoints = [savedPoints, points];
		savedLabels = [savedLabels, labels];
		savedPlaneParameters = [savedPlaneParameters, plane_parameters];
	end
	abcd = [abcd, planeParametersForThisPlane(1:4,1)];
end
save('newPointsAndLabelsGazebo.mat','savedPoints','savedLabels','savedPlaneParameters');

%% Plotting figures
% Figure after doing K-means and removing points farther from the plane
figure;
[params3]=ShowResults(newDataAfterKmeans, @visualfn_plane, @fittingfn_plane, minPointsPerPlane, planeIndices3);
xlabel('x axis');ylabel('y axis');zlabel('z axis');axis equal;
title('Plotting data points after removing points farther from planes');
% Figure after projecting the left over points onto the plane
figure;
[params4]=ShowResults(plane3DProjections, @visualfn_plane, @fittingfn_plane, minPointsPerPlane, planeIndices3);
title('Plotting data points after projecting onto their respective planes');
xlabel('x axis');ylabel('y axis');zlabel('z axis');axis equal;



elapsedTime = toc(startTime);
fprintf(1, 'Time taken for myAddedAdditions: %f seconds.\n', elapsedTime);




end