%% Test J-Linkage algorithm and additions to it
%
% *Initial Code Authors:* R.Toldo, A.Fusiello
% *Modifed Code Authors:* Sona Praneeth Akula
% *Date Modified:* 2016-03-14
% *Authors:* R.Toldo A.Fusiello, department of computer science - University of Verona.
% *Reference Paper:* R. Toldo, A. Fusiello. Robust Multiple Structures Estimation with J-linkage. Proceeding of the European Conference on Computer Vision, 2008.
%

startTime = tic;

%% F C Kohli Planes
% Parameters for the Algorithm
numberOfTrials = 5000; % M in the paper
ClusterThreshold = 75;
inliersThreshold = 0.1; % $\delta$ in the paper
sigmaExp = 0.5;

mix = csvread('points_gazebo_test.csv'); % Set of data points
data = mix'; % Dimension: 3*numberOfPoints

% Generate an exponential Cumulative distribution function(cdf) needed to generate a non-uniform sampling. Equation (1) Pg:3/12
[nearPtsTab] = calcNearPtsTab(data, 'exp', sigmaExp);

% Generate Hypothesis (random sampling)
[totm, totd] = generateHypothesis(data, @getfn_plane, @distfn_plane, @degenfn_plane, 3, 4, 100,numberOfTrials, nearPtsTab);

% Perform J-Linkage clusterization
[T, Z, Y, totdbin] = clusterPoints(totd, inliersThreshold);

%% Plot results
figure(1);clf;
[paramsOne] = ShowResults(data, @visualfn_plane, @fittingfn_plane, 50, T);
axis equal;

%%  My additions to the code
myAddedAdditions(data, T);

elapsedTime = toc(startTime);
fprintf(1, 'Time taken for myTestJLinkage: %f seconds.\n', elapsedTime);