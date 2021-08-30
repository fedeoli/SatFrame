%% test sigma analysis in theory

dirname = 'simulations/Singleshot/SigmaAnalysis/';
fileinfo = dir(strcat(dirname,'*.mat'));
fnames = {fileinfo.name};
load(strcat(dirname,fnames{end}));

% extract distances
adj_UWB = DynOpt.ObserverTest.MeasuredDistances;
adj_GPS = setAdjacencyMatrixNorm(DynOpt.y_GPS(DynOpt.ObserverTest.PositionArrayIndex,end),DynOpt.ObserverTest.Nagents);
adj_dist = setDistMatrix(DynOpt.y_GPS(DynOpt.ObserverTest.PositionArrayIndex,end),DynOpt.ObserverTest.Nagents);

dij_UWB = mean(nonzeros(triu(adj_UWB)));
dij_GPS = mean(nonzeros(triu(adj_GPS)));

dist = zeros(3,1);
for i=1:3
   dist(i) =  mean(nonzeros(adj_dist(:,:,i)));
end


% agents
N = 10;

% ranges
N_range = 2:N;
step = 5e-2;
theta_range = step:step:(1-step);

% sigma
sigma_GPS = DynOpt.ObserverTest.ErrorAmplitudeGPS;
sigma_UWB = DynOpt.ObserverTest.ErrorAmplitudeUWB;

% meshgrid
[X, Y] = meshgrid( N_range, theta_range );
Z = SigmaAnalysis(X,Y,sigma_GPS,sigma_UWB,dij_GPS,dij_UWB,dist);
colormap autumn
surf(X, Y, Z, 'FaceAlpha','0.5');

hold on

% surf the simulations
dirname = 'simulations/Singleshot/SigmaAnalysis/';
fileinfo = dir(strcat(dirname,'*.mat'));
fnames = {fileinfo.name};
for s = 1:length(fnames)
    load(strcat(dirname,fnames{s}));
    for i = 1:DynOpt.ObserverTest.Nagents
        tmp_sigma(i) = mean(DynOpt.out(i).sigma_p_mean);
    end

    sigma_p_mean = mean(tmp_sigma);
%     plot3(DynOpt.ObserverTest.Nagents,DynOpt.ObserverTest.theta,sigma_p_mean,'ro');
    x(s,1) = DynOpt.ObserverTest.Nagents;
    y(s,1) = DynOpt.ObserverTest.theta;
    z(s,1) = sigma_p_mean;
end
% load 'simulations/Singleshot/SigmaAnalysis/recap.mat';

f = fit( [x, y], z, 'cubicinterp' );

freezeColors;
plot(f, [x,y], z);
colormap winter

legend('Theo','Fit','Data')
xlabel('N')
ylabel('\theta')
zlabel('\Sigma')
