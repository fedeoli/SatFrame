%%%%%%% Set the plot preferences %%%%%%%

% Plot layout
% set(0, 'defaultAxesFontName', 'normal')
set(0, 'defaultAxesFontSize', 18)
set(0, 'defaultAxesFontWeight', 'bold')
set(0, 'defaultAxesGridAlpha', 1)
set(0, 'defaultFigureColor', 'white')
set(0, 'defaultLineLineWidth', 2)
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% Time vector format
if strcmpi(params.PlotTimeFormat, 'hours')
    
    time_plot = time./3600;
    time_label = 'Time $(h)$';
    
elseif strcmpi(params.PlotTimeFormat, 'seconds')
    
    time_plot = time;
    time_label = 'Time $(s)$';
    
end


%%%%%%% Orbital Plots %%%%%%%

OrbitalPlot_V2_1;


%%%%%%% Attitude Plots %%%%%%%

if params.Attitude
    
    AttitudePlot_V2_2;
    
end


%%%%%%% Animation %%%%%%%

% Animation_V1_1;
