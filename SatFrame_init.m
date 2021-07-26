%% simulation data init

clear all
close all


% simulation time
DynOpt.t_start = 0;
DynOpt.Tend = 500;
DynOpt.Ts = 1e0;


% plot and graphics
DynOpt.plot = 0;
DynOpt.print = 1;

% montecarlo or single simulation
DynOpt.randstart = 1;
DynOpt.control = 0;
DynOpt.montecarlo = 1;

% noise
DynOpt.noise_enable = 1;
DynOpt.true_pos = 1;
DynOpt.true_att = 0;

%%%%% OBSERVER %%%%%
DynOpt.ObserverOn_pos = 0;
DynOpt.ObserverOn_att = 1;
DynOpt.Observer_pos = 'EKF';
DynOpt.Observer_att = 'EKF';

DynOpt.model = 'satellite';