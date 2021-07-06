%% simulation data init

clear all
close all


% simulation time
DynOpt.t_start = 0;
DynOpt.Tend = 19;
DynOpt.Ts = 1e0;


% plot and graphics
DynOpt.plot = 0;
DynOpt.print = 1;

% montecarlo or single simulation
DynOpt.randstart = 0;
DynOpt.control = 0;
DynOpt.noise_enable = 1;
DynOpt.montecarlo = 0;

%%%%% OBSERVER %%%%%
DynOpt.ObserverOn = 1;
DynOpt.Observer = 'OPT';

DynOpt.model = 'satellite';