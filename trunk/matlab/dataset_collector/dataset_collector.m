% SETTINGS
global settings
settings = {};
settings.usarsim_ip = 'localhost';
settings.usarsim_port = 3000;

addpath('usarsim');
init ();


% COMMUNCATIONS
%communication_start ();
%communication_send ('INIT {ClassName USARBot.ARDrone}{Location 0.0,0.0,0.8}{Name ARDrone}');


% CONTROLS
%controls_start ();


% GUI
ui_start ();