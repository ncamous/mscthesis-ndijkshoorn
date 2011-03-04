% SETTINGS
global settings
settings = {};
settings.usarsim_ip = 'localhost';
settings.usarsim_port = 3000;

addpath('interfaces/usarsim', 'ardrone', 'ui');


% ARDRONE
ardrone ();


% INTERFACE
interface ();


% CONTROLS
%controls_start ();
%ardrone_set ('AltitudeVelocity', 0.1);


% GUI
%ui ();
%mainGUIdata  = guidata(ui);
%global ui_imagebox;
%ui_imagebox = mainGUIdata.imageBox;
%ui_display_image('7');
%image_receiver ();