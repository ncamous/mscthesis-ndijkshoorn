function varargout = ui(varargin)
% UI MATLAB code for ui.fig
%      UI, by itself, creates a new UI or raises the existing
%      singleton*.
%
%      H = UI returns the handle to a new UI or the handle to
%      the existing singleton*.
%
%      UI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UI.M with the given input arguments.
%
%      UI('Property','Value',...) creates a new UI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ui

% Last Modified by GUIDE v2.5 15-Feb-2011 20:42:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ui_OpeningFcn, ...
                   'gui_OutputFcn',  @ui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ui is made visible.
function ui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ui (see VARARGIN)

% Choose default command line output for ui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btnAltitudeVelocityP.
function btnAltitudeVelocityP_Callback(hObject, eventdata, handles)
% hObject    handle to btnAltitudeVelocityP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('AltitudeVelocity', 0.1);


% --- Executes on button press in btnAltitudeVelocityM.
function btnAltitudeVelocityM_Callback(hObject, eventdata, handles)
% hObject    handle to btnAltitudeVelocityM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('AltitudeVelocity', -0.1);


% --- Executes on button press in btnLinearVelocityP.
function btnLinearVelocityP_Callback(hObject, eventdata, handles)
% hObject    handle to btnLinearVelocityP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('LinearVelocity', 0.1);

% --- Executes on button press in btnLinearVelocityM.
function btnLinearVelocityM_Callback(hObject, eventdata, handles)
% hObject    handle to btnLinearVelocityM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('LinearVelocity', -0.1);

% --- Executes on button press in btnLateralVelocityM.
function btnLateralVelocityM_Callback(hObject, eventdata, handles)
% hObject    handle to btnLateralVelocityM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('LateralVelocity', -0.1);

% --- Executes on button press in btnLateralVelocityP.
function btnLateralVelocityP_Callback(hObject, eventdata, handles)
% hObject    handle to btnLateralVelocityP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('LateralVelocity', 0.1);


% --- Executes on button press in btnTakeOff.
function btnTakeOff_Callback(hObject, eventdata, handles)
% hObject    handle to btnTakeOff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnLand.
function btnLand_Callback(hObject, eventdata, handles)
% hObject    handle to btnLand (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnRotationalVelocityP.
function btnRotationalVelocityP_Callback(hObject, eventdata, handles)
% hObject    handle to btnRotationalVelocityP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('RotationalVelocity', 0.1);


% --- Executes on button press in btnRotationalVelocityM.
function btnRotationalVelocityM_Callback(hObject, eventdata, handles)
% hObject    handle to btnRotationalVelocityM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_set ('RotationalVelocity', -0.1);


% --- Executes on button press in btnReset.
function btnReset_Callback(hObject, eventdata, handles)
% hObject    handle to btnReset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ardrone_reset ();


% --- Executes during object deletion, before destroying properties.
function uipanel1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to uipanel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
	interface_close ();