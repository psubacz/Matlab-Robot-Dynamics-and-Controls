function varargout = SensorEx_Client(varargin)
% SENSOREX_CLIENT MATLAB code for SensorEx_Client.fig
%      SENSOREX_CLIENT, by itself, creates a new SENSOREX_CLIENT or raises the existing
%      singleton*.
%
%      H = SENSOREX_CLIENT returns the handle to a new SENSOREX_CLIENT or the handle to
%      the existing singleton*.
%
%      SENSOREX_CLIENT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SENSOREX_CLIENT.M with the given input arguments.
%
%      SENSOREX_CLIENT('Property','Value',...) creates a new SENSOREX_CLIENT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SensorEx_Client_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SensorEx_Client_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SensorEx_Client

% Last Modified by GUIDE v2.5 23-Jan-2013 17:38:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SensorEx_Client_OpeningFcn, ...
    'gui_OutputFcn',  @SensorEx_Client_OutputFcn, ...
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


% --- Executes just before SensorEx_Client is made visible.
function SensorEx_Client_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SensorEx_Client (see VARARGIN)

% Choose default command line output for SensorEx_Client
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

Init_GUI(hObject, handles, false);

% UIWAIT makes SensorEx_Client wait for user response (see UIRESUME)
% uiwait(handles.SensorEx_GUI);


% --- Outputs from this function are returned to the command line.
function varargout = SensorEx_Client_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

function Init_GUI(fig_handle, handles, isreset)

clc;

handles.h_SensorEx = create_SensorEx(1);

addlistener(handles.h_SensorEx,'SensorsValues_Received',@SensorsValues_Received);
addlistener(handles.h_SensorEx,'File_Received',@File_Received);
addlistener(handles.h_SensorEx,'GraphInfos_Received',@GraphInfos_Received);
addlistener(handles.h_SensorEx,'CameraImage_Received',@CameraImage_Received);
addlistener(handles.h_SensorEx,'Connection_Changed',@Connection_Changed);
addlistener(handles.h_SensorEx,'IPClient_Changed',@IPClient_Changed);
addlistener(handles.h_SensorEx,'IPLocal_Changed',@IPLocal_Changed);

handles.h_SensorsTimeStamp=0;
handles.h_SensorsValues=[0,0,0,0,0,0,0];

set(handles.ValueLocalIPL, 'String', 'N/A');
set(handles.ValueClientIPL, 'String', 'N/A');

Assign_Var(handles);

guidata(handles.SensorEx_GUI, handles);

function Assign_Var(handles)
assignin('base','SE_SensorsValues',handles.h_SensorsValues);

function DeviceNumberCB_Callback(hObject, eventdata, handles)
x = get(hObject,'Value');
handles.h_SensorEx.SetDeviceNumber(x);

function DeviceNumberCB_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% Menu Events
% --------------------------------------------------------------------
function File_Menu_Callback(hObject, eventdata, handles)
function Exit_Menu_Callback(hObject, eventdata, handles)
delete(hObject);
close(handles.SensorEx_GUI);
function Tools_Menu_Callback(hObject, eventdata, handles)
function Help_Menu_Callback(hObject, eventdata, handles)
function Check_Update_Menu_Callback(hObject, eventdata, handles)
handles.h_SensorEx.CheckUpdate();
function Preferences_Menu_Callback(hObject, eventdata, handles)
% --------------------------------------------------------------------

% Sensor Ex Events
% --------------------------------------------------------------------
function SensorsValues_Received(source,arg)
fig = findall(0,'Type','figure','Name','SensorEx_Client');
handles = guidata(fig);

siz = size(handles.h_SensorsValues,1)+1;
handles.h_SensorsValues(siz,2)=arg.TimeSpan;
sumTime = sum(handles.h_SensorsValues(:,2))/1000;
handles.h_SensorsValues(siz,1)=sumTime;

for i = 3:8
    handles.h_SensorsValues(siz,i)=arg.Result(i-2);
end

set(handles.ValueAccXL, 'String', arg.Result(1));
set(handles.ValueAccYL, 'String', arg.Result(2));
set(handles.ValueAccZL, 'String', arg.Result(3));

set(handles.ValueOriXL, 'String', arg.Result(4));
set(handles.ValueOriYL, 'String', arg.Result(5));
set(handles.ValueOriZL, 'String', arg.Result(6));

Assign_Var(handles);
guidata(handles.SensorEx_GUI, handles);

function File_Received(source,arg)
% disp('Changed')

function GraphInfos_Received(source,arg)
% disp('Changed')

function CameraImage_Received(source,arg)
fig = findall(0,'Type','figure','Name','SensorEx_Client');
handles = guidata(fig);

try
    resultCamera = arg.Result;
    resultCameraSize = double(arg.Size);
    
    [R G B] = bytes2RGB(resultCamera, resultCameraSize);
    
    set(handles.ValueFPSL, 'String', arg.FrameRate);
    set(handles.ValueFPSAvgL, 'String', arg.FrameRateAvg);

    showRGB(R, G, B);
catch e
    e.message;
end

function Connection_Changed(source,arg)
% disp(arg.Result)

function IPClient_Changed(source,arg)
fig = findall(0,'Type','figure','Name','SensorEx_Client');
handles = guidata(fig);

f = sprintf('%d.%d.%d.%d',arg.IPAddress(1),arg.IPAddress(2),arg.IPAddress(3),arg.IPAddress(4));
set(handles.ValueClientIPL, 'String', f);

function IPLocal_Changed(source,arg)
fig = findall(0,'Type','figure','Name','SensorEx_Client');
handles = guidata(fig);

f = sprintf('%d.%d.%d.%d',arg.IPAddress(1),arg.IPAddress(2),arg.IPAddress(3),arg.IPAddress(4));
set(handles.ValueLocalIPL, 'String', f);
% --------------------------------------------------------------------

function SensorEx_GUI_CloseRequestFcn(hObject, eventdata, handles)
try
    handles.h_SensorEx.StopConnection;
catch e
    e.message;
end
delete(hObject);
