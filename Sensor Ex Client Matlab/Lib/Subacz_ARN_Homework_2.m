%Peter Subacz - 2/6/18
%Advanced Robot Navigation - Homework 2

%% Create Sensor Ex object if doesnt exist (specify device number)
if(exist('m_SensorEx') == 0)
    m_SensorEx = create_SensorEx(1);
end
%% Retrieve Local IP
resultIPLocal(4) = uint8(0);
try
    resultIPLocal = uint8(m_SensorEx.ResultLocalIPAddress.IPAddress);
catch e
    e.message;
end
resultIPLocal
%% Retrieve Devices IP
resultIPDevice(4) = uint8(0);
try
    resultIPDevice = uint8(m_SensorEx.ResultDeviceIPAddress.IPAddress);
catch e
    e.message;
end
resultIPDevice
%% Retrieve Sensors Values
resultSensors(8) = 0;
try
    ts = double(m_SensorEx.ResultSensorsValues.TimeSpan);
    resultSensors = [ts/1000 double(m_SensorEx.ResultSensorsValues.Result)];
catch e
    e.message;
end
resultSensors
%% Retrieve Camera Image
try
    resultCamera = m_SensorEx.ResultCameraImage.Result;
    resultCameraSize = double(m_SensorEx.ResultCameraImage.Size);
    resultCameraFPS = [double(m_SensorEx.ResultCameraImage.FrameRate), ...
        double(m_SensorEx.ResultCameraImage.FrameRateAvg)];
    
    [R G B] = bytes2RGB(resultCamera, resultCameraSize);
    showRGB(R, G, B);
catch e
    e.message;
end
%% Repeat
%% Close Sensor Ex object when done
% m_SensorEx.StopConnection();
% clear all;