function m_SensorEx = create_SensorEx(deviceNumber)

domain = System.AppDomain.CurrentDomain;
assemblies = domain.GetAssemblies;
flag = 0;

for i= 1:assemblies.Length
    
    asm = assemblies.Get(i-1);
    
    fin = strfind(char(asm.FullName), 'Sensor Ex Client Basic');
    if size(fin,1) > 0
        flag = 1;
    end
    
end


if flag
    %     fprintf('Assembly already added\n');
else
    
    [pathstr, name, ext] = fileparts(mfilename('fullpath'));
    str = [pathstr '\DLLs\Sensor Ex Client Basic.dll'];
    
    try
        NET.addAssembly(str);
    catch e
        e.message
        if(isa(e,'NET.NetException'))
            e.ExceptionObject
        end
    end
    
end

m_SensorEx = SensorEx_Client_Basic.SensorExBasic;
m_SensorEx.SetDeviceNumber(deviceNumber);
m_SensorEx.StartConnection;

% addlistener(handles.h_SensorExBasic,'SensorsValues_Received',@SensorsValues_Received);
% addlistener(handles.h_SensorExBasic,'File_Received',@File_Received);
% addlistener(handles.h_SensorExBasic,'GraphInfos_Received',@GraphInfos_Received);
% addlistener(handles.h_SensorExBasic,'CameraImage_Received',@CameraImage_Received);
% addlistener(handles.h_SensorExBasic,'Connection_Changed',@Connection_Changed);
% addlistener(handles.h_SensorExBasic,'IPClient_Changed',@IPClient_Changed);
% addlistener(handles.h_SensorExBasic,'IPLocal_Changed',@IPLocal_Changed);

end

