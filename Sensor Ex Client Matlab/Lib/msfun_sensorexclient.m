function msfun_sensorex(block)

m_DeviceNumber = 1;
m_imgSize = [640 480];

m_SensorEx = 0;
m_SensorsTimeTotal = 0;

setup(block);

%% ---------------------------------------------------
    function setup(block)
        
        block.NumDialogPrms  = 2;
        block.DialogPrmsTunable = {'Nontunable','Nontunable'};
        
        block.NumInputPorts  = 0;
        block.NumOutputPorts = 7;
        
        for i = 1:2
            block.OutputPort(i).SamplingMode = 'Sample';
            block.OutputPort(i).Dimensions = 4;
            block.OutputPort(i).DatatypeID = 3;
        end
        
        block.OutputPort(2).Dimensions       = 4;
        
        block.OutputPort(3).SamplingMode = 'Sample';
        block.OutputPort(3).Dimensions = 8;
        
        m_imgSize  = block.DialogPrm(2).Data;
        
        for i = 4:6
            block.OutputPort(i).SamplingMode = 'Sample';
            block.OutputPort(i).Dimensions = m_imgSize;
            block.OutputPort(i).DatatypeID = 3;
        end
        
        block.OutputPort(7).SamplingMode = 'Sample';
        block.OutputPort(7).Dimensions = 2;
        
        block.SampleTimes = [-1 0];
        
        block.RegBlockMethod('CheckParameters',     @CheckPrms);
        block.RegBlockMethod('PostPropagationSetup',@DoPostPropSetup);
        block.RegBlockMethod('InitializeConditions',@InitConditions);
        block.RegBlockMethod('Outputs',             @Output);
        block.RegBlockMethod('Terminate',           @Terminate);
    end

%%
    function CheckPrms(block)
        dn = block.DialogPrm(1).Data;
        
        if (dn == 0 || dn > 5)
            error('Invalid parameter. 0 and bigger than 5 are not allowed as parameter values.');
        end
        
        dn = block.DialogPrm(2).Data;
        dnS = size(dn,2);
        
        if (dnS ~= 2)
            error('Invalid parameter. Parameter must be of the form [ImageHeight ImageWidth].');
        end
        
    end

%%
    function DoPostPropSetup(block)
        nbDW = 2;
        block.NumDworks = nbDW;
        
        block.Dwork(1).Name            = 'x0';
        block.Dwork(1).Dimensions      = 1;
        
        block.Dwork(2).Name            = 'x1';
        block.Dwork(2).Dimensions      = 2;
        
        for i = 1:nbDW
            block.Dwork(i).DatatypeID      = 0;
            block.Dwork(i).Complexity      = 'Real';
            block.Dwork(i).UsedAsDiscState = true;
        end
        
        
        
    end

%%
    function InitConditions(block)
        m_DeviceNumber  = block.DialogPrm(1).Data;
        block.Dwork(1).Data = m_DeviceNumber;
        
        block.Dwork(2).Data = m_imgSize;
        
        m_SensorEx = create_SensorEx(m_DeviceNumber);
    end

%%
    function Output(block)
        resultIPLocal(4) = uint8(0);
        resultIPDevice(4) = uint8(0);
        
        resultSensors(8) = 0;
        
        R(m_imgSize(1),m_imgSize(2)) = uint8(0);
        G(m_imgSize(1),m_imgSize(2)) = uint8(0);
        B(m_imgSize(1),m_imgSize(2)) = uint8(0);
        
        resultCameraFPS(2) = 0;
        
        try
            resultIPLocal = uint8(m_SensorEx.ResultLocalIPAddress.IPAddress);
        catch e
            e.message;
        end
        
        try
            resultIPDevice = uint8(m_SensorEx.ResultDeviceIPAddress.IPAddress);
        catch e
            e.message;
        end
        
        try
            ts = double(m_SensorEx.ResultSensorsValues.TimeSpan);
            
            m_SensorsTimeTotal = m_SensorsTimeTotal+ts;
            
            resultSensors = m_SensorsTimeTotal/1000;
            resultSensors = [resultSensors ts/1000];
            resultSensors = [resultSensors double(m_SensorEx.ResultSensorsValues.Result)];
        catch e
            e.message;
        end
        if (size(resultSensors,2) ~= 8)
            resultSensors(8) = 0;
        end
        
        try
            resultCamera = m_SensorEx.ResultCameraImage.Result;
            resultCameraSize = double(m_SensorEx.ResultCameraImage.Size);
            resultCameraFPS = [double(m_SensorEx.ResultCameraImage.FrameRate), ...
                double(m_SensorEx.ResultCameraImage.FrameRateAvg)];

            [R G B] = bytes2RGB(resultCamera, resultCameraSize);
        catch e
            e.message;
        end
        
        block.OutputPort(1).Data = resultIPLocal;
        block.OutputPort(2).Data = resultIPDevice;
        
        block.OutputPort(3).Data = resultSensors;
        
        block.OutputPort(4).Data = R;
        block.OutputPort(5).Data = G;
        block.OutputPort(6).Data = B;
        
        block.OutputPort(7).Data = resultCameraFPS;
    end

%%
    function Terminate(block)
        m_SensorEx.StopConnection;
    end
end