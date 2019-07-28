function result = fetch_SensorEx_SensorsValues(u)

% eml.extrinsic('evalin')

tt = u(1);

chars = u(2:size(u,1));

str = transp(char(chars));

result(8) = 0;

try  
    net = evalin('base', str);
    
    ts = double(net.ResultSensorsValues.TimeSpan);
    
    result = tt+ts;
    
    result = [result ts];
    
    result = [result double(net.ResultSensorsValues.Result)];
catch e
    e.message;
end

