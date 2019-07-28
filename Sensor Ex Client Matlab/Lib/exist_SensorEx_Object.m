function result = exist_SensorEx_Object(u)

str = u;

if isnumeric(u(1))
  str = transp(char(u));   
end

result = 0;

try
    x = evalin('base', str);
    
    result = 1;
catch e
    e.message
end


end

