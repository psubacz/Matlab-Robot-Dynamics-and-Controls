function [az,el]=normalToAzimuthElevationDEG(x,y,z)

    if length(x)>1
        v         = x;
        x=v(1);
        y=v(2);
        z=v(3);
    end

    if x==0 && y==0
        x =eps;
    end

    vNorm = sqrt(x^2+y^2+z^2);
    x=x/vNorm;
    y=y/vNorm;
    z=z/vNorm;

    az = 180/pi*asin(x/sqrt(x^2+y^2));
    el = 180/pi*asin(z);

    end
