function blkStruct = slblocks

name = 'SensorEx_Lib';

Browser.Library = name;
Browser.Name    = 'Sensor Ex Library';
Browser.IsFlat  = 1; % Is this library "flat" (i.e. no subsystems)?

blkStruct.Name = ['Sensor Ex' sprintf('\n') 'Library'];
blkStruct.OpenFcn = name;
blkStruct.MaskDisplay = '';

blkStruct.Browser = Browser;
