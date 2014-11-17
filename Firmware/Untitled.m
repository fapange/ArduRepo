%%
%DEPLOYMENT = 'Deployment_20';
DEPLOYMENT = 'CDnR_Message';
FLIGHT = 'HILSIM';
VEHICLE = 'R2_AP1';
bsize = '116408';
LAHEAD = 1;
WP4D   = 1;

%source = 'C:\Users\svazquez\AppData\Local\Temp\build6167467465953758867.tmp\ArduPlane.cpp.hex';
%source = 'C:\Users\svazquez\AppData\Local\Temp\1\build783117794192491456.tmp\ArduPlane.cpp.hex';
%source = 'C:\Users\svazquez\AppData\Local\Temp\1\build6747380112653398725.tmp\ArduPlane.cpp.hex';
%source = 'C:\Users\svazquez\AppData\Local\Temp\1\build29141823590098166.tmp\ArduPlane.cpp.hex';
%source = 'C:\Users\svazquez\AppData\Local\Temp\1\build1140307808257852056.tmp\ArduPlane.cpp.hex';
source = 'C:\Users\svazquez\AppData\Local\Temp\1\build8418534378581606187.tmp\ArduPlane.cpp.hex';

disp(source)
dest = 'g:\ArduRepo\ArdupilotMegaPlanner\Firmware';
cd (dest);
mkdir(DEPLOYMENT);
dest = strcat(dest,sprintf('\\%s\\APM-%s-%s',DEPLOYMENT,FLIGHT,VEHICLE));
%disp(dest)
if (LAHEAD == 1)
    dest = strcat(dest,'-wLookAhead');
%disp(dest)
end
if (WP4D == 1)
    dest = strcat(dest,'-w4DWpts');
%disp(dest)
end
dest = strcat(dest,'-',date);
dest = strcat(dest,'-',bsize);
dest = strcat(dest,'.hex');
disp(dest)
movefile(source,dest);
