cd "C:\Users\svazquez\Documents\GitHub\ArduRepo\ArduPlane" 
set /p="#define SLV_GIT_CODE " <nul > git_hash.h 
"C:\Program Files (x86)\Git\bin\git.exe" rev-parse HEAD >> "git_hash.h"




