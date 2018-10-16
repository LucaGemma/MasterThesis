@echo off
echo Start DRONE-1 simulator

start "" ""dronekit-sitl copter --instance 1 --home=46.097995,11.098938,0,180""
start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\python-2.7.13.amd64\Lib\site-packages\MAVProxy\mavproxy.py --master tcp:127.0.0.1:5770 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14550

TIMEOUT /T 1 /NOBREAK

echo Start DRONE-2 simulator

start "" ""dronekit-sitl copter --instance 2 --home=46.097400,11.099646,0,180""
start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\python-2.7.13.amd64\Lib\site-packages\MAVProxy\mavproxy.py --master tcp:127.0.0.1:5780 --out udp:127.0.0.1:14561 --out udp:127.0.0.1:14560

echo now run demo_eni.bat

REM run this script within WinPython Command Prompt typing %userprofile%\path-to-batch\SITL.bat
REM %userprofile% = C:\Users\mauri\ in my case
REM %WINPYDIR% points to WinPython-32bit-2.7.13.1Zero\python-2.7.13\