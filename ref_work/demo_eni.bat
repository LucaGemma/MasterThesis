@echo off

echo Start DRONE-1 and DRONE-2 autopilot
start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\scripts\demo_eni.py --connect udp:127.0.0.1:14551 --connect2 udp:127.0.0.1:14561