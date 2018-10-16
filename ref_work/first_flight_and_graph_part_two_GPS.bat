@echo off
echo Start autopilot script
echo Start GCS script to acquire data

TIMEOUT /T 1 /NOBREAK

start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\scripts\first_flight_GPS.py --connect udp:127.0.0.1:14551
REM start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\scripts\GCS_wLogGraph_and_map.py --connect1 udp:127.0.0.1:14552