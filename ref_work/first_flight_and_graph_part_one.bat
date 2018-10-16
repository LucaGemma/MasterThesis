@echo off
echo Connecting to drone and sharing output into 3 ports
TIMEOUT /T 1 /NOBREAK

start "" ""%WINPYDIR%\python.exe"" %userprofile%\Downloads\WinPython-64bit-2.7.13.1Zero\python-2.7.13.amd64\Lib\site-packages\MAVProxy\mavproxy.py --master udp:192.168.8.72:14560 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14552

echo now run first_flight_and_graph_part_two.bat