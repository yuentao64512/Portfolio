@echo off
rem RoboCommand bridge for team SGIT - double-click to start.
rem Type "task 1" ... "task 4" / "task none" in the window to set current_task.

rem RoboCommand's broker address - from the organisers on competition day.
rem 127.0.0.1 = RoboNation's test server running on this laptop (docker compose up).
rem 192.168.1.103 = the organisers' laptop (testing) - change to the Raspberry Pi's IP when they issue it.
set BROKER=192.168.1.103

cd /d "%~dp0"
python -u robocommand_bridge.py --broker %BROKER%
pause
