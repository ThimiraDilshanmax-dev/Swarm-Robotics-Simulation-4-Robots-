@echo off
cd /d "%~dp0"
python run_simulation.py --scenario stationary
if errorlevel 1 pause

