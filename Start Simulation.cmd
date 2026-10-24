@echo off
cd /d "%~dp0"
python run_simulation.py
if errorlevel 1 pause
