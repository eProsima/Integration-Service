@echo off
:: Get install\bin directory
set np=%~dp0\bin
:: Add to system path avoiding duplicates
echo %path%|find /i "%np:"=%">nul  || set path=%np%;%path%
:: Run generic local_setup
%~dp0\local_setup.bat