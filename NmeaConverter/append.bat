@echo off
echo My First Report_%date:~-4,4%%date:~-7,2%%date:~-10,2%.pdf

set "header=c:\SomeFolder\Headings.txt"
set "folder=."
set "tempFile=%folder%\nmea.txt"
for %%F in ("%folder%\*.NMEA") do (
  rem type "%header%" >"%tempFile%"
  echo %%F
  type "%%F" >>"%tempFile%"
  rem move /y "%tempFile%" "%%F" >nul
)

pause