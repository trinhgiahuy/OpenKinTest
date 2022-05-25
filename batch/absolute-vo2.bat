@echo off
REM SETLOCAL
REM (
for /f "usebackq tokens=1 delims=, skip=8" %%i in (D:\a.csv) do (
::setlocal enabledelayedexpansion
call :PROCESS %%i
)
REM )
pause
::endlocal

::goto :PROCESS
:PROCESS

::GET TIME INFORMATION
set clk_hour=%time:~0,2%
set clk_mins=%time:~3,2%
set clk_secs=%time:~6,2%

REM echo 1 %1 
REM echo 1~ %~1

call :strlen result "%1"
::echo "%1%" 
::echo %1
::echo %result%

setlocal enabledelayedexpansion
set tmp_data=%~1
echo before loop 3 !tmp_data!
echo before loop 6 !tmp_data!
::result=3: NO HOUR (31:30)
::result=6: HAVE HOUR (1:25:45)
if %result%==3 (
echo after loop 3 !tmp_data!
REM call :ECHO3 %tmp_data%
::echo %result% loop result = 3
set /A tmp_mins=1!tmp_data:~0,2! - 100
set /A tmp_secs=1!tmp_data:~3,2! - 100
::echo !tmp_mins! !tmp_secs! 
set new_hours=!clk_hour!
REM if !tmp_mins!==08 (
REM echo 08
REM set tmp_mins=8
REM )

REM if !tmp_mins!==09 (
REM echo 09
REM set tmp_mins=9
REM )
set /A new_mins=!clk_mins! + !tmp_mins!
set /A new_secs=!clk_secs! + !tmp_secs!

echo new3 !new_hours! !new_mins! !new_secs! 
)

if %result%==6 (
echo after loop 6 !tmp_data! 
::echo %result% loop result = 6
set tmp_hours=!tmp_data:~0,1!
set tmp_mins=1!tmp_data:~2,2! - 100
set tmp_secs=!tmp_data:~5,2!

set /A new_hours=!clk_hour! + !tmp_hour!
set /A new_mins=!clk_mins! + !tmp_mins!
set /A new_secs=!clk_secs! + !tmp_secs!

echo new6 !tmp_hours! !tmp_mins! !tmp_secs!
)

::Check if secs and hours are larger than 60
if !new_secs! geq 60 (
echo ---------set60
echo ---beforeadd !new_mins! !new_secs!
set /A new_mins=!new_mins!+1
set /A new_secs=!new_secs!-60
echo ---afteradd !new_mins! !new_secs!
)

REM :ECHO3
REM ::echo callecho3 %1
REM set _data_=%1
REM set tmp_mins=%_data:~0,2%
REM set tmp_secs=%_data:~3,2%
REM echo echo3 %tmp_mins% %tmp_secs%

REM :ECHO6
REM echo 6 %tmp_hours% %tmp_mins% %tmp_secs%

::set tmp_hour=%tmp:
::echo %clk_hour%
::echo %clk_mins%
::echo %clk_secs%
 
REM set tmp_hours=%tmp%

REM set /a new_secs=%clk_secs% + 
REM set /a new_hours=%clk_hour% + 

REM FUNCTION TO GET THE STRING LENGTH
:strlen <resultVar> <stringVar>
(   
    setlocal EnableDelayedExpansion
	(set^ tmp=!%~2! )
    if defined tmp (
        set "len=1"
        for %%P in (4096 2048 1024 512 256 128 64 32 16 8 4 2 1) do (
            if "!tmp:~%%P,1!" NEQ "" ( 
                set /a "len+=%%P"
                set "tmp=!tmp:~%%P!"
            )
        )
    ) ELSE (
        set len=0
    )
)
( 
    endlocal
    set "%~1=%len%"
    exit /b
)

