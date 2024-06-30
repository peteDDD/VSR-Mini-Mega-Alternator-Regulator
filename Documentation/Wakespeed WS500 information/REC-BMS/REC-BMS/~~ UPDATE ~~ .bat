@echo off
REM *****************************************************
REM *
REM *
REM *	Simple batch file to update firmware and then load in 
REM *	a configuration file into WS500 Alternator Regulators
REM *
REM *	(c) Copyright 2020, TJC Micro
REM *   All Rights Reserved
REM *
REM *
REM *	To use, either define the config/firmware file below
REM *	and those will be used.   
REM *
REM *	As an alternative, do NOT define the two veriables below,
REM *   and place a .txt and .dfu file in the same subdirectory and
REM * 	those files will be transfered to the WS500
REM *
REM *
REM *  Ver 2.0.0  --> Added auto scan for config and firmware files
REM *
REM *****************************************************
SETLOCAL ENABLEDELAYEDEXPANSION




REM Set these two varables to the two files you want to use...
REM
SET firmware="WS500Firmware.dfu"
SET configure="config.txt"






REM *****************************************************
REM See if the two config/DFU files exist, if not scan for ones that might be able to be used
REM

IF NOT EXIST "!firmware!" (
  FOR %%F IN (*.dfu) DO (
    SET firmware=%%F
  )
)


IF NOT EXIST "!configure!" (
  FOR %%F IN (*.txt) DO (
    SET configure=%%F
  )
)



IF NOT EXIST "!configure!" (
  IF NOT EXIST "!firmware!" (
    ECHO.
    ECHO No Firmware or Configuration file found.
    ECHO Please copy one or both of a .TXT configuration file
    ECHO and/or a .DFU Firmware file into this subdurectory 
    ECHO and re-run the  UPDATE.BAT file
    ECHO.
    ECHO Press any key to exit
    PAUSE > NUL
    EXIT
  )
)




REM *****************************************************
REM   Tell User what is about to happen
REM 

ECHO.
ECHO.
ECHO This simple batch file will upgrade your Wakespeed Advanced Alternator Regualtor.
ECHO You will need to remove the top cover of the regulator and connect this 
ECHO computer via a cable to the USB port on the regulator's board.
ECHO.
ECHO.
IF EXIST "!firmware!" (
  ECHO The Regulator will be updated to firmware version:  "!firmware!"
)
IF EXIST "!configure!" (
  ECHO The configuration file "!configure!" will be loaded into the regulator
)
ECHO.
ECHO.

ECHO Connect a USB cable between the regulator and this computer.
ECHO (It is best of the regualtor is the ONLY device connected to
ECHO  this computer.  If you receive errros, try removing any other
ECHO  device plugged into a USB port and restarting this update batch file)
ECHO.
ECHO.
ECHO When the USB cable is attached, 


REM *****************************************************
REM   Update Firmware
REM 

IF EXIST "!firmware!" (
    ECHO Press-and-hold the RESET button on the regulator for 5 seconds to allow firmware upgrade.
    ECHO Then press any key to continue . . .

    PAUSE > NUL

    ECHO.
    ECHO Looking for a regulator to update
    ECHO.
    ECHO.


    dfusecommand -c -d --v --fn "!firmware!"

    ECHO.
    ECHO.
    ECHO.
    ECHO Finished.
    ECHO.
    ECHO.
    ECHO Please verify upgrade completed without errors.  
    ECHO.

    IF EXIST "!configure!" (
      ECHO Now the configuration will be loaded into the regulator.
      ECHO Press-and-release the RESET button on the regulator to return to normal mode.
      ECHO Count to 5 and then
    ) ELSE (
      ECHO Press-and-release the RESET button on the regulator to return to normal mode.
    )

)






REM *****************************************************
REM   Send configuration file
REM 

IF EXIST "!configure!" (
    PAUSE
    ECHO.
    ECHO.
    ECHO Loading configuraton


    SET comPort=COM1
    if ["%~1"]==[""] (
        REM  User did not (optional) specify which COM port is to be used.
        ECHO.
        ECHO Scanning USB ports, looking for the regulator.
 
        REM **********************************************************
        REM *
        REM *	Scan the COM ports, looking for a list of active ports.
        REM *	When one is found just try calling WS500_transfer and see if
        REM *	it works.
        REM *
        REM **********************************************************
        for /f "tokens=4" %%A in ('mode^|findstr "COM[0-9]*:"') do (

            :: Looks like we may have found one.
            ECHO  -- Found something attached to port %%A, checking to see if it is a regulator.
            SET comPort=%%A

            :: Does string have a trailing `:` char? If so remove it 
            IF !comPort:~-1!==: SET comPort=!comPort:~0,-1!

        WS500_transfer -P!comPort!  -f"!configure!" -v -R
        IF ERRORLEVEL 1 (
	  ECHO.
	  ECHO.
    	  ECHO  -- Does not look like that was a regulator!
	  ECHO.
        )
	ECHO.
	ECHO.
        )


    ) else (
        REM  User defined COM port to use
        ECHO.
        ECHO Com port %1 specified, will upload to that port
        
        SET comPort=%1
        
        :: Does string have a trailing `:` char? If so remove it 
        IF !comPort:~-1!==: SET comPort=!comPort:~0,-1!
        
        WS500_transfer -P!comPort!  -f"!configure!"  -v -R
        ECHO.
    )
  ECHO.
  ECHO Finished.
  ECHO.
  ECHO Verify the updates completed without errors and once done 
)




PAUSE
ECHO.










