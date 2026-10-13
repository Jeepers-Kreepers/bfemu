@echo off
rem ---------------------------------------------------------------------------
rem  Lead the emulator channels with a REAL radio in USB Joystick mode.
rem  Put the radio into USB Joystick mode first: EdgeTX/OpenTX -> MODEL SETUP
rem  -> USB Joystick, then plug the USB cable into the computer.
rem  No libraries needed: the radio is read with the built-in Windows way.
rem  Ctrl+C stops the script and the sticks go back to center.
rem ---------------------------------------------------------------------------
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py

echo Python:
%PY% -V
echo.
echo Step 1/3: devices and axes (5 seconds). Move each stick and see which AXIS it moves.
echo   If several joysticks are listed, pick yours with --index N (see the [N] numbers).
echo.
%PY% tools\tx.py axes %*
echo.
echo Step 2/4: if the sticks drive the WRONG channels, calibrate first.
echo   Symptoms: the YAW stick moves an AUX channel, the craft spins or tilts in
echo   "Preview" while the sticks are centered.
echo   The calibration asks you to move one stick at a time and finds the axes itself:
echo       %PY% tools\tx.py joystick --detect
echo   It saves the layout to tx_map.json, so it is done only once.
echo   Manual mapping, if you prefer to set it yourself:
echo       %PY% tools\tx.py joystick --map roll=0,pitch=1,throttle=2,yaw=3 --invert pitch
echo.
echo Step 3/4: switches and knobs (AUX channels). Flip EVERY switch up and down
echo   (a three-position one - through all three), turn the knobs to both ends.
echo   The tool finds their axes itself, tells a 2-position switch from a 3-position
echo   one and from a knob, and assigns AUX channels (first switch - AUX1, next - AUX2).
echo       %PY% tools\tx.py joystick --detect-aux
echo   If your switches do not move anything at all, they are not assigned to channels
echo   on the radio: MODEL SETUP - MIXES - assign each switch to CH5, CH6, ... - only
echo   assigned channels are sent over USB in Joystick mode.
echo   Manual layout, if you prefer: --aux-map 3=switch2,4=switch3:4,5=knob:7
echo.
echo Step 4/4: leading channels with the radio. Ctrl+C stops.
echo   Other windows of this tool are closed automatically (--kill-others): they would
echo   write the channels at the same time, which makes the sticks "jump" - the yaw
echo   stick moving an AUX channel and the craft spinning in "Preview".
echo   The tool prints the layout it uses, for example:
echo       roll - axis 0 (X); throttle - axis 2 (Z); ...
echo   Check it: the axis listed for THROTTLE must be the one your throttle stick moves.
echo   The throttle is not checked: it may rest at an END (stick down) or in the CENTER
echo   (ratchet) - both are normal.
echo   If the tool prints "[!]" about the layout, stop it and run: --detect
echo   IMPORTANT: run only ONE window of this tool. If an old window is still running,
echo   the new one will stop and say so - close the old window (Ctrl+C) and start again.
echo.
%PY% tools\tx.py joystick --kill-others %*
echo.
pause
