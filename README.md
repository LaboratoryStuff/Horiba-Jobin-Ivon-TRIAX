# Horiba-Jobin-Ivon-TRIAX
Horiba Jobin Ivon TRIAX series Monochromators

The code and mechanical parts presented here were tested with a Horiba Jobin Ivon TRIAX 320 monochromator.
They should work with the other models of the series with minimum or no alterations.

-------------
Matlab Folder

Two files are provided: a class to control the TRIAX monochromators and a GUI.
Both were tested with a TRIAX 320 connected by serial port.
The monochromator configuration (diffraction gratings, ...) should be updated.


Known bugs

The monochromator motors have to be initialized two times to bring turn grating tower from positon #2 to position #0.

-----------
Adapter TRIAX port to Thorlabs SM1 tube
