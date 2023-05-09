## DEC ES40 Simulator

es40 is free software. Please see the file COPYING for details.
For documentation, please see the files in the doc subdirectory.
For building and installation instructions please see the INSTALL file.

Windows build - VS2022 x64 target only currently.  
Requires npcap (upgrade from old legacy winpcap - last release 2013)  
Currently using latest npcap 1.75, with npcap-sdk-1.13.zip extracted to c:\program files\npcap\  
  
Formerly used poco c++ libraries, but they were copied into the source tree directly,    
perhaps we can update these at some point?  

libSDL 1.2 current master as of 5/8/2023 https://github.com/libsdl-org/SDL-1.2  
Extract contents of SDL-1.2-master folder into C:\Program Files\SDL\
SDL Built with VS 2022, using the included SDL_VS2010.sln project file.  
Follow instructions in VisualC.html - automatic project conversion for VS2013 works.
  
  
Direct SDK can be found here:  
https://www.microsoft.com/en-us/download/details.aspx?id=6812  
  
  
Install that and the SDL_config.h copy per documentation and should build without issue  
on latest, fully updated VS2022 17.5.5
  
Move compiled SDL.lib and SDLmain.lib to C:\Program Files\SDL\lib\ from target outdir  
SDL.dll will be required to be placed with the compiled es40 application  
  
Older VS version support will be dropped from this branch as we move forward.   
This is initial build currentlyto re-create and reproduce the build environment   
currently using src\VS2022\es40.sln    
  
Now we can build es40-cfg and es40! Yay! Don't forget to copy SDL.dll into the appropriate
release or debug output directory!
  
  
gdwnldsKSC - 5/9/2023 VS2022 build, works great.
