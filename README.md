## DEC ES40 Simulator

es40 is free software. Please see the file COPYING for details.
For documentation, please see the files in the doc subdirectory.
For building and installation instructions please see the INSTALL file.

Windows build - VS2022 x64 target only currently.  
Requires npcap (upgrade from old legacy winpcap - which had its last release 2013)  
Currently using latest npcap 1.83, with npcap-sdk-1.15.zip extracted to c:\program files\npcap\  
  
Formerly used poco c++ libraries, but they were copied into the source tree directly,    
perhaps we can update these at some point?  

## Status

12/12/2025 - introduce config file variable tig.fw_write_enable; acts like 
hardware jumper to enable firmware write/flash update. J12/J13 on ES40 board,
when shorted ('true') here, enable flashrom update on real ES40 board. 
tig.fw_write_enable = true enables firmware write support.

# 8/27/25 Actual S3 BIOS WORKS! Still not complete, but it boots and executes SRM! 
Use S3Trio64 bios 86c764x1.bin

------------------------------------------------------------------------

## SDL Building Instructions

libSDL 1.2 current master as of 8/27/2025 https://github.com/libsdl-org/SDL-1.2  
Extract contents of SDL-1.2-master folder into C:\Program Files\SDL\
SDL Built with VS 2022, using the included SDL_VS2010.sln project file.  
Follow instructions in VisualC.html - automatic project conversion works for VS2022.

Essentially:
Go into SDL\include and copy SDL_config.h.default to SDL_config.h

Open C:\Program Files\SDL\VisualC in VS2022 - run VS as administrator since it is in
c:\program files

Accept the 'trust and continue' dialog if it is displayed

When the retarget projects dialog appears, select the desired windows SDK version
and upgrade the platform toolset to v143

Run Build solution

Move compiled SDL.lib and SDLmain.lib to C:\Program Files\SDL\lib\Debug X64 from 
target outdir. You may need to create this directory. For Release builds of SDL 
place in C:\Program Files\SDL\lib\Release X64

SDL.dll will be required to be placed with the compiled es40 application, for debug x64
build it would be placed similar to this location: es40\src\VS2022\x64\Debug

SDL.dll will be found in C:\Program Files\SDL\VisualC\SDL\x64\Debug for example
if you built x64 debug release configuration. 

------------------------------------------------------------------------

  
Direct SDK can be found here: (This may not be needed, attempt SDL build without first)
https://www.microsoft.com/en-us/download/details.aspx?id=6812  

Make sure to set the debug working directory in project settings as appropriate, such
as .\x64\Debug for debug build and .\x64\Release for release build.
  
Older VS version support will be dropped from this branch as we move forward.   
This is initial build currentlyto re-create and reproduce the build environment   
currently using src\VS2022\es40.sln    
  
Now we can build es40-cfg and es40! Yay! Don't forget to copy SDL.dll into the appropriate
release or debug output directory!
