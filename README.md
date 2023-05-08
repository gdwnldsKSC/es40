## DEC ES40 Simulator

es40 is free software. Please see the file COPYING for details.
For documentation, please see the files in the doc subdirectory.
For building and installation instructions please see the INSTALL file.

Windows build - VS2008 x64 target only currently.  
Requires npcap (upgrade from old legacy winpcap - last release 2013)  
Currently using latest npcap 1.75, with npcap-sdk-1.13.zip extracted to c:\program files\npcap\  
  
Formerly used poco c++ libraries, but they were copied into the source tree directly,    
perhaps we can update these at some point?  

libSDL 1.2 current master as of 5/8/2023 https://github.com/libsdl-org/SDL-1.2  

Older VS version support will be dropped from this branch as we move forward.  
This is initial build currentlyto re-create and reproduce the build environment  
currently using src\build_win64_visual_studio_2008\es40.sln  
gdwnldsKSC - 5/8/2023  As of this note, only the es40-cfg build is running