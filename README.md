## DEC ES40 Simulator  
  
es40 is free software. Please see the file COPYING for details.  
For documentation, please see the files in the doc subdirectory.  
For building and installation instructions please see below.  
  
Windows build - VS2026 x64 target is the main development environment.  
Requires npcap (old legacy winpcap saw its last release in 2013)    
  
Formerly used poco c++ libraries, but they were copied into the source tree  
directly,  perhaps we can update these at some point?  

Latest VC redist may be required to run binaries, available here:
https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist  
Latest 2017-2026 x64 redistributable is recommended. At minimum the version  
matching your build toolchain is needed. Binary release builds provided here   
are done with the v145 / VS2026 toolchain. Matching version is available in  
VS build directory.  

# Live shot of X11 running on emulated S3 via SDL display! 

![Screenshot](https://github.com/gdwnldsKSC/es40/blob/main/screenshots/OpenVMS.png?raw=true)

------------------------------------------------------------------------

## Status  

### 2/28/26 - S3 text rendering resolved. Keyboard input buffer resolved. Tru64  
OpenVMS X11 display is almost flawless! Mouse support has been added and works  
great! Tru64 X11 now displays from the install CD of 5.1B, however install sill  
fails on SCSI disk. To be resolved. Keyboard issues have been resolved, no more  
buffer full while using VGA console.  
  
### 2/20/26 - S3 Update, Cirrus exlcuded. SDL3 conversion
X11 has basic display support under OpenVMS now, some text corruption issues  
exist that are being worked on. Mouse input support is currently non-present.  
Boot to login and CDE desktop works.
Cirrus is being excluded from builds now, as it is just a generic VGA card  
implementation and after render path reworks will need some tending to for  
conversion.  
Upgraded to SDL3. Brings lots of new goodies. Gets rid of archaic SDL1.2.  
Incrimented version to 0.5 due to major changes across the board such as above.  
  
### 2/14/26 - S3 Graphics port from MAME, work towards ARC support
Working through the S3 implementation, a more feature-complete and functional  
implementation is found in MAME, though we already implemented some functions  
that MAME does not yet. So we've refactored a lot of the code which, while not  
yet complete, has already brought along some interesting improvements, such  
as better text-mode graphics and somewhat more concise code. Contribution back  
to MAME upstream for this code will be forthcoming for the items we implemented  
prior that MAME does not have.
Large comments and functions that documented VGA behavior and standards  
information have been moved to 'old_vga_docs.txt' to clean up so that our code  
more closely tracks MAME's S3 implementation to better enable simple diff'ing  
and code sharing as progress is made.  
  
Some basic work has been done in general as well in the direction of getting  
ARC firmware working.  
  
### 12/24/25 - Build housekeeping  
All configurations now build cleanly. Default location for include and lib  
files is now c:\dev\SDL3\ and c:\dev\npcap-sdk\ - you can change this by doing  
a simple find/replace in the .vcxproj files if you want to use different paths.  
Simple POKEQ/POKEL support added to IDB as well.  
NN = No Network  
NS = No Screen (VGA Console)  
LSM/LSS = Lockstep Master/Slave builds.  
IDB = Integrated Debugger build.  
  
### 12/19/25 - Somewhat working full FLASH ROM support.  
After doing the initial bootstrap with cl67srmrom.exe, you can now use the  
HP firmware update CD to flash the ES40 firmware, including ARC/Abios, which  
doesn't work yet, but you can as of this point at least downgrade via the   
HP firmware update CD to SRM V7.2-1, which does work, and then boot off of  
the flash rom directly on that version.  
  
### 8/27/25 Actual S3 VBIOS WORKS! 
S3 Incomplete, but it boots and executes SRM!    
Use S3Trio64 bios 86c764x1.bin   
  
------------------------------------------------------------------------  
  
## Building Instructions  
  
We'll need both npcap and SDL-3.4 for full featured es40 builds. 
  
If you build only NS target configurations, then you do not need SDL.  
If you build only NN target configurations, then you do not need npcap.  
If you build NS NN target configurations, then you do not need either.  
  
### SDL:  
  
Retrieve SDL from https://github.com/libsdl-org/SDL  

For simplicity, I had extracted and configured include and link directories  
for all es40 targets/configurations to look for SDL under C:\dev\SDL\ - a simple   
find and replace in the .vcxproj files can change this if you want - eg find  
"C:\dev\SDL\" and replace with your desired SDL location.  
  
Extract the root of the repository to C:\dev\SDL\  
  
Under C:\dev\SDL3\VisualC\ there is a SDL.sln file - open this in VS2022/26.  
  
Accept the 'trust and continue' dialog if it is displayed  
  
If prompted to retarget the solution, select the desired Windows SDK and  
toolchain version  
  
Build the solution for x64 Debug and Release configurations.  
  
### npcap:  
  
If you wish to build only NN target configurations, you can skip this step.  
  
If you wish to build all configurations, but not run, you only need to extract  
the npcap SDK to C:\dev\npcap-sdk\  
  
To run network enabled binaries, you will need to install npcap itself.  
The installer and SDK for npcap can be found here: https://npcap.com/#download  
  
### es40:  
  
After your pathing is fixed in the vcxproj files or you used the default  
c:\dev\ locations and structure, you should be able to open the es40.sln file  
  
Run Build solution for individual configurations, or batch build to build  
all configurations.

SDL3.dll will be required to be placed with the compiled es40 application, for  
debug x64 build it would be placed in this location: src\VS2022\x64\Debug  
or wherever you copy es40.exe to.  

SDL3.dll will be found in C:\dev\SDL3\VisualC\x64\Debug for example  
if you built x64 debug release configuration.  

Make sure to set the debug working directory in project settings as  
`$(OutDir)` for this configuration, the command being `$(TargetPath)` as is set  
by default is fine, however.  

Resulting binaries will be in x64\Debug, x64\Release, x64\Release IDB, etc,  
with the binary names being similar to es40 Release NS NN.exe.  

------------------------------------------------------------------------

### Old notes, to be reviewed. 
  
Direct SDK can be found here: (This may not be needed, attempt SDL build  
without first) https://www.microsoft.com/en-us/download/details.aspx?id=6812  
