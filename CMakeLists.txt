cmake_minimum_required(VERSION 4.1)

project(ES40 LANGUAGES C CXX)

option(ES40_DISABLE_SDL "Disable SDL for headless builds" OFF)
option(ES40_DISABLE_NETWORKING "Disable PCap/networking for networkless builds" OFF)
option(ES40_DISABLE_LSS_LSM "Disable LSS/LSM (lockstep) builds." OFF)
option(ES40_DISABLE_IDB "Disable IDB (built-in debugger) builds." OFF)
#option(ES40_DISABLE_ASMJIT "Disable ASMJIT" OFF)

# Only used for PCap/NPCap at the moment. 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/extras/cmake")

# set(ASMJIT_NO_INSTALL ON)
# add_subdirectory(third_party/asmjit)

if (NOT ES40_DISABLE_SDL)
    if (NOT SDL3_DIR)
        set(SDL3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDL3/cmake")
    endif()
    find_package(SDL3 REQUIRED)
endif()

if (NOT ES40_DISABLE_NETWORKING)
    if (WIN32)
        #if (NOT NPCAP_DIR)
        #    set(NPCAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../NPCAP")
        #endif()
        #find_package(NPCap REQUIRED)
    else()
        find_package(PCap REQUIRED)
    endif()
endif()

if (NOT WIN32)
    include(CheckSymbolExists)
    include(CheckIncludeFile)
    include(CheckLibraryExists)
    include(CheckTypeSize)
    include(CheckIncludeFiles)

    check_symbol_exists(alarm "unistd.h" HAVE_ALARM)
    check_include_file("arpa/inet.h" HAVE_ARPA_INET_H)
    check_include_file("arpa/telnet.h" HAVE_ARPA_TELNET_H)
    check_symbol_exists(atexit "stdlib.h" HAVE_ATEXIT)
    check_include_file("ctype.h" HAVE_CTYPE_H)
    set(HAVE_CXX17 1)
    check_include_file("errno.h" HAVE_ERRNO_H)
    check_include_file("fcntl.h" HAVE_FCNTL_H)
    check_symbol_exists(fopen "stdio.h" HAVE_FOPEN)
    check_symbol_exists(fopen64 "stdio.h" HAVE_FOPEN64)
    check_symbol_exists(fork "unistd.h" HAVE_FORK)
    check_symbol_exists(fseek "stdio.h" HAVE_FSEEK)
    check_symbol_exists(fseeko "stdio.h" HAVE_FSEEKO)
    check_symbol_exists(fseeko64 "stdio.h" HAVE_FSEEKO64)
    check_symbol_exists(ftell "stdio.h" HAVE_FTELL)
    check_symbol_exists(ftello "stdio.h" HAVE_FTELLO)
    check_symbol_exists(ftello64 "stdio.h" HAVE_FTELLO64)
    check_symbol_exists(gmtime_s "time.h" HAVE_GMTIME_S)
    check_symbol_exists(inet_aton "arpa/inet.h" HAVE_INET_ATON)
    check_include_file("inet.h" HAVE_INET_H)
    check_include_file("inttypes.h" HAVE_INTTYPES_H)
    check_include_file("in.h" HAVE_IN_H)
    check_symbol_exists(isblank "ctype.h" HAVE_ISBLANK)
    check_symbol_exists(localtime_s "time.h" HAVE_LOCALTIME_S)
    check_symbol_exists(malloc "stdlib.h" HAVE_MALLOC)
    check_include_file("malloc.h" HAVE_MALLOC_H)
    check_include_file("memory.h" HAVE_MEMORY_H)
    check_symbol_exists(memset "string.h" HAVE_MEMSET)
    check_include_file("netinet/in.h" HAVE_NETINET_IN_H)
    check_symbol_exists(pow "math.h" HAVE_POW)
    check_include_file("process.h" HAVE_PROCESS_H)
    check_library_exists(pthread pthread_self "" HAVE_PTHREAD)
    check_include_file("pthread.h" HAVE_PTHREAD_H)
    check_symbol_exists(PTHREAD_PRIO_INHERIT "pthread.h" HAVE_PTHREAD_PRIO_INHERIT)
    check_symbol_exists(realloc "stdlib.h" HAVE_REALLOC)
    if (NOT ES40_DISABLE_SDL)
        set(HAVE_SDL 1)
    endif()
    check_symbol_exists(select "sys/select.h" HAVE_SELECT)
    check_include_file("signal.h" HAVE_SIGNAL_H)
    check_symbol_exists(socket "sys/socket.h" HAVE_SOCKET)
    check_include_file("socket.h" HAVE_SOCKET_H)
    check_symbol_exists(sqrt "math.h" HAVE_SQRT)
    check_include_file("stdbool.h" HAVE_STDBOOL_H)
    check_include_file("stdint.h" HAVE_STDINT_H)
    check_include_file("stdio.h" HAVE_STDIO_H)
    check_include_file("stdlib.h" HAVE_STDLIB_H)
    check_symbol_exists(strcasecmp "string.h" HAVE_STRCASECMP)
    check_symbol_exists(strchr "string.h" HAVE_STRCHR)
    check_symbol_exists(strdup "string.h" HAVE_STRDUP)
    check_include_file("stddef.h" HAVE_STDDEF_H)
    check_include_file("strings.h" HAVE_STRINGS_H)
    check_include_file("string.h" HAVE_STRING_H)
    check_symbol_exists(strncasecmp "string.h" HAVE_STRNCASECMP)
    check_symbol_exists(strspn "string.h" HAVE_STRSPN)
    check_include_file("sys/param.h" HAVE_SYS_PARAM_H)
    check_include_file("sys/select.h" HAVE_SYS_SELECT_H)
    check_include_file("sys/socket.h" HAVE_SYS_SOCKET_H)
    check_include_file("sys/stat.h" HAVE_SYS_STAT_H)
    check_include_file("sys/time.h" HAVE_SYS_TIME_H)
    check_include_file("sys/types.h" HAVE_SYS_TYPES_H)
    check_include_file("sys/wait.h" HAVE_SYS_WAIT_H)
    check_include_file("unistd.h" HAVE_UNISTD_H)
    check_symbol_exists(vfork "unistd.h" HAVE_VFORK)
    check_include_file("vfork.h" HAVE_VFORK_H)
    set(HAVE_WORKING_FORK ${HAVE_FORK})
    set(HAVE_WORKING_VFORK ${HAVE_VFORK})
    check_type_size(_Bool _BOOL)
    check_symbol_exists(_fseeki64 "stdio.h" HAVE__FSEEKI64)
    check_symbol_exists(_ftelli64 "stdio.h" HAVE__FTELLI64)
    check_symbol_exists(_strdup "string.h" HAVE__STRDUP)
    check_symbol_exists(_stricasecmp "string.h" HAVE__STRICASECMP)
    check_symbol_exists(_stricmp "string.h" HAVE__STRICMP)
    check_include_file("X11/X.h" HAVE_X11)

    set(PACKAGE \"${PROJECT_NAME}\")
    set(PACKAGE_BUGREPORT "\"https://github.com/gdwnldsKSC/es40/issues/new\"")
    string(TOLOWER ${PROJECT_NAME} PACKAGE_NAME_L)
    set(PACKAGE_NAME \"${PACKAGE_NAME_L}\")
    set(PROJECT_VERSION 0.65)
    set(PACKAGE_VERSION \"${PROJECT_VERSION}\")
    set(PACKAGE_STRING "\"${PACKAGE_NAME_L} ${PROJECT_VERSION}\"")
    set(PACKAGE_TARNAME \"${PACKAGE_NAME_L}.tar.gz\")
    set(PACKAGE_URL "\"https://github.com/gdwnldsKSC/es40/issues\"")
    set(VERSION \"${PROJECT_VERSION}\")

    set(RETSIGTYPE "void")
    set(SELECT_TYPE_ARG1 "int")
    set(SELECT_TYPE_ARG234 "(fd_set *)")
    set(SELECT_TYPE_ARG5 "(struct timeval *)")
    set(STDC_HEADERS 1)
    check_include_files("time.h;sys/time.h" TIME_WITH_SYS_TIME)
    set(HAVE_SYS_TIME ${TM_IN_SYS_TIME})

    configure_file(src/config.h.cmake src/config.h)
    add_compile_definitions(HAVE_CONFIG_H)
endif()

############################# ES40-CFG #############################

set(ES40_CFG_SOURCES)
set(ES40_CFG_LIBRARIES)
set(ES40_CFG_DEFINITIONS)

list(APPEND ES40_CFG_SOURCES
    src/es40-cfg.cpp
    src/base/Exception.cpp
    )

if (WIN32)
    list(APPEND ES40_CFG_LIBRARIES winmm)
endif()

if (NOT ES40_DISABLE_NETWORKING)
    list(APPEND ES40_CFG_DEFINITIONS HAVE_PCAP)
    if (WIN32)
        #list(APPEND ES40_CFG_LIBRARIES NPCap::NPCap)
    else()
        list(APPEND ES40_CFG_LIBRARIES PCap::PCap)
    endif()
endif()

if (NOT ES40_DISABLE_SDL)
    list(APPEND ES40_CFG_DEFINITIONS HAVE_SDL)
endif()

add_executable(es40-cfg 
    ${ES40_CFG_SOURCES}
    )

target_include_directories(es40-cfg PUBLIC 
    src/
    )

target_link_libraries(es40-cfg PRIVATE 
    ${ES40_CFG_LIBRARIES}
    )

target_compile_definitions(es40-cfg PUBLIC ${ES40_CFG_DEFINITIONS})

set_property(TARGET es40-cfg PROPERTY CXX_STANDARD 17)

############################### ES40 ###############################

set(ES40_SOURCES)
set(ES40_INCLUDES)
set(ES40_LIBRARIES)
set(ES40_DEFINITIONS)

list(APPEND ES40_SOURCES
    src/base/Exception.cpp
    src/base/ThreadTLS.cpp
    src/gui/keymap.cpp
    src/gui/scancodes.cpp
    src/AliM1543C_ide.cpp
    src/AliM1543C_pmu.cpp
    src/AliM1543C_usb.cpp
    src/AliM1543C.cpp
    src/AlphaCPU_ieeefloat.cpp
    src/AlphaCPU_vaxfloat.cpp
    src/AlphaCPU_vmspal.cpp
    src/AlphaCPU.cpp
    src/AlphaSim.cpp
    # src/Cirrus.cpp
    src/Configurator.cpp
    src/DEC21143.cpp
    src/Disk.cpp
    src/DiskController.cpp
    src/DiskDevice.cpp
    src/DiskFile.cpp
    src/DiskRam.cpp
    src/DMA.cpp
    src/dox.cpp
    src/DPR.cpp
    src/es40_debug.cpp
    src/ES1370.cpp
    src/Ethernet.cpp
    src/Flash.cpp
    src/FloppyController.cpp
    src/i2c_spd.cpp
    src/ibm8514a.cpp
    src/Keyboard.cpp
    src/lockstep.cpp
    src/MPU401.cpp
    src/PCIDevice.cpp
    src/Port80.cpp
    src/S3Trio64.cpp
    src/SCSIBus.cpp
    src/SCSIDevice.cpp
    src/Serial.cpp
    src/StdAfx.cpp
    src/Sym53C810.cpp
    src/Sym53C895.cpp
    src/System.cpp
    src/SystemComponent.cpp
    src/TraceEngine.cpp
    src/VGA.cpp
    )

if (WIN32)
    list(APPEND ES40_LIBRARIES ws2_32)
endif()

if (NOT ES40_DISABLE_SDL)
    list(APPEND ES40_DEFINITIONS HAVE_SDL)
    list(APPEND ES40_LIBRARIES SDL3::SDL3)

    list(APPEND ES40_SOURCES
        src/gui/sdl.cpp
        src/gui/gui.cpp
        )
    
    if (WIN32)
        list(APPEND ES40_SOURCES
            src/gui/gui_win32.cpp
            )
        list(APPEND ES40_LIBRARIES winmm)
    else()
        list(APPEND ES40_SOURCES
            src/gui/gui_x11.cpp
            )
    endif()
endif()

if (NOT ES40_DISABLE_NETWORKING)
    list(APPEND ES40_DEFINITIONS HAVE_PCAP)
    if (WIN32)
        # list(APPEND ES40_LIBRARIES NPCap::NPCap)
    else()
        list(APPEND ES40_LIBRARIES PCap::PCap)
    endif()
endif()

list(APPEND ES40_INCLUDES
    src/
    src/emu
    src/base
    src/gui
    )

add_executable(es40 ${ES40_SOURCES})
if (NOT ES40_DISABLE_IDB)
    add_executable(es40_idb ${ES40_SOURCES})
endif()
if (NOT ES40_DISABLE_LSS_LSM)
    add_executable(es40_lss ${ES40_SOURCES})
    add_executable(es40_lsm ${ES40_SOURCES})
endif()

target_include_directories(es40 PUBLIC ${ES40_INCLUDES})
if (NOT ES40_DISABLE_IDB)
    target_include_directories(es40_idb PUBLIC ${ES40_INCLUDES})
endif()
if (NOT ES40_DISABLE_LSS_LSM)
    target_include_directories(es40_lss PUBLIC ${ES40_INCLUDES})
    target_include_directories(es40_lsm PUBLIC ${ES40_INCLUDES})
endif()

target_link_libraries(es40 PUBLIC ${ES40_LIBRARIES})
if (NOT ES40_DISABLE_IDB)
    target_link_libraries(es40_idb PUBLIC ${ES40_LIBRARIES})
endif()
if (NOT ES40_DISABLE_LSS_LSM)
    target_link_libraries(es40_lss PUBLIC ${ES40_LIBRARIES})
    target_link_libraries(es40_lsm PUBLIC ${ES40_LIBRARIES})
endif()

target_compile_definitions(es40 PUBLIC ${ES40_DEFINITIONS})
if (NOT ES40_DISABLE_IDB)
    target_compile_definitions(es40_idb PUBLIC ${ES40_DEFINITIONS} IDB)
endif()
if (NOT ES40_DISABLE_LSS_LSM)
    target_compile_definitions(es40_lss PUBLIC ${ES40_DEFINITIONS} IDB LS_SLAVE)
    target_compile_definitions(es40_lsm PUBLIC ${ES40_DEFINITIONS} IDB LS_MASTER)
endif()

set_property(TARGET es40 PROPERTY CXX_STANDARD 17)
if (NOT ES40_DISABLE_IDB)
    set_property(TARGET es40_idb PROPERTY CXX_STANDARD 17)
endif()
if (NOT ES40_DISABLE_LSS_LSM)
    set_property(TARGET es40_lss PROPERTY CXX_STANDARD 17)
    set_property(TARGET es40_lsm PROPERTY CXX_STANDARD 17)
endif()
