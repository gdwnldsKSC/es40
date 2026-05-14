cmake_minimum_required(VERSION 4.1)

project(ES40 LANGUAGES C CXX)

option(ES40_DISABLE_SDL OFF "Disable SDL for headless builds")
option(ES40_DISABLE_NETWORKING OFF "Disable PCap/networking for networkless builds")
#option(ES40_DISABLE_ASMJIT OFF "Disable ASMJIT")

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
        if (NOT NPCAP_DIR)
            set(NPCAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../NPCAP")
        endif()
        find_package(NPCap REQUIRED)
    else()
        find_package(PCap REQUIRED)
    endif()
endif()

############################# ES40-CFG #############################

set(ES40_CFG_SOURCES)
set(ES40_CFG_LIBRARIES)
set(ES40_CFG_DEFINITIONS)

list(APPEND ES40_CFG_SOURCES
    src/es40-cfg.cpp
    src/base/Exception.cpp
    )

if (NOT ES40_DISABLE_NETWORKING)
    list(APPEND ES40_CFG_DEFINITIONS HAVE_PCAP)
    if (WIN32)
        list(APPEND ES40_CFG_LIBRARIES NPCap::NPCap)
    else()
        list(APPEND ES40_CFG_LIBRARIES PCap::PCap)
    endif()
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
    else()
        list(APPEND ES40_SOURCES
            src/gui/gui_x11.cpp
            )
    endif()
endif()

if (NOT ES40_DISABLE_NETWORKING)
    list(APPEND ES40_DEFINITIONS HAVE_PCAP)
    if (WIN32)
        list(APPEND ES40_LIBRARIES NPCap::NPCap)
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
add_executable(es40_idb ${ES40_SOURCES})
add_executable(es40_lss ${ES40_SOURCES})
add_executable(es40_lsm ${ES40_SOURCES})

target_include_directories(es40 PUBLIC ${ES40_INCLUDES})
target_include_directories(es40_idb PUBLIC ${ES40_INCLUDES})
target_include_directories(es40_lss PUBLIC ${ES40_INCLUDES})
target_include_directories(es40_lsm PUBLIC ${ES40_INCLUDES})

target_link_libraries(es40 PUBLIC ${ES40_LIBRARIES})
target_link_libraries(es40_idb PUBLIC ${ES40_LIBRARIES})
target_link_libraries(es40_lss PUBLIC ${ES40_LIBRARIES})
target_link_libraries(es40_lsm PUBLIC ${ES40_LIBRARIES})

target_compile_definitions(es40 PUBLIC ${ES40_DEFINITIONS})
target_compile_definitions(es40_idb PUBLIC ${ES40_DEFINITIONS} IDB)
target_compile_definitions(es40_lss PUBLIC ${ES40_DEFINITIONS} IDB LS_SLAVE)
target_compile_definitions(es40_lsm PUBLIC ${ES40_DEFINITIONS} IDB LS_MASTER)

set_property(TARGET es40 PROPERTY CXX_STANDARD 17)
set_property(TARGET es40_idb PROPERTY CXX_STANDARD 17)
set_property(TARGET es40_lss PROPERTY CXX_STANDARD 17)
set_property(TARGET es40_lsm PROPERTY CXX_STANDARD 17)
