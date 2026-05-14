include(FindPackageHandleStandardArgs)

set(_NPCap_HINTS
    "$ENV{NPCAP_DIR}"
    "$ENV{NPCAP_ROOT}"
    "${NPCAP_DIR}"
    "${NPCAP_ROOT}"
    "$ENV{NPCap_DIR}"
    "$ENV{NPCap_ROOT}"
    "${NPCap_DIR}"
    "${NPCap_ROOT}"
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_NPCap_LIB_SUFFIXES Lib/x64)
else()
    set(_NPCap_LIB_SUFFIXES Lib)
endif()

find_path(NPCap_INCLUDE_DIR
    NAMES pcap.h pcap/pcap.h
    HINTS ${_NPCap_HINTS}
    PATH_SUFFIXES Include include
)

find_library(NPCap_WPCAP_LIBRARY
    NAMES wpcap
    HINTS ${_NPCap_HINTS}
    PATH_SUFFIXES ${_NPCap_LIB_SUFFIXES}
)

find_library(NPCap_PACKET_LIBRARY
    NAMES Packet packet
    HINTS ${_NPCap_HINTS}
    PATH_SUFFIXES ${_NPCap_LIB_SUFFIXES}
)

find_package_handle_standard_args(NPCap
    REQUIRED_VARS
        NPCap_INCLUDE_DIR
        NPCap_WPCAP_LIBRARY
        NPCap_PACKET_LIBRARY
)

if(NPCap_FOUND)
    set(NPCap_INCLUDE_DIRS "${NPCap_INCLUDE_DIR}")
    set(NPCap_LIBRARIES
        "${NPCap_WPCAP_LIBRARY}"
        "${NPCap_PACKET_LIBRARY}"
    )

    if(NOT TARGET NPCap::NPCap)
        add_library(NPCap::NPCap INTERFACE IMPORTED)

        set_target_properties(NPCap::NPCap PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${NPCap_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${NPCap_LIBRARIES}"
        )
    endif()
endif()

mark_as_advanced(
    NPCap_INCLUDE_DIR
    NPCap_WPCAP_LIBRARY
    NPCap_PACKET_LIBRARY
)