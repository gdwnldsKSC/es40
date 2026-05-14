include(FindPackageHandleStandardArgs)

set(_PCap_HINTS
    "$ENV{PCAP_DIR}"
    "$ENV{PCAP_ROOT}"
    "${PCAP_DIR}"
    "${PCap_DIR}"
    "${PCAP_ROOT}"
    "${PCap_ROOT}"
    /usr
    /usr/local
    /opt/local
    /opt/homebrew
)

find_path(PCap_INCLUDE_DIR
    NAMES pcap.h pcap/pcap.h
    HINTS ${_PCap_HINTS}
    PATH_SUFFIXES include
)

find_library(PCap_LIBRARY
    NAMES pcap
    HINTS ${_PCap_HINTS}
    PATH_SUFFIXES lib lib64
)

find_package_handle_standard_args(PCap
    REQUIRED_VARS
        PCap_INCLUDE_DIR
        PCap_LIBRARY
)

if(PCap_FOUND)
    set(PCap_INCLUDE_DIRS "${PCap_INCLUDE_DIR}")
    set(PCap_LIBRARIES "${PCap_LIBRARY}")

    if(NOT TARGET PCap::PCap)
        add_library(PCap::PCap INTERFACE IMPORTED)

        set_target_properties(PCap::PCap PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PCap_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${PCap_LIBRARY}"
        )
    endif()
endif()

mark_as_advanced(
    PCap_INCLUDE_DIR
    PCap_LIBRARY
)