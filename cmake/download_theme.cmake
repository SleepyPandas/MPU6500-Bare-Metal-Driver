# cmake/download_theme.cmake
# Downloads doxygen-awesome-css theme files for local documentation builds.
# Called by the 'docs' CMake target. Uses CMake's built-in file(DOWNLOAD)
# so it works on Windows, macOS, and Linux without shell/curl dependencies.
#
# Required variable: THEME_DIR (absolute path to docs/theme/)

if(NOT THEME_DIR)
    message(FATAL_ERROR "THEME_DIR not set")
endif()

set(AWESOME_VERSION "v2.3.4")
set(BASE_URL "https://raw.githubusercontent.com/jothepro/doxygen-awesome-css/${AWESOME_VERSION}")

foreach(FILE
        doxygen-awesome.css
        doxygen-awesome-sidebar-only.css
        doxygen-awesome-darkmode-toggle.js)
    if(NOT EXISTS "${THEME_DIR}/${FILE}")
        message(STATUS "Downloading doxygen-awesome: ${FILE}")
        file(DOWNLOAD
            "${BASE_URL}/${FILE}"
            "${THEME_DIR}/${FILE}"
            STATUS DOWNLOAD_STATUS
            SHOW_PROGRESS
        )
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        if(NOT STATUS_CODE EQUAL 0)
            message(WARNING "Failed to download ${FILE} (code ${STATUS_CODE}). "
                            "Download manually from ${BASE_URL}/${FILE}")
        endif()
    endif()
endforeach()
