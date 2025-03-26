if(ESP_PLATFORM)
    idf_component_register( SRCS "ESP/DCP.c"
                            INCLUDE_DIRS "include"
                            PRIV_REQUIRES "esp_ringbuf" "driver")

else()
    cmake_minimum_required(VERSION 3.24)
    project(DCP LANGUAGES C)

    # Add the DCP library
    if (ESP32)
        add_library(DCP STATIC ESP/DCP.c)
    elseif(RPICO2)
        add_library(DCP STATIC RPICO2/DCP.c)
    endif()

    # Specify the include directory for the library
    target_include_directories(DCP PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
endif()
