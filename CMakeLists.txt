if(ESP_PLATFORM)
    idf_component_register( SRCS "ESP/DCP.c"
                            INCLUDE_DIRS "include"
                            PRIV_REQUIRES "esp_ringbuf" "driver")

else()
    add_library(DCP STATIC)

    target_sources(DCP PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../DCP/generic/DCP.c
    )

    target_include_directories(DCP PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(DCP PRIVATE FreeRTOS-Kernel)
endif()
