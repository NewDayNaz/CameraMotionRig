# Generate HTML header from HTML file
set(HTML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/web_ui.html")
set(HTML_HEADER "${CMAKE_CURRENT_BINARY_DIR}/web_ui_html.h")

# Find Python executable (try python3 first, then python)
find_program(PYTHON_EXECUTABLE
    NAMES python3 python
    REQUIRED
)

idf_component_register(
    SRCS 
        "main.c"
        "board.c"
        "stepper_simple.c"
        "preset_storage.c"
        "usb_serial.c"
        "wifi_manager.c"
        "http_server.c"
    INCLUDE_DIRS 
        "."
        "${CMAKE_CURRENT_BINARY_DIR}"
    PRIV_REQUIRES 
        driver
        nvs_flash
        esp_timer
        esp_wifi
        esp_http_server
        json
        esp_netif
        app_update
)

# Generate HTML header file (only during build phase, not requirements phase)
# ESP-IDF processes component CMakeLists.txt in two phases:
# 1. Requirements phase (scripting mode) - metadata gathering  
# 2. Build phase (project mode) - actual build configuration
# add_custom_command can only be used in phase 2.
# Try to get component target - this will only succeed during build phase
# If it fails, we're in requirements phase and should skip
if(COMMAND idf_build_get_property)
    idf_build_get_property(COMPONENT_TARGET component_target)
    if(component_target)
        add_custom_command(
            OUTPUT ${HTML_HEADER}
            COMMAND ${CMAKE_COMMAND} -E env PYTHONIOENCODING=utf-8
                ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/embed_html.py" "${HTML_FILE}" "${HTML_HEADER}"
            DEPENDS ${HTML_FILE} "${CMAKE_CURRENT_SOURCE_DIR}/embed_html.py"
            COMMENT "Generating web_ui_html.h from web_ui.html"
        )
        
        # Make sure the header is generated before compilation
        add_custom_target(generate_html_header ALL DEPENDS ${HTML_HEADER})
        add_dependencies(${component_target} generate_html_header)
        
        # Also add the header as a dependency to http_server.c specifically
        set_source_files_properties(http_server.c PROPERTIES OBJECT_DEPENDS ${HTML_HEADER})
    endif()
endif()

