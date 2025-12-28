# Generate HTML header from HTML file
set(HTML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/web_ui.html")
set(HTML_HEADER "${CMAKE_CURRENT_BINARY_DIR}/web_ui_html.h")

# Run Python script to convert HTML to header
add_custom_command(
    OUTPUT ${HTML_HEADER}
    COMMAND ${CMAKE_COMMAND} -E env PYTHONIOENCODING=utf-8
        python3 "${CMAKE_CURRENT_SOURCE_DIR}/embed_html.py" "${HTML_FILE}" "${HTML_HEADER}"
    DEPENDS ${HTML_FILE} "${CMAKE_CURRENT_SOURCE_DIR}/embed_html.py"
    COMMENT "Generating web_ui_html.h from web_ui.html"
)

idf_component_register(
    SRCS 
        "main.c"
        "board.c"
        "segment.c"
        "stepper_executor.c"
        "motion_planner.c"
        "quintic.c"
        "homing.c"
        "preset_storage.c"
        "usb_serial.c"
        "motion_controller.c"
        "tmc2209.c"
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

