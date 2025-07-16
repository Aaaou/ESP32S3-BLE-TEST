idf_component_register(SRCS "esp32_s3_szp.c" 
                            "main.c"
                            "ble_file_transfer.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hw_support nvs_flash bt esp_driver_gpio)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wno-unused-const-variable
    -Wno-implicit-function-declaration
    -Wno-unused-function
)
