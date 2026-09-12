idf_component_register(
    SRCS "src/gpsLib.cpp"
    INCLUDE_DIRS "src"
    PRIV_REQUIRES esp_timer esp_driver_uart
)
