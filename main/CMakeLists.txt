file(GLOB_RECURSE pizda_sources
    "*.*"
)

idf_component_register(
    SRCS ${pizda_sources}
    PRIV_REQUIRES spi_flash
    INCLUDE_DIRS "."
    REQUIRES esp_timer esp_driver_ledc esp_driver_i2c esp_driver_uart esp_driver_gptimer nvs_flash ESP-IDF-NVSExtensions ESP-IDF-Units ESP-IDF-BitStream ESP-IDF-SX1262
)
