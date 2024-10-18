
target_sources(sensor PRIVATE GPIO.cpp i2c.cpp spi.cpp uart.cpp PUBLIC ${PROJECT_SOURCE_DIR} GPIO.h i2c.h spi.h uart.h)