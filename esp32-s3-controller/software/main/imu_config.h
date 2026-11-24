#pragma once

// Wiring confirmed by the user's 2026-10-17 screenshot. INT1/GPIO9 is unused;
// STATUS_REG polling avoids requiring an interrupt wire.
#define IMU_SPI_SCK_GPIO  12
#define IMU_SPI_MOSI_GPIO 11
#define IMU_SPI_MISO_GPIO 13
#define IMU_SPI_CS_GPIO   10

// Upright reference measured on 2026-10-23; persisted from the verified RAM zero.
#define IMU_ROLL_ZERO_DEG (-177.796f)
#define IMU_PITCH_ZERO_DEG (-0.312f)
