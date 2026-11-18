
from machine import Pin, SPI
import time
import struct

# XIAO RP2350 SPI0
spi = SPI(
    0,
    baudrate=1_000_000,
    polarity=0,
    phase=0,
    sck=Pin(2),   # D8
    mosi=Pin(3),  # D10
    miso=Pin(4),  # D9
)

cs = Pin(5, Pin.OUT, value=1)  # D3


def read_reg(address, length=1):
    cs.value(0)
    spi.write(bytes([address | 0x80]))
    data = spi.read(length)
    cs.value(1)
    return data


def write_reg(address, value):
    cs.value(0)
    spi.write(bytes([address & 0x7F, value]))
    cs.value(1)


# Switch BMI160 into SPI mode
time.sleep_ms(50)
read_reg(0x7F)
time.sleep_ms(2)

# Verify sensor identity
chip_id = read_reg(0x00)[0]
print("BMI160 CHIP_ID:", hex(chip_id))

if chip_id != 0xD1:
    raise RuntimeError("BMI160 not detected: check wiring")

# Configure ranges
write_reg(0x41, 0x03)  # Accelerometer: +/-2g
write_reg(0x43, 0x00)  # Gyroscope: +/-2000 dps

# Enable sensors
write_reg(0x7E, 0x11)  # Accelerometer normal mode
time.sleep_ms(10)

write_reg(0x7E, 0x15)  # Gyroscope normal mode
time.sleep_ms(100)

print("Reading IMU data...")

while True:
    data = read_reg(0x0C, 12)

    gx, gy, gz, ax, ay, az = struct.unpack(
        "<hhhhhh", data
    )

    # Convert raw readings
    ax, ay, az = [v / 16384.0 for v in (ax, ay, az)]
    gx, gy, gz = [v / 16.4 for v in (gx, gy, gz)]

    print(
        "Accel (g): {:.2f}, {:.2f}, {:.2f} | "
        "Gyro (dps): {:.2f}, {:.2f}, {:.2f}".format(
            ax, ay, az, gx, gy, gz
        )
    )

    time.sleep_ms(100)
