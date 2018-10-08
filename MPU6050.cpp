void MPU6050::setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

void MPU6050::setRange( uint8_t range )
{
    uint8_t value;

    value = port.read(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);

    port.write(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setScale( uint8_t scale)
{
    uint8_t value;

    value = port.read(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);

    port.write(MPU6050_REG_GYRO_CONFIG, value);
}




void MPU6050::setSleepEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}