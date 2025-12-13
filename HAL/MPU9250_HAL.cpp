#include "MPU9250_HAL.hpp"
#include <cstring>

MPU9250_HAL::MPU9250_HAL(i2c_inst_t* i2c, uint8_t address)
: i2c_(i2c), address_(address), i2c_configured_(false){ }

bool MPU9250_HAL::begin(uint sda_pin, uint scl_pin, uint32_t baudrate_hz) 
{
    i2c_init(i2c_, baudrate_hz);
    
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    i2c_configured_ = true;

    sleep_ms(10);
    return testConnection();
}

bool MPU9250_HAL::testConnection() // edit private function
{
    if(!i2c_configured_)
    {
        return false;
    }
    uint8_t who;
    readBytes(WHO_AM_I,&who,1);
    /* WHO_AM_I for MPU6500 typically 0x70 or 0x71 or 0x73 etc depending on part; accept non-zero */
    if((who == 70) || (who == 71) || (who == 73)) // edit to the first 
    {
        return (true);
    }
    else
    {
        return false;
    }
       
}

bool MPU9250_HAL::initMPU9250() 
{
    if(!i2c_configured_)
    {
        return false;
    }

    /* Reset device : write 1 on bit 7 */
    if(!writeByte(PWR_MGMT_1, 0x80)) 
    {
        return false;
    }
    sleep_ms(100);

    /* Wake up and set clock source to PLL with X axis gyroscope reference: write 1 on bit 0 */
    if(!writeByte(PWR_MGMT_1, 0x01))
    {
        return false;
    }
    sleep_ms(50);


    /* CONFIG: disable FSYNC, set DLPF (0) , enable LPS */
    if(!writeByte(CONFIG, 0x03)) 
    {
        return false; // set some DLPF
    }
    
    /* Set sample rate divider (SMPLRT_DIV) */
    if(!writeByte(SMPLRT_DIV, 0x04))
    {
        return false; // sample = gyro_rate/(1+4)
    } 

    /* ACCEL_CONFIG2: set DLPF for accel */
    if(!writeByte(ACCEL_CONFIG2, 0x03))
    {
        return false;
    }

    /*

    // INT pin config: enable bypass to access magnetometer directly if needed
    if(!writeByte(INT_PIN_CFG, 0x02))
    {
        return false;
    }

    // Enable data ready interrupt 
    if(!writeByte(INT_ENABLE, 0x01))
    {
        return false;
    }
    
    sleep_ms(50);

    // Disable I2C master (we use bypass), disable sleep mode important for magnometer 
    if(!writeByte(USER_CTRL, 0x00))
    {
        return false;
    }
    sleep_ms(10);
    */

    return true;
}

bool MPU9250_HAL::readBytes(uint8_t reg, uint8_t* buffer, size_t len)
{
    // edit  will remove it 
    if(!i2c_configured_) 
    {
        return false;
    }

    int ret = i2c_write_blocking(i2c_, address_, &reg, 1, true); // keep master control (no stop) // edit return std_optional
    if (ret < 0)
    {
        return false;
    }
    int r = i2c_read_blocking(i2c_, address_, buffer, len, false);

    return (r == (int)len);
}

bool MPU9250_HAL::writeByte(uint8_t reg, uint8_t value) 
{
    if(!i2c_configured_)
    {
        return false;
    }

    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_blocking(i2c_, address_, buf, 2, false); // send with stop

    return (ret == 2);
}

bool MPU9250_HAL::readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) 
{
    uint8_t buf[6];
    if(!readBytes(ACCEL_XOUT_H, buf, 6)) 
    {
        return false;
    }

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

bool MPU9250_HAL::readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) 
{
    uint8_t buf[6];
    if(!readBytes(GYRO_XOUT_H, buf, 6)) 
    {
        return false;
    }

    gx = (int16_t)((buf[0] << 8) | buf[1]);
    gy = (int16_t)((buf[2] << 8) | buf[3]);
    gz = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

bool MPU9250_HAL::readTempRaw(int16_t &temp) 
{
    uint8_t buf[2];

    if (!readBytes(TEMP_OUT_H, buf, 2))
    {
        return false;
    }
    temp = (int16_t)((buf[0] << 8) | buf[1]);

    return true;
}

bool MPU9250_HAL::readAllRaw(int16_t &ax, int16_t &ay, int16_t &az,
                             int16_t &gx, int16_t &gy, int16_t &gz,
                             int16_t &temp)
{
    // read 14 bytes ACCEL..TEMP..GYRO
    uint8_t buf[14];
    if(!readBytes(ACCEL_XOUT_H, buf, 14)) 
    {
        return false;
    }

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);

    // buf[6..7] for temp
    temp = (int16_t)((buf[6]<< 8) | buf[7]);

    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}


bool MPU9250_HAL::initAK8963() 
{
    if (!writeByte(INT_PIN_CFG, 0x02)) // BYPASS_EN
    {
        return false;
    }
    return true;
}

bool MPU9250_HAL::readMagRaw(int16_t &mx, int16_t &my, int16_t &mz) 
{
    uint8_t buf[6];
    if (!i2c_configured_)
    {
        return false;
    }

    uint8_t reg = AK8963_XOUT_L;
    int ret = i2c_write_blocking(i2c_, AK8963_DEFAULT_ADDRESS, &reg, 1, true);
    if (ret < 0)
    {
        return false;
    }

    ret = i2c_read_blocking(i2c_, AK8963_DEFAULT_ADDRESS, buf, 6, false);
    if (ret != 6) 
    {
        return false;
    }

    mx = (int16_t)((buf[1] << 8) | buf[0]);
    my = (int16_t)((buf[3] << 8) | buf[2]);
    mz = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}
