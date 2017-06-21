//-----------------------------------------------------------------------------
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

static int _remove_this_counter;
static int on_off;

//-----------------------------------------------------------------------------
#define COMM SD2
#define COMMS ((BaseSequentialStream*)&COMM)
static const SerialConfig SD2_config =
{
    921600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};

//-----------------------------------------------------------------------------
static const I2CConfig i2c1_config =
{
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

//-----------------------------------------------------------------------------
void start_write(void)
{
    if ( on_off ) return;
    palSetPad(GPIOB, GPIOB_USART1_DE);
}

//-----------------------------------------------------------------------------
void end_write(void)
{
    if ( on_off ) return;
    palClearPad(GPIOB, GPIOB_USART1_DE);
}

//-----------------------------------------------------------------------------
int stdout_printf(BaseSequentialStream *chp, const char *fmt, ...)
{
    va_list ap;
    int formatted_bytes;

    start_write();
    va_start(ap, fmt);
    formatted_bytes = chvprintf(chp, fmt, ap);
    va_end(ap);
    end_write();

    return formatted_bytes;
}

//-----------------------------------------------------------------------------
void initialise(void)
{
    // SD2 comms
    sdStart(&COMM, &SD2_config);
    palClearPad(GPIOB, GPIOB_USART1_DE);
    palClearPad(GPIOB, GPIOB_USART1_RE);

    // i2c - the barometer (and temperature sensor) hang off this bus, the
    // gps too, though the preferred method is USART1 for GPS
    i2cStart(&I2CD1, &i2c1_config);
}

//-----------------------------------------------------------------------------
#define ADDR 0x60 // C0 write / C1 read
static uint8_t rx_data[8];
static uint8_t tx_data[8];
static uint8_t txbuf[2];    // i only ever write out 2 regs
static uint8_t rxbuf[5];    // i only ever read the 5 status regs
static i2cflags_t errors = 0;

//-----------------------------------------------------------------------------
void write_reg(uint8_t reg, uint8_t dat)
{
    systime_t tmo = MS2ST(4);
    tx_data[0] = reg;
    tx_data[1] = dat;
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, ADDR, tx_data, 2, rx_data, 0, tmo);
    i2cReleaseBus(&I2CD1);
}

//-----------------------------------------------------------------------------
uint8_t read_reg(uint8_t reg)
{
    systime_t tmo = MS2ST(4);
    tx_data[0] = reg;
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, ADDR, tx_data, 1, rx_data, 1, tmo);
    i2cReleaseBus(&I2CD1);
    return rx_data[0];
}

//-----------------------------------------------------------------------------
msg_t write_buf(uint8_t reg, uint8_t dat)
{
    systime_t timeout = MS2ST(4);
    txbuf[0] = reg;
    txbuf[1] = dat;
    i2cAcquireBus(&I2CD1);
    msg_t status = i2cMasterTransmitTimeout(&I2CD1, ADDR, txbuf, 2, rxbuf, 0, timeout);
    i2cReleaseBus(&I2CD1);
    if (status != MSG_OK)
    {
        errors = i2cGetErrors(&I2CD1);
    }
    return status;
}

//-----------------------------------------------------------------------------
msg_t read_buf(uint8_t reg, uint8_t * dat, uint8_t dat_len)
{
    systime_t timeout = MS2ST(4);
    txbuf[0] = reg;
    i2cAcquireBus(&I2CD1);
    msg_t status = i2cMasterTransmitTimeout(&I2CD1, ADDR, txbuf, 1, dat, dat_len, timeout);
    i2cReleaseBus(&I2CD1);
    if (status != MSG_OK)
    {
        errors = i2cGetErrors(&I2CD1);
    }
    return status;
}

//-----------------------------------------------------------------------------
int main(void)
{
    halInit();
    chSysInit();

    initialise();

    chThdSleepMilliseconds(100);
    stdout_printf(COMMS, "Yep, hi there!!\r\n");
    chThdSleepMilliseconds(100);

    write_reg(0x26, 0xB8); // set to altimeter with an OSR=128
    write_reg(0x13, 0x07); // enable data flags in PT_DATA_CFG
    write_reg(0x26, 0xB9); // set active

    chThdSleepMilliseconds(100);
    stdout_printf(COMMS, "All done!!\r\n");
    chThdSleepMilliseconds(100);

    /*
    do
    {
        uint8_t sta = read_reg(0x00);
        if (sta & 0x08)
            break;

        if (palReadPad(GPIOA, GPIOA_BUTTON))
            palSetPad(GPIOA, GPIOA_LED);
        else
            palClearPad(GPIOA, GPIOA_LED);
    } while(1);

     */

    while( true )
    {
        uint8_t status = read_buf(0x01, rx_data, 5);
        if ( status == MSG_OK )
        {
            int m_a,m_t,c_a;
            float l_a,l_t;

            m_a = rx_data[0];
            c_a = rx_data[1];
            l_a = (float)(rx_data[2]>>4)/16.0f;
            m_t = rx_data[3];
            l_t = (float)(rx_data[4]>>4)/16.0f;
            float altitude = (float)((m_a << 8)|c_a)+l_a;
            float temperature = (float)(m_t + l_t);
            int ialtitude = (int) altitude;
            int itemperature = (int) temperature;

            stdout_printf(COMMS, "A: %d; T: %d / %d\r\n", ialtitude, itemperature, _remove_this_counter);
        }
        chThdSleepMilliseconds(200);
        if (palReadPad(GPIOA, GPIOA_BUTTON))
        {
            on_off = 0;
            palSetPad(GPIOA, GPIOA_LED);
        }
        else
        {
            on_off = 1;
            palClearPad(GPIOA, GPIOA_LED);
        }
        _remove_this_counter++;
    }


    while (true)
    {
        if (palReadPad(GPIOA, GPIOA_BUTTON))
            palSetPad(GPIOA, GPIOA_LED);
        else
            palClearPad(GPIOA, GPIOA_LED);
    }
}
