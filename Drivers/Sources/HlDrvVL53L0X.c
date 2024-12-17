// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <stdint.h>
#include <DrvSYS.h>
#include "DrvTWI.h"
#include "HlDrvVL53L0X.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
const uint8_t HlDrvVL53L0X_InitData[] PROGMEM = {
/*Addr  Data */
#define HLDRVVL53L0X_BLOB_DEFAULT_TUNING_SETTINGS 0, 80
    0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00,
    0x10, 0x00, 0x11, 0x00, 0x24, 0x01, 0x25, 0xFF,
    0x75, 0x00, 0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00,
    0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00,
    0x31, 0x04, 0x32, 0x03, 0x40, 0x83, 0x46, 0x25,
    0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00,
    0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00,
    0x62, 0x00, 0x64, 0x00, 0x65, 0x00, 0x66, 0xA0,
    0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF,
    0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00,
    0x78, 0x21, 0xFF, 0x01, 0x23, 0x34, 0x42, 0x00,
    0x44, 0xFF, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40,
    0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00,
    0x34, 0x03, 0x35, 0x44, 0xFF, 0x01, 0x31, 0x04,
    0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00,
    0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28,
    0x67, 0x00, 0x70, 0x04, 0x71, 0x01, 0x72, 0xFE,
    0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01,
    0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01,
    0x8E, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00,
#define HLDRVVL53L0X_BLOB_DATA_INIT 160, 4
    0x88, 0x00, 0x80, 0x01, 0xFF, 0x01, 0x00, 0x00,
#define HLDRVVL53L0X_BLOB_DATA_INIT2 168, 3
    0x00, 0x01, 0xFF, 0x00, 0x80, 0x00,
#define HLDRVVL53L0X_BLOB_SET_REFERENCE_SPADS 174, 5
    0xFF, 0x01, 0x4F, 0x00, 0x4E, 0x2C, 0xFF, 0x00,
    0xB6, 0xB4,
#define HLDRVVL53L0X_BLOB_GET_SPAD_INFO_A 184, 4
    0x80, 0x01, 0xFF, 0x01, 0x00, 0x00, 0xFF, 0x06,
#define HLDRVVL53L0X_BLOB_GET_SPAD_INFO_B 192, 5
    0xFF, 0x07, 0x81, 0x01, 0x80, 0x01, 0x94, 0x6b,
    0x83, 0x00,
#define HLDRVVL53L0X_BLOB_GET_SPAD_INFO_C 202, 4
    0xFF, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00,

#define HLDRVVL53L0X_BLOB_x 206,
    //_HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_GET_SPAD_INFO_A);
};

//---------------------------------------------------------
// Local variables within this file (private)
//---------------------------------------------------------
uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0; // no timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t g_measTimBudUs;

//---------------------------------------------------------
// Locally used functions (private)
//---------------------------------------------------------
uint8_t _HlDrvVL53L0X_GetSPADInfo(uint8_t *count, uint8_t *type_is_aperture);
void _HlDrvVL53L0X_GetSequenceStepEnables(SequenceStepEnables *enables);
void _HlDrvVL53L0X_GetSequenceStepTimeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);
uint8_t _HlDrvVL53L0X_PerformSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t _HlDrvVL53L0X_DecodeTimeout(uint16_t value);
static uint16_t _HlDrvVL53L0X_EncodeTimeout(uint16_t timeout_mclks);
static uint32_t _HlDrvVL53L0X_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

uint8_t buffer[3];
//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
// Write an 8-bit register

void _HlDrvVL53L0X_BulkWrite(uint8_t start, uint8_t count)
{
#ifdef LLDEBUG
    printf("bulkwrite %i->%02X\r\n", start, pgm_read_byte(&HlDrvVL53L0X_InitData[start]));
#endif
    while (count--)
    {
        buffer[0] = pgm_read_byte(&HlDrvVL53L0X_InitData[start++]);
        buffer[1] = pgm_read_byte(&HlDrvVL53L0X_InitData[start++]);
        _HlDrvVL53L0X_WriteReg(buffer[0], buffer[1]);
    }
}

void _HlDrvVL53L0X_WriteReg(uint8_t reg, uint8_t value)
{
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_Write(value);
    DrvTWI_Stop();
}

// Write a 16-bit register
void _HlDrvVL53L0X_WriteReg16Bit(uint8_t reg, uint16_t value)
{
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_Write((value >> 8) & 0xFF);
    DrvTWI_Write((value) & 0xFF);
    DrvTWI_Stop();
}

// Write a 32-bit register
void _HlDrvVL53L0X_WriteReg32Bit(uint8_t reg, uint32_t value)
{
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_Write((value >> 24) & 0xFF);
    DrvTWI_Write((value >> 16) & 0xFF);
    DrvTWI_Write((value >> 8) & 0xFF);
    DrvTWI_Write((value) & 0xFF);
    DrvTWI_Stop();
}

// Read an 8-bit register
uint8_t _HlDrvVL53L0X_ReadReg(uint8_t reg)
{
    uint8_t value;
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_RepeatedStart(g_i2cAddr | I2C_READ);
    value = DrvTWI_ReadNAck();
    DrvTWI_Stop();
    return value;
}

// Read a 16-bit register
uint16_t _HlDrvVL53L0X_ReadReg16Bit(uint8_t reg)
{
    uint16_t value;
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_RepeatedStart(g_i2cAddr | I2C_READ);
    value = DrvTWI_ReadAck() << 8;
    value |= DrvTWI_ReadNAck();
    DrvTWI_Stop();
    return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void _HlDrvVL53L0X_WriteMulti(uint8_t reg, uint8_t const *src, uint8_t count)
{
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    while (count-- > 0)
    {
        DrvTWI_Write(*src++);
    }
    DrvTWI_Stop();
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void _HlDrvVL53L0X_ReadMulti(uint8_t reg, uint8_t *dst, uint8_t count)
{
    DrvTWI_Start(g_i2cAddr | I2C_WRITE);
    DrvTWI_Write(reg);
    DrvTWI_RepeatedStart(g_i2cAddr | I2C_READ);
    while (count > 0)
    {
        if (count > 1)
        {
            *dst++ = DrvTWI_ReadAck();
        }
        else
        {
            *dst++ = DrvTWI_ReadNAck();
        }
        count--;
    }
    DrvTWI_Stop();
}

// Public Methods //////////////////////////////////////////////////////////////

void HlDrvVL53L0X_Deinit()
{
    PCICR = 0;
    PCMSK2 = 0;
    cli();
    DrvPWR_ModuleDisable(PRR_PCI);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is 1 or not given, the sensor is configured for 2V8
// mode.
uint8_t HlDrvVL53L0X_Init()
{
    // VL53L0X_DataInit() begin

    // PCINT auf PD4
    DrvPWR_ModuleEnable(PRR_PCI);
    PCICR = (1 << PCIE2);
    PCMSK2 = (1 << PCINT20);
    sei();

    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_DATA_INIT);
    g_stopVariable = _HlDrvVL53L0X_ReadReg(0x91);
    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_DATA_INIT2);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    _HlDrvVL53L0X_WriteReg(MSRC_CONFIG_CONTROL, _HlDrvVL53L0X_ReadReg(MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    HlDrvVL53L0X_SetSignalRateLimit(0.25);

    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin
    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    if (!_HlDrvVL53L0X_GetSPADInfo(&spad_count, &spad_type_is_aperture))
    {
        return 0;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    _HlDrvVL53L0X_ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_SET_REFERENCE_SPADS);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    _HlDrvVL53L0X_WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_DEFAULT_TUNING_SETTINGS);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    _HlDrvVL53L0X_WriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    _HlDrvVL53L0X_WriteReg(GPIO_HV_MUX_ACTIVE_HIGH, _HlDrvVL53L0X_ReadReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    _HlDrvVL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    g_measTimBudUs = HlDrvVL53L0X_GetMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    HlDrvVL53L0X_SetMeasurementTimingBudget(g_measTimBudUs);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!_HlDrvVL53L0X_PerformSingleRefCalibration(0x40))
    {
        return 0;
    }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!_HlDrvVL53L0X_PerformSingleRefCalibration(0x00))
    {
        return 0;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    // VL53L0X_PerformRefCalibration() end

    return 1;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
uint8_t HlDrvVL53L0X_SetSignalRateLimit(float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99)
    {
        return 0;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    _HlDrvVL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return 1;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(void)
{
    return (float)_HlDrvVL53L0X_ReadReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
uint8_t HlDrvVL53L0X_SetMeasurementTimingBudget(uint32_t budget_us)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
    {
        return 0;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    _HlDrvVL53L0X_GetSequenceStepEnables(&enables);
    _HlDrvVL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us)
        {
            // "Requested timeout too big."
            return 0;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(final_range_timeout_us,
                                                     timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        _HlDrvVL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    _HlDrvVL53L0X_EncodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        g_measTimBudUs = budget_us; // store for internal reuse
    }
    return 1;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t HlDrvVL53L0X_GetMeasurementTimingBudget(void)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    _HlDrvVL53L0X_GetSequenceStepEnables(&enables);
    _HlDrvVL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    g_measTimBudUs = budget_us; // store for internal reuse
    return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
uint8_t HlDrvVL53L0X_SetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    _HlDrvVL53L0X_GetSequenceStepEnables(&enables);
    _HlDrvVL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type == VcselPeriodPreRange)
    {
        // "Set phase check limits"
        switch (period_pclks)
        {
        case 12:
            _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;

        case 14:
            _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;

        case 16:
            _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;

        case 18:
            _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;

        default:
            // invalid period
            return 0;
        }
        _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        _HlDrvVL53L0X_WriteReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks =
            _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        _HlDrvVL53L0X_WriteReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    _HlDrvVL53L0X_EncodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks =
            _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        _HlDrvVL53L0X_WriteReg(MSRC_CONFIG_TIMEOUT_MACROP,
                               (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    }
    else if (type == VcselPeriodFinalRange)
    {
        switch (period_pclks)
        {
        case 8:
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            _HlDrvVL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x30);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
            break;

        case 10:
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            _HlDrvVL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
            break;

        case 12:
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            _HlDrvVL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
            break;

        case 14:
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            _HlDrvVL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
            _HlDrvVL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
            _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
            break;

        default:
            // invalid period
            return 0;
        }

        // apply new VCSEL period
        _HlDrvVL53L0X_WriteReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
            _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range)
        {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        _HlDrvVL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    _HlDrvVL53L0X_EncodeTimeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
    }
    else
    {
        // invalid type
        return 0;
    }

    // "Finally, the timing budget must be re-applied"

    HlDrvVL53L0X_SetMeasurementTimingBudget(g_measTimBudUs);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = _HlDrvVL53L0X_ReadReg(SYSTEM_SEQUENCE_CONFIG);
    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    _HlDrvVL53L0X_PerformSingleRefCalibration(0x0);
    _HlDrvVL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return 1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t HlDrvVL53L0X_GetVcselPulsePeriod(vcselPeriodType type)
{
    if (type == VcselPeriodPreRange)
    {
        return decodeVcselPeriod(_HlDrvVL53L0X_ReadReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == VcselPeriodFinalRange)
    {
        return decodeVcselPeriod(_HlDrvVL53L0X_ReadReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else
    {
        return 255;
    }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void HlDrvVL53L0X_StartContinuous(uint32_t period_ms)
{
    _HlDrvVL53L0X_WriteReg(0x80, 0x01);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
    _HlDrvVL53L0X_WriteReg(0x00, 0x00);
    _HlDrvVL53L0X_WriteReg(0x91, g_stopVariable);
    _HlDrvVL53L0X_WriteReg(0x00, 0x01);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
    _HlDrvVL53L0X_WriteReg(0x80, 0x00);

    if (period_ms != 0)
    {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        uint16_t osc_calibrate_val = _HlDrvVL53L0X_ReadReg16Bit(OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }

        _HlDrvVL53L0X_WriteReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else
    {
        // continuous back-to-back mode
        _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void HlDrvVL53L0X_StopContinuous(void)
{
    _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
    _HlDrvVL53L0X_WriteReg(0x00, 0x00);
    _HlDrvVL53L0X_WriteReg(0x91, 0x00);
    _HlDrvVL53L0X_WriteReg(0x00, 0x01);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t HlDrvVL53L0X_ReadRangeContinuousMillimeters(statInfo_t *extraStats)
{
    uint8_t tempBuffer[12];
    uint16_t temp;
    
    _HlDrvVL53L0X_WaitForInt();
    
    if (extraStats == 0)
    {
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        temp = _HlDrvVL53L0X_ReadReg16Bit(RESULT_RANGE_STATUS + 10);
    }
    else
    {
        // Register map starting at 0x14
        //     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
        //    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
        //   0: Ranging status, uint8_t
        //   1: ???
        // 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
        //   4: 0 ?
        //   5: ???
        // 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
        // 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
        // A,B: uncorrected distance [mm], uint16_t
        _HlDrvVL53L0X_ReadMulti(0x14, tempBuffer, 12);
        extraStats->rangeStatus = tempBuffer[0x00] >> 3;
        extraStats->spadCnt = (tempBuffer[0x02] << 8) | tempBuffer[0x03];
        extraStats->signalCnt = (tempBuffer[0x06] << 8) | tempBuffer[0x07];
        extraStats->ambientCnt = (tempBuffer[0x08] << 8) | tempBuffer[0x09];
        temp = (tempBuffer[0x0A] << 8) | tempBuffer[0x0B];
        extraStats->rawDistance = temp;
    }
    _HlDrvVL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    return temp;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t HlDrvVL53L0X_ReadRangeSingleMillimeters(statInfo_t *extraStats)
{
    _HlDrvVL53L0X_WriteReg(0x80, 0x01);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x01);
    _HlDrvVL53L0X_WriteReg(0x00, 0x00);
    _HlDrvVL53L0X_WriteReg(0x91, g_stopVariable);
    _HlDrvVL53L0X_WriteReg(0x00, 0x01);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x00);
    _HlDrvVL53L0X_WriteReg(0x80, 0x00);
    _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x01);
    // "Wait until start bit has been cleared"
    // while (_HlDrvVL53L0X_ReadReg(SYSRANGE_START) & 0x01){
    //}
    return HlDrvVL53L0X_ReadRangeContinuousMillimeters(extraStats);
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
uint8_t _HlDrvVL53L0X_GetSPADInfo(uint8_t *count, uint8_t *type_is_aperture)
{
    uint8_t tmp;
    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_GET_SPAD_INFO_A);
    // 0x80, 0x01, 0xFF, 0x01, 0x00, 0x00, 0xFF, 0x06,
    _HlDrvVL53L0X_WriteReg(0x83, _HlDrvVL53L0X_ReadReg(0x83) | 0x04);
    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_GET_SPAD_INFO_B);
    while (_HlDrvVL53L0X_ReadReg(0x83) == 0x00)
    {
    }
    _HlDrvVL53L0X_WriteReg(0x83, 0x01);
    tmp = _HlDrvVL53L0X_ReadReg(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    _HlDrvVL53L0X_WriteReg(0x81, 0x00);
    _HlDrvVL53L0X_WriteReg(0xFF, 0x06);
    _HlDrvVL53L0X_WriteReg(0x83, _HlDrvVL53L0X_ReadReg(0x83) & ~0x04);
    _HlDrvVL53L0X_BulkWrite(HLDRVVL53L0X_BLOB_GET_SPAD_INFO_C);

    return 1;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void _HlDrvVL53L0X_GetSequenceStepEnables(SequenceStepEnables *enables)
{
    uint8_t sequence_config = _HlDrvVL53L0X_ReadReg(SYSTEM_SEQUENCE_CONFIG);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void _HlDrvVL53L0X_GetSequenceStepTimeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = HlDrvVL53L0X_GetVcselPulsePeriod(VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = _HlDrvVL53L0X_ReadReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        _HlDrvVL53L0X_TimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                                 timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
        _HlDrvVL53L0X_DecodeTimeout(_HlDrvVL53L0X_ReadReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        _HlDrvVL53L0X_TimeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                                 timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = HlDrvVL53L0X_GetVcselPulsePeriod(VcselPeriodFinalRange);

    timeouts->final_range_mclks =
        _HlDrvVL53L0X_DecodeTimeout(_HlDrvVL53L0X_ReadReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        _HlDrvVL53L0X_TimeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                                 timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t _HlDrvVL53L0X_DecodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t _HlDrvVL53L0X_EncodeTimeout(uint16_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t _HlDrvVL53L0X_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t _HlDrvVL53L0X_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
uint8_t _HlDrvVL53L0X_PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
    _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
    _HlDrvVL53L0X_WaitForInt();
    _HlDrvVL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    _HlDrvVL53L0X_WriteReg(SYSRANGE_START, 0x00);

    return 1;
}
#include <avr/cpufunc.h>
void _HlDrvVL53L0X_WaitForInt()
{
    do
        sleep_mode();
    while ((_HlDrvVL53L0X_ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0);
    // while(PIND & _BV(4))
    //   _NOP();
}

ISR(PCINT2_vect)
{
    return;
}