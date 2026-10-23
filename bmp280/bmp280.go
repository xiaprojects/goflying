/*
Reference 1: https://github.com/BoschSensortec/BMP280_driver
Reference 2: https://forums.adafruit.com/viewtopic.php?f=19&t=89049
*/

package bmp280

import (
	"fmt"
	"log"
	"math"
	"time"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"
)

const (
	// I2C Address Definitions
	Address1 = 0x76
	Address2 = 0x77
	// Chip ID Definitions
	ChipID1 = 0x56
	ChipID2 = 0x57
	ChipID3 = 0x58 // BMP280 (pressure + temperature)
	ChipID4 = 0x60 // BME280 (pressure + temperature + humidity)
	// ChipIDBME280 is an alias for ChipID4, used where humidity support matters.
	ChipIDBME280 = ChipID4
	// Power Mode Definitions
	SleepMode     = 0x00
	ForcedMode    = 0x01
	NormalMode    = 0x03
	SoftResetCode = 0xB6
	// Standby Time Definitions
	StandbyTime1ms    = 0x00
	StandbyTime63ms   = 0X01
	StandbyTime125ms  = 0x02
	StandbyTime250ms  = 0x03
	StandbyTime500ms  = 0x04
	StandbyTime1000ms = 0x05
	StandbyTime2000ms = 0x06
	StandbyTime4000ms = 0x07
	// Filter Definitions
	FilterCoeffOff = 0x00
	FilterCoeff2   = 0x01
	FilterCoeff4   = 0x02
	FilterCoeff8   = 0x03
	FilterCoeff16  = 0x04
	// Oversampling Definitions
	OversampSkipped = 0x00
	Oversamp1x      = 0x01
	Oversamp2x      = 0x02
	Oversamp4x      = 0x03
	Oversamp8x      = 0x04
	Oversamp16x     = 0x05
	// Working Mode Definitions
	UltraLowPowerMode       = 0X00
	LowPowerMode            = 0X01
	StandardResolutionMode  = 0X02
	HighResolutionMode      = 0X03
	UltraHighResolutionMode = 0x04

	// BMP280 registers
	RegisterCompData      = 0x88
	RegisterChipID        = 0xD0
	RegisterSoftReset     = 0xE0
	RegisterStatus        = 0xF3
	RegisterControl       = 0xF4
	RegisterConfig        = 0xF5
	RegisterPressDataMSB  = 0xF7
	RegisterPressDataLSB  = 0xF8
	RegisterPressDataXLSB = 0xF9
	RegisterTempDataMSB   = 0xFA
	RegisterTempDataLSB   = 0xFB
	RegisterTempDataXLSB  = 0xFC

	// BME280-only registers (chip ID 0x60): humidity control, calibration and data
	RegisterCtrlHum      = 0xF2 // osrs_h[2:0]; must be written before RegisterControl to take effect
	RegisterHumCompData1 = 0xA1 // dig_H1 (1 byte)
	RegisterHumCompData2 = 0xE1 // dig_H2..dig_H6 (7 bytes)
	RegisterHumDataMSB   = 0xFD
	RegisterHumDataLSB   = 0xFE

	// Connection retries definition
	ConnAttempts = 5

	// Sea level reference pressure in hPa
	QNH = 1013.25

	// Buffer size for reading data from BMP
	BufSize = 256

	// Read delay between chip reading attempts
	ReadDelay = time.Millisecond
)

type BMPData struct {
	Temperature float64       // Temperature, degrees C
	Pressure    float64       // Barometric pressure, hPa
	Humidity    float64       // Relative humidity, percent; 0 if the chip has no humidity sensor (plain BMP280)
	T           time.Duration // Time since the driver was started
}

type BMP280 struct {
	i2cbus *embd.I2CBus

	address     byte
	chipID      byte
	config      byte
	control     byte
	humControl  byte // ctrl_hum value (BME280 only)
	hasHumidity bool // true when chipID == ChipIDBME280

	t time.Time

	Delay time.Duration

	DigT map[int]int32
	DigP map[int]int64
	DigH map[int]int32 // Humidity compensation words dig_H1..dig_H6 (BME280 only)

	T_fine int32

	C      <-chan *BMPData
	CBuf   <-chan *BMPData
	cClose chan bool
}

/*
NewBMP280 returns a BMP280 object with the chosen settings
address is one of bmp280.Address1 or bmp280.Address2
powerMode is one of bmp280.SleepMode, bmp280.ForcedMode, or bmp280.NormalMode
standby is one of the bmp280.StandbyTimeX
filter is one of bmp280.FilterCoeffX
tempRes is one of bmp280.OversampX
presRes is one of bmp280.XMode
*/
func NewBMP280(i2cbus *embd.I2CBus, address, powerMode, standby, filter, tempRes, presRes byte) (bmp *BMP280, err error) {
	bmp = new(BMP280)
	bmp.i2cbus = i2cbus
	bmp.address = address

	// Make sure we can connect to the chip and read a valid ChipID
	v := make([]byte, 1)
	for n := 0; n < ConnAttempts; n++ {
		if errv := bmp.i2cReadBytes(RegisterChipID, v); errv != nil {
			time.Sleep(ReadDelay)
			err = fmt.Errorf("BMP280: couldn't find chip at address %x: %s", address, errv)
			continue
		}
		if v[0] == ChipID1 || v[0] == ChipID2 || v[0] == ChipID3 || v[0] == ChipID4 {
			err = nil
			break
		}
		time.Sleep(ReadDelay)
		err = fmt.Errorf("BMP280: Wrong ChipID, got %x", v)
	}
	if err != nil {
		return nil, err
	}

	bmp.chipID = v[0]
	bmp.hasHumidity = bmp.chipID == ChipIDBME280

	bmp.config = (standby << 5) + (filter << 2)               // combine bits for config
	bmp.control = (tempRes << 5) + (presRes << 2) + powerMode // combine bits for control

	bmp.t = time.Now()
	bmp.Delay = delayFromStandby(standby)

	bmp.i2cWrite(RegisterSoftReset, SoftResetCode) // reset sensor
	time.Sleep(ReadDelay)

	if bmp.hasHumidity {
		// On the BME280 the humidity oversampling (ctrl_hum) must be written
		// before ctrl_meas (RegisterControl) or the change is ignored.
		// Humidity changes slowly, so 1x oversampling is plenty.
		bmp.humControl = Oversamp1x
		bmp.i2cWrite(RegisterCtrlHum, bmp.humControl)
		time.Sleep(ReadDelay)
	}

	bmp.i2cWrite(RegisterControl, bmp.control) //
	time.Sleep(ReadDelay)
	bmp.i2cWrite(RegisterConfig, bmp.config) //
	time.Sleep(ReadDelay)

	bmp.DigT = make(map[int]int32)
	bmp.DigP = make(map[int]int64)
	bmp.DigH = make(map[int]int32)
	bmp.ReadCorrectionSettings()

	go bmp.readSensor()
	time.Sleep(ReadDelay)
	return
}

// Close closes the BMP280
func (bmp *BMP280) Close() {
	bmp.SetPowerMode(SleepMode)
	bmp.cClose <- true
}

func delayFromStandby(standby byte) (delay time.Duration) {
	if standby == 0 {
		delay = 500 * time.Microsecond
	} else if standby == 1 {
		delay = 62500 * time.Microsecond
	} else {
		delay = time.Duration(int(4000)>>uint(7-standby)) * time.Millisecond
	}
	return
}

func (bmp *BMP280) ReadCorrectionSettings() (err error) {
	var raw []byte = make([]byte, 24)

	errf := bmp.i2cReadBytes(RegisterCompData, raw)
	if errf != nil {
		err = fmt.Errorf("BMP280: Error reading calibration: %s", errf)
	}

	bmp.DigT[1] = int32(raw[1])<<8 + int32(raw[0])
	for i := 1; i < 3; i++ {
		bmp.DigT[i+1] = int32(int16(raw[2*i+1])<<8 + int16(raw[2*i]))
	}

	bmp.DigP[1] = int64(raw[7])<<8 + int64(raw[6])
	for i := 1; i < 9; i++ {
		bmp.DigP[i+1] = int64(int16(raw[2*i+7])<<8 + int16(raw[2*i+6]))
	}

	if bmp.hasHumidity {
		if errh := bmp.readHumidityCorrectionSettings(); errh != nil && err == nil {
			err = errh
		}
	}

	return
}

// readHumidityCorrectionSettings loads the BME280 humidity compensation words
// dig_H1..dig_H6. dig_H1 lives on its own at 0xA1; dig_H2..dig_H6 are packed
// into the 7 bytes starting at 0xE1 (BME280 datasheet Table 16).
func (bmp *BMP280) readHumidityCorrectionSettings() (err error) {
	h1 := make([]byte, 1)
	if errf := bmp.i2cReadBytes(RegisterHumCompData1, h1); errf != nil {
		return fmt.Errorf("BMP280: Error reading humidity calibration (dig_H1): %s", errf)
	}
	e := make([]byte, 7)
	if errf := bmp.i2cReadBytes(RegisterHumCompData2, e); errf != nil {
		return fmt.Errorf("BMP280: Error reading humidity calibration (dig_H2..dig_H6): %s", errf)
	}

	bmp.DigH[1] = int32(h1[0])                                 // unsigned
	bmp.DigH[2] = int32(int16(uint16(e[1])<<8 | uint16(e[0]))) // signed, little-endian
	bmp.DigH[3] = int32(e[2])                                  // unsigned
	bmp.DigH[4] = int32(int8(e[3]))<<4 | int32(e[4]&0x0F)      // signed, 12-bit packed
	bmp.DigH[5] = int32(int8(e[5]))<<4 | int32(e[4]>>4)        // signed, 12-bit packed
	bmp.DigH[6] = int32(int8(e[6]))                            // signed

	return
}

func (bmp *BMP280) readSensor() {
	var (
		raw_temp         int32
		raw_press        int64
		raw_hum          int32
		temp, press, hum float64
		err              error
		t                time.Time
	)

	raw := make([]byte, 6)
	if bmp.hasHumidity {
		raw = make([]byte, 8) // press[3] + temp[3] + hum[2], contiguous from 0xF7
	}

	cC := make(chan *BMPData)
	defer close(cC)
	bmp.C = cC
	cBuf := make(chan *BMPData, BufSize)
	defer close(cBuf)
	bmp.CBuf = cBuf
	bmp.cClose = make(chan bool)
	defer close(bmp.cClose)

	clock := time.NewTicker(bmp.Delay)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	makeBMPData := func() *BMPData {
		d := BMPData{
			Temperature: temp,
			Pressure:    press,
			Humidity:    hum,
			T:           t.Sub(bmp.t),
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read sensor data:
			err = bmp.i2cReadBytes(RegisterPressDataMSB, raw)
			if err != nil {
				log.Printf("BMP280 Warning: error reading sensor data: %s", err)
				continue
			}

			raw_temp = (int32(raw[3]) << 12) + (int32(raw[4]) << 4) + (int32(raw[5]) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			raw_press = (int64(raw[0]) << 12) + (int64(raw[1]) << 4) + (int64(raw[2]) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			temp = bmp.CalcCompensatedTemp(raw_temp)
			press = bmp.CalcCompensatedPress(raw_press)

			if bmp.hasHumidity {
				raw_hum = (int32(raw[6]) << 8) + int32(raw[7])
				hum = bmp.CalcCompensatedHumidity(raw_hum)
			}
		case cC <- makeBMPData(): // Send the latest values
		case cBuf <- makeBMPData():
		case <-bmp.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

func (bmp *BMP280) CalcCompensatedTemp(raw_temp int32) (temp float64) {
	var var1, var2, t int32

	var1 = (((raw_temp >> 3) - (bmp.DigT[1] << 1)) * bmp.DigT[2]) >> 11
	var2 = (((((raw_temp >> 4) - bmp.DigT[1]) * ((raw_temp >> 4) - bmp.DigT[1])) >> 12) * bmp.DigT[3]) >> 14
	bmp.T_fine = var1 + var2
	t = (bmp.T_fine*5 + 128) >> 8
	temp = float64(t) / 100 // Temperature in degC
	return
}

func (bmp *BMP280) CalcCompensatedPress(raw_press int64) (press float64) {
	var var1, var2, p int64

	var1 = int64(bmp.T_fine) - 128000
	var2 = var1 * var1 * bmp.DigP[6]
	var2 += (var1 * bmp.DigP[5]) << 17
	var2 += bmp.DigP[4] << 35
	var1 = ((var1 * var1 * bmp.DigP[3]) >> 8) + ((var1 * bmp.DigP[2]) << 12)
	var1 = ((int64(1) << 47) + var1) * bmp.DigP[1] >> 33
	if var1 == 0 {
		return 0
	}
	p = 1048576 - raw_press
	p = (((p << 31) - var2) * 3125) / var1
	var1 = (bmp.DigP[9] * (p >> 13) * (p >> 13)) >> 25
	var2 = (bmp.DigP[8] * p) >> 19
	p = ((p + var1 + var2) >> 8) + (bmp.DigP[7] << 4)
	press = float64(p) / 25600
	return
}

// CalcCompensatedHumidity converts a raw BME280 humidity reading into percent
// relative humidity, using the floating-point compensation formula from the
// BME280 datasheet (section 8.1). It relies on T_fine, so CalcCompensatedTemp
// must have been run for the same sample first. Only meaningful when the
// connected chip is a BME280 (HasHumiditySensor returns true).
func (bmp *BMP280) CalcCompensatedHumidity(raw_hum int32) (humidity float64) {
	h := float64(bmp.T_fine) - 76800.0
	h = (float64(raw_hum) - (float64(bmp.DigH[4])*64.0 + float64(bmp.DigH[5])/16384.0*h)) *
		(float64(bmp.DigH[2]) / 65536.0 * (1.0 + float64(bmp.DigH[6])/67108864.0*h*
			(1.0+float64(bmp.DigH[3])/67108864.0*h)))
	h = h * (1.0 - float64(bmp.DigH[1])*h/524288.0)
	if h > 100.0 {
		h = 100.0
	} else if h < 0.0 {
		h = 0.0
	}
	return h
}

// HasHumiditySensor reports whether the connected chip is a BME280 and therefore
// fills in BMPData.Humidity. A plain BMP280 returns false and Humidity stays 0.
func (bmp *BMP280) HasHumiditySensor() bool {
	return bmp.hasHumidity
}

func CalcAltitude(press float64) (altitude float64) {
	altitude = 145366.45 * (1.0 - math.Pow(press/QNH, 0.190284))
	return
}

func (bmp *BMP280) GetPowerMode() (powerMode byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read power mode: %s", errv)
		return
	}
	powerMode = v[0] & 0x03
	return
}

func (bmp *BMP280) SetPowerMode(powerMode byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't read power mode: %s", errv)
	}
	v[0] = (v[0] & 0xfc) | powerMode

	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write power mode: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetOversampPress() (oversampPres byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read Pressure Oversampling: %s", errv)
		return
	}
	oversampPres = (v[0] & 0x1c) >> 2
	return
}

func (bmp *BMP280) SetOversampPress(oversampPres byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't read Pressure Oversampling: %s", errv)
	}
	v[0] = (v[0] & 0xe3) | (oversampPres << 2)

	time.Sleep(bmp.Delay)
	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write Pressure Oversampling: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetOversampTemp() (oversampTemp byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read Temperature Oversampling: %s", errv)
		return
	}
	oversampTemp = (v[0] & 0xe0) >> 5
	return
}

func (bmp *BMP280) SetOversampTemp(oversampTemp byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't read Temperature Oversampling: %s", errv)
	}
	v[0] = (v[0] & 0x1f) | (oversampTemp << 5)

	time.Sleep(bmp.Delay)
	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write Temperature Oversampling: %s", errv)
	}
	return nil
}

// GetOversampHum returns the current humidity oversampling setting (BME280 only).
func (bmp *BMP280) GetOversampHum() (oversampHum byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterCtrlHum, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read Humidity Oversampling: %s", errv)
		return
	}
	oversampHum = v[0] & 0x07
	return
}

// SetOversampHum sets the humidity oversampling (BME280 only). Per the datasheet
// a write to ctrl_meas is required afterwards for the change to take effect.
func (bmp *BMP280) SetOversampHum(oversampHum byte) error {
	if errv := bmp.i2cWrite(RegisterCtrlHum, oversampHum&0x07); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write Humidity Oversampling: %s", errv)
	}
	bmp.humControl = oversampHum & 0x07
	if errv := bmp.i2cWrite(RegisterControl, bmp.control); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't apply Humidity Oversampling: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetFilterCoeff() (filterCoeff byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read Filter Coefficient: %s", errv)
		return
	}
	filterCoeff = (v[0] & 0x1c) >> 2
	return
}

func (bmp *BMP280) SetFilterCoeff(filterCoeff byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't read Filter Coefficient: %s", errv)
	}
	v[0] = (v[0] & 0xe3) | (filterCoeff << 2)

	if errv := bmp.i2cWrite(RegisterConfig, v[0]); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write Filter Coefficient: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetStandbyTime() (standbyTime byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't read Standby Time: %s", errv)
		return
	}
	standbyTime = (v[0] & 0xe0) >> 5
	return
}

func (bmp *BMP280) SetStandbyTime(standbyTime byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't read Standby Time: %s", errv)
	}
	v[0] = (v[0] & 0x1f) | (standbyTime << 5)

	if errv := bmp.i2cWrite(RegisterConfig, v[0]); errv != nil {
		return fmt.Errorf("BMP280 Error: Couldn't write Standby Time: %s", errv)
	}
	bmp.Delay = delayFromStandby(standbyTime)
	return nil
}

func (bmp *BMP280) i2cWrite(register, value byte) (err error) {
	if errWrite := (*bmp.i2cbus).WriteByteToReg(bmp.address, register, value); errWrite != nil {
		err = fmt.Errorf("BMP280 Error writing %X to %X: %s\n",
			value, register, errWrite)
	} else {
		time.Sleep(ReadDelay)
	}
	return
}

func (bmp *BMP280) i2cReadBytes(register byte, value []byte) (err error) {
	errRead := (*bmp.i2cbus).ReadFromReg(bmp.address, register, value)
	if errRead != nil {
		err = fmt.Errorf("BMP280 Error reading from %X: %s", register, err)
	}
	return
}
