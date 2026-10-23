package mpu9250

import (
	"math"
	"sync/atomic"
	"testing"
	"time"

	"github.com/kidoman/embd"
)

func TestTempConversion(t *testing.T) {
	for _, tc := range []struct {
		name   string
		whoAmI byte
		raw    float64
		want   float64
	}{
		{"MPU-6050 at 36.53 degC", 0x68, 0, 36.53},
		{"MPU-6050 at 25 degC", 0x68, (25 - 36.53) * 340, 25},
		{"MPU-9250 at 21 degC", 0x71, 0, 21},
		{"MPU-9250 at 25 degC", 0x71, 4 * 333.87, 25},
		{"MPU-9255 at 25 degC", 0x73, 4 * 333.87, 25},
		{"MPU-6500 at 25 degC", 0x70, 4 * 333.87, 25},
		{"clone 0x75 at 25 degC", 0x75, 4 * 333.87, 25},
	} {
		scale, offset := tempConversion(tc.whoAmI)
		if got := tc.raw*scale + offset; math.Abs(got-tc.want) > 1e-9 {
			t.Errorf("%s: raw %.0f -> %.3f degC, want %.3f", tc.name, tc.raw, got, tc.want)
		}
	}
	// The bug this guards against: the MPU-6050 formula on an MPU-9250 at 25 degC.
	if s, o := tempConversion(0x68); math.Abs(4*333.87*s+o-25) < 10 {
		t.Error("the two families' conversions should differ by ~15 degC at 25 degC")
	}
}

// fakeBus is an I2C bus on which every register reads as regs[reg], or 0, and
// writes are dropped; reads counts the reads.  The rest of embd.I2CBus is left
// nil: the read loop does not use it.
type fakeBus struct {
	embd.I2CBus
	regs  map[byte]uint16
	reads int64
}

func (b *fakeBus) ReadByteFromReg(addr, reg byte) (byte, error) {
	atomic.AddInt64(&b.reads, 1)
	return byte(b.regs[reg]), nil
}
func (b *fakeBus) ReadWordFromReg(addr, reg byte) (uint16, error) {
	atomic.AddInt64(&b.reads, 1)
	return b.regs[reg], nil
}
func (b *fakeBus) WriteByteToReg(addr, reg, value byte) error { return nil }

// The averaged magnetometer is the mean of the magnetometer samples since the
// last read.  The gyro/accel tick used to add the last magnetometer sample to
// the sums too, without counting it, so a constant field averaged to about
// twice its value - 1.5 or 3 times in the windows where a gyro tick and its
// magnetometer tick fell on either side of the read.
func TestMagAverage(t *testing.T) {
	bus := &fakeBus{regs: map[byte]uint16{
		MPUREG_EXT_SENS_DATA_00: 256, // low byte 0: not flagged as an overrun
		MPUREG_EXT_SENS_DATA_02: 512,
		MPUREG_EXT_SENS_DATA_04: 768,
	}}
	mpu := &MPU9250{i2cbus: bus, sampleRate: 50, enableMag: true, mcal1: 1, mcal2: 1, mcal3: 1}
	mpu.mpuCalData.reset()
	go mpu.readSensors()
	time.Sleep(200 * time.Millisecond) // readSensors makes the channels; NewMPU9250 waits the same way
	<-mpu.CAvg

	windows := 0
	for i := 0; i < 20; i++ {
		time.Sleep(50 * time.Millisecond) // stratux polls at 20 Hz
		d := <-mpu.CAvg
		if d.NM == 0 {
			continue
		}
		windows++
		if d.M1 != 256 || d.M2 != 512 || d.M3 != 768 {
			t.Errorf("window %d: %d gyro and %d mag samples averaged to %.1f %.1f %.1f, want 256 512 768",
				i, d.N, d.NM, d.M1, d.M2, d.M3)
		}
	}
	if windows == 0 {
		t.Fatal("no window had a magnetometer sample")
	}
}

// CloseMPU stops the read goroutine: no bus traffic once it has returned, the
// channels close, and a second call returns.  The loop used to break out of its
// select only, so it went on polling the bus and serving CAvg after the close.
func TestCloseMPU(t *testing.T) {
	bus := &fakeBus{}
	mpu := &MPU9250{i2cbus: bus, sampleRate: 50, enableMag: true, mcal1: 1, mcal2: 1, mcal3: 1}
	mpu.mpuCalData.reset()
	go mpu.readSensors()
	time.Sleep(200 * time.Millisecond) // readSensors makes the channels; NewMPU9250 waits the same way
	<-mpu.CAvg

	mpu.CloseMPU()
	reads := atomic.LoadInt64(&bus.reads)
	select {
	case d, ok := <-mpu.CAvg:
		if ok {
			t.Fatalf("CAvg still delivers after CloseMPU: %d gyro samples", d.N)
		}
	case <-time.After(time.Second):
		t.Fatal("CAvg neither delivers nor closes after CloseMPU")
	}
	time.Sleep(100 * time.Millisecond) // five sample periods
	if n := atomic.LoadInt64(&bus.reads) - reads; n != 0 {
		t.Errorf("%d bus reads after CloseMPU returned", n)
	}

	done := make(chan struct{})
	go func() { mpu.CloseMPU(); close(done) }()
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("a second CloseMPU blocks")
	}
}
