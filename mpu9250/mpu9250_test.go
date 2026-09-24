package mpu9250

import (
	"math"
	"testing"
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
