package ahrs

import (
	"math"
	"testing"
)

// mountF returns the rotation matrix f a sensor quaternion stands for, as
// calcRotationMatrices builds it.
func mountF(q [4]float64) (s State) {
	s.E0 = 1
	s.F0, s.F1, s.F2, s.F3 = q[0], q[1], q[2], q[3]
	s.normalize()
	return s
}

// atRest returns what an IMU driver reports for an aircraft standing still at
// roll, pitch (rad), the sensor mounted by q, scaled by aNorm: the reaction
// to gravity, +aNorm g on the up axis, in the chip frame.  q is the daemon's
// SensorQuaternion, i.e. what SimpleState takes: its f carries a sensor-frame
// vector into the aircraft frame, so the chip sees the transpose of it.
func atRest(q [4]float64, roll, pitch, aNorm float64) (a [3]float64) {
	var e State
	e.F0 = 1
	e.E0, e.E1, e.E2, e.E3 = ToQuaternion(roll, pitch, 0)
	e.normalize()
	up := [3]float64{e.e31, e.e32, e.e33}
	f := mountF(q)
	a[0] = aNorm * (f.f11*up[0] + f.f21*up[1] + f.f31*up[2])
	a[1] = aNorm * (f.f12*up[0] + f.f22*up[1] + f.f32*up[2])
	a[2] = aNorm * (f.f13*up[0] + f.f23*up[1] + f.f33*up[2])
	return a
}

// TestKalmanAHRSAttitudeAtRest is the contract between the daemon and the
// Kalman AHRS: fed the drivers' accelerometer (sign and scale as they report
// it) with the sensor quaternion and calibrations the daemon hands
// SimpleState, KalmanAHRS settles on the attitude SimpleState reports.  The
// unit standing still with no GPS fix is the hangar case.
func TestKalmanAHRSAttitudeAtRest(t *testing.T) {
	const (
		dt    = 0.05 // the daemon's 20 Hz loop
		secs  = 60.0
		tolSi = 0.5 // deg, SimpleState against the truth (checks the test data)
		tolKa = 1.0 // deg, KalmanAHRS against the truth
	)
	tilt := func(deg float64, axis [3]float64) [4]float64 {
		h := deg * Deg / 2
		return [4]float64{math.Cos(h), axis[0] * math.Sin(h), axis[1] * math.Sin(h), axis[2] * math.Sin(h)}
	}
	cases := []struct {
		name        string
		q           [4]float64
		roll, pitch float64 // deg
		aNorm       float64
	}{
		{"level, chip aligned", [4]float64{1, 0, 0, 0}, 0, 0, 1},
		{"hangar attitude, chip aligned", [4]float64{1, 0, 0, 0}, 0, 9, 1},
		{"chip tilted 17 deg about the wing", tilt(17, [3]float64{0, 1, 0}), 0, 9, 1},
		{"chip tilted 17 deg about the nose", tilt(17, [3]float64{1, 0, 0}), 3, 9, 1},
		// Near a half turn, so nearly its own conjugate: it cannot tell the
		// two directions of F apart, the tilted mounts above can.
		{"I-RVDX sensor quaternion", [4]float64{0.0008540788764084712, 0.14747589890389523, -0.005727889129517793, 0.989048695022907}, 0, 9, 1},
		{"chip yawed 90 and tilted 20", [4]float64{0.6963642, 0.1227878, 0.1227878, 0.6963642}, -5, 4, 1},
		{"accelerometer reading 0.877 g", tilt(17, [3]float64{0, 1, 0}), 0, 9, 0.877},
	}
	for _, c := range cases {
		roll, pitch := c.roll*Deg, c.pitch*Deg
		a := atRest(c.q, roll, pitch, c.aNorm)
		// The daemon's C is the accelerometer at level (cage).
		cal := atRest(c.q, 0, 0, c.aNorm)
		d := [3]float64{0, 0, 0}

		simple := NewSimpleAHRS()
		simple.SetCalibrations(&cal, &d)
		simple.SetSensorQuaternion(&c.q)
		kalman := NewKalmanAHRS()
		kalman.SetCalibrations(&cal, &d)
		kalman.SetSensorQuaternion(&c.q)

		sm, km := NewMeasurement(), NewMeasurement()
		for i := 0; float64(i)*dt < secs; i++ {
			for _, m := range []*Measurement{sm, km} {
				m.T = 1000 + float64(i)*dt
				m.SValid = true
				m.A1, m.A2, m.A3 = a[0], a[1], a[2]
				m.B1, m.B2, m.B3 = 0, 0, 0
			}
			simple.Compute(sm)
			kalman.Compute(km)
		}

		sr, sp, _ := simple.RollPitchHeading()
		kr, kp, _ := kalman.RollPitchHeading()
		kdr, kdp, _ := kalman.CalcRollPitchHeadingUncertainty()
		t.Logf("%s: truth %.1f/%.1f  simple %.2f/%.2f  kalman %.2f/%.2f (sigma %.2f/%.2f, valid %v)",
			c.name, c.roll, c.pitch, sr/Deg, sp/Deg, kr/Deg, kp/Deg, kdr/Deg, kdp/Deg, kalman.Valid())
		if math.Abs(sr/Deg-c.roll) > tolSi || math.Abs(sp/Deg-c.pitch) > tolSi {
			t.Errorf("%s: SimpleState roll/pitch %.2f/%.2f, want %.1f/%.1f: test data is not what SimpleState expects",
				c.name, sr/Deg, sp/Deg, c.roll, c.pitch)
			continue
		}
		if math.Abs(kr/Deg-c.roll) > tolKa || math.Abs(kp/Deg-c.pitch) > tolKa {
			t.Errorf("%s: KalmanAHRS roll/pitch %.2f/%.2f, want %.1f/%.1f as SimpleState",
				c.name, kr/Deg, kp/Deg, c.roll, c.pitch)
		}
		if math.Abs(kalman.GLoad()-simple.GLoad()) > 0.01 || math.Abs(kalman.SlipSkid()-simple.SlipSkid()) > 0.5 {
			t.Errorf("%s: KalmanAHRS g-load %.3f slip/skid %.2f, want %.3f %.2f as SimpleState",
				c.name, kalman.GLoad(), kalman.SlipSkid(), simple.GLoad(), simple.SlipSkid())
		}
		// The quaternion comes back as it was given, up to the filter's own
		// refinement of it and the sign a quaternion is free to flip.
		g := kalman.GetSensorQuaternion()
		n := math.Sqrt(c.q[0]*c.q[0] + c.q[1]*c.q[1] + c.q[2]*c.q[2] + c.q[3]*c.q[3])
		dot := (g[0]*c.q[0] + g[1]*c.q[1] + g[2]*c.q[2] + g[3]*c.q[3]) / n
		if math.Abs(dot) < 0.9999 {
			t.Errorf("%s: GetSensorQuaternion %v, want %v", c.name, *g, c.q)
		}
	}
}
