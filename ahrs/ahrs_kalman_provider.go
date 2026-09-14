package ahrs

import "math"

// KalmanAHRS adapts KalmanState to AHRSProvider so that it can run wherever
// SimpleState runs today, fed the same Measurement and the same calibrations.
//
// What it does on top of the raw filter:
//
//   - Measurement.A is the reaction the sensor drivers report, +1 g on the up
//     axis at rest; the filter's measurement model predicts -1 g, so A is
//     negated here.  It is also scaled by |C| the way SimpleState does: the
//     C the daemon calibrates is the accelerometer reading at level, not the
//     bias the filter's C state is, so it goes into the scale and the bias
//     state starts at zero.
//   - The earth field N is pinned through SetConfig ("MagField" in uT and
//     "MagDip" in degrees, positive down); the heading is then magnetic.
//     Without it N is a free state and any heading error can be absorbed by
//     rotating N, so the magnetometer only stiffens heading between GPS
//     updates.  "MagBiasSigma" (uT) sets the prior on the mag bias L: leave
//     it at the default for a raw magnetometer, or a few uT when the hard
//     iron has been calibrated out, or the mag has nothing to say about
//     heading either (a 20 degree error is 8 uT).  "WindSigma" and
//     "WindNoise" (kt, kt per sqrt(s)) size the wind state; the crab angle
//     between magnetic heading and GPS track is what makes it observable.
//   - Initialisation happens on the first Compute and after Reset, or when
//     the measurements go stale (maxDT), like SimpleState.
type KalmanAHRS struct {
	KalmanState

	needsInit bool
	aNorm     float64
	f         *[4]float64 // sensor quaternion, applied at every init
	c, d      [3]float64  // calibrations as given, re-applied at every init

	magField, magDip float64 // 0 field: N stays free
	magBiasSigma     float64
	windSigma        float64 // 0: filter default
	windNoise        float64 // 0: filter default
	maxSigma         float64 // roll/pitch sigma above which Valid is false, rad

	// Aircraft-frame acceleration from the last measurement, for slip/skid and
	// g-load, smoothed like SimpleState smooths them.
	slipSkid, gLoad float64
	logMap          map[string]interface{}
}

var _ AHRSProvider = (*KalmanAHRS)(nil)

// NewKalmanAHRS returns a Kalman AHRS that initialises itself on the first
// Compute.
func NewKalmanAHRS() *KalmanAHRS {
	return &KalmanAHRS{
		needsInit:    true,
		aNorm:        1,
		magBiasSigma: 10, // KalmanState.init's own prior
		maxSigma:     5 * Deg,
		logMap:       make(map[string]interface{}),
	}
}

// adjust returns the measurement as the filter expects it: acceleration
// negated and scaled, see the type comment.  The copy shares the variance
// accumulators and noise matrix with m, which is what Update wants.
func (s *KalmanAHRS) adjust(m *Measurement) *Measurement {
	mm := *m
	mm.A1, mm.A2, mm.A3 = -m.A1/s.aNorm, -m.A2/s.aNorm, -m.A3/s.aNorm
	return &mm
}

// initialise starts the filter from m and re-applies everything the caller
// has configured, since KalmanState.init overwrites the state.
func (s *KalmanAHRS) initialise(m *Measurement) {
	s.KalmanState = KalmanState{}
	s.init(m)
	s.T = m.T // init leaves it zero, which would read as stale at once
	if s.f != nil {
		s.F0, s.F1, s.F2, s.F3 = s.f[0], s.f[1], s.f[2], s.f[3]
		s.normalize()
	}
	s.D1, s.D2, s.D3 = s.d[0], s.d[1], s.d[2]
	// The bias state starts at zero: the calibration went into aNorm.
	s.C1, s.C2, s.C3 = 0, 0, 0

	if s.magField > 0 {
		// Earth frame is east, north, up: the field points to magnetic north
		// and, in the northern hemisphere, down.
		s.N1 = 0
		s.N2 = s.magField * math.Cos(s.magDip*Deg)
		s.N3 = -s.magField * math.Sin(s.magDip*Deg)
		for i := 13; i <= 15; i++ {
			s.M.Set(i, i, 0.01)
			s.N.Set(i, i, 1e-6)
		}
	}
	for i := 29; i <= 31; i++ {
		s.M.Set(i, i, s.magBiasSigma*s.magBiasSigma)
	}
	if s.windSigma > 0 {
		for i := 16; i <= 18; i++ {
			s.M.Set(i, i, s.windSigma*s.windSigma)
		}
	}
	if s.windNoise > 0 {
		for i := 16; i <= 18; i++ {
			s.N.Set(i, i, s.windNoise*s.windNoise)
		}
	}
	s.needsInit = false
}

// Compute runs a predict/update cycle, initialising first when needed.
func (s *KalmanAHRS) Compute(m *Measurement) {
	mm := s.adjust(m)
	if s.needsInit || mm.T-s.T > maxDT || mm.T-s.T < 0 {
		s.initialise(mm)
		return
	}
	s.KalmanState.Compute(mm)

	// Slip/skid and g-load from the measured acceleration in the aircraft
	// frame, as SimpleState reports them: the reaction, so -A.
	_, a2, a3 := s.rotateByF(-mm.A1, -mm.A2, -mm.A3)
	s.slipSkid += slowSmoothConst * (math.Atan2(a2, -a3) - s.slipSkid)
	s.gLoad += slowSmoothConst * (-a3 - s.gLoad)
	s.updateLogMap(mm)
}

// Valid reports whether the attitude is usable: the filter is initialised,
// the state is within physical bounds, and roll and pitch are known to within
// "MaxAttitudeSigma" degrees (SetConfig; default 5).  Without a magnetometer
// roll is only weakly observable and its sigma sits around 8 degrees in
// cruise, with the field pinned around 2.
func (s *KalmanAHRS) Valid() bool {
	if s.needsInit || s.M == nil {
		return false
	}
	if s.U1 < -5 || math.Abs(s.U1) > 300 || math.Abs(s.U2) > 20 || math.Abs(s.U3) > 20 ||
		math.Abs(s.V1) > 40 || math.Abs(s.V2) > 40 || math.Abs(s.V3) > 40 {
		return false
	}
	droll, dpitch, _ := s.CalcRollPitchHeadingUncertainty()
	return droll <= s.maxSigma && dpitch <= s.maxSigma
}

// Reset restarts the filter on the next Compute.
func (s *KalmanAHRS) Reset() {
	s.needsInit = true
}

// RollPitchHeading returns the attitude in radians.  The heading is magnetic
// when the field is pinned, otherwise it is whatever GPS track and gyro make
// of it.
func (s *KalmanAHRS) RollPitchHeading() (roll, pitch, heading float64) {
	return FromQuaternion(s.E0, s.E1, s.E2, s.E3)
}

// MagHeading returns the magnetic heading in degrees, or Invalid when the
// field is not pinned and so the filter's heading is not referenced to it.
func (s *KalmanAHRS) MagHeading() float64 {
	if s.magField <= 0 {
		return Invalid
	}
	_, _, heading := s.RollPitchHeading()
	return heading / Deg
}

// SlipSkid returns the slip/skid angle in degrees.
func (s *KalmanAHRS) SlipSkid() float64 {
	return s.slipSkid / Deg
}

// RateOfTurn returns the turn rate in degrees per second: the yaw rate in the
// earth frame, which the filter estimates directly.
func (s *KalmanAHRS) RateOfTurn() float64 {
	return -s.H3
}

// GLoad returns the load factor in g.
func (s *KalmanAHRS) GLoad() float64 {
	return s.gLoad
}

// SetSensorQuaternion sets the sensor-to-aircraft rotation F.
func (s *KalmanAHRS) SetSensorQuaternion(f *[4]float64) {
	ff := *f
	s.f = &ff
	if !s.needsInit {
		s.F0, s.F1, s.F2, s.F3 = ff[0], ff[1], ff[2], ff[3]
		s.normalize()
	}
}

// GetSensorQuaternion returns the sensor-to-aircraft rotation F.
func (s *KalmanAHRS) GetSensorQuaternion() *[4]float64 {
	return &[4]float64{s.F0, s.F1, s.F2, s.F3}
}

// SetCalibrations takes the accelerometer reading at level c, whose norm
// becomes the accelerometer scale, and the gyro zero-rate d.
func (s *KalmanAHRS) SetCalibrations(c, d *[3]float64) {
	if c != nil {
		s.c = *c
		if n := math.Sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]); n > 0.5 {
			s.aNorm = n
		} else {
			s.aNorm = 1
		}
	}
	if d != nil {
		s.d = *d
		if !s.needsInit {
			s.D1, s.D2, s.D3 = d[0], d[1], d[2]
		}
	}
}

// GetCalibrations returns the calibrations as given to SetCalibrations.
func (s *KalmanAHRS) GetCalibrations() (c, d *[3]float64) {
	cc, dd := s.c, s.d
	return &cc, &dd
}

// SetConfig sets the local magnetic field and the priors described in the
// type comment.  A change takes effect at the next initialisation; call Reset
// to force one.
func (s *KalmanAHRS) SetConfig(configMap map[string]float64) {
	if v, ok := configMap["MagField"]; ok {
		s.magField = v
	}
	if v, ok := configMap["MagDip"]; ok {
		s.magDip = v
	}
	if v, ok := configMap["MagBiasSigma"]; ok && v > 0 {
		s.magBiasSigma = v
	}
	if v, ok := configMap["WindSigma"]; ok {
		s.windSigma = v
	}
	if v, ok := configMap["WindNoise"]; ok {
		s.windNoise = v
	}
	if v, ok := configMap["MaxAttitudeSigma"]; ok && v > 0 {
		s.maxSigma = v * Deg
	}
}

// GetState returns the filter state.
func (s *KalmanAHRS) GetState() *State {
	return &s.State
}

// GetLogMap returns the values logged for analysis.
func (s *KalmanAHRS) GetLogMap() map[string]interface{} {
	return s.logMap
}

func (s *KalmanAHRS) updateLogMap(m *Measurement) {
	roll, pitch, heading := s.RollPitchHeading()
	droll, dpitch, dheading := s.CalcRollPitchHeadingUncertainty()
	p := s.logMap
	p["T"] = m.T
	p["Roll"] = roll / Deg
	p["Pitch"] = pitch / Deg
	p["Heading"] = heading / Deg
	p["dRoll"] = droll / Deg
	p["dPitch"] = dpitch / Deg
	p["dHeading"] = dheading / Deg
	p["U1"] = s.U1
	p["V1"] = s.V1
	p["V2"] = s.V2
	p["N1"] = s.N1
	p["N2"] = s.N2
	p["N3"] = s.N3
	p["D1"] = s.D1
	p["D2"] = s.D2
	p["D3"] = s.D3
	p["L1"] = s.L1
	p["L2"] = s.L2
	p["L3"] = s.L3
	p["W1"] = m.W1
	p["W2"] = m.W2
	p["A1"] = m.A1
	p["A2"] = m.A2
	p["A3"] = m.A3
	p["B1"] = m.B1
	p["B2"] = m.B2
	p["B3"] = m.B3
	p["M1"] = m.M1
	p["M2"] = m.M2
	p["M3"] = m.M3
	if m.WValid {
		p["WValid"] = 1.0
	} else {
		p["WValid"] = 0.0
	}
}
