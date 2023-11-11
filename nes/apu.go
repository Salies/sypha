package nes

const frameCounterRate = CPUFrequency / 240.0

var lengthTable = []byte{
	10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
	12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
}

var dutyTable = [][]byte{
	{0, 1, 0, 0, 0, 0, 0, 0},
	{0, 1, 1, 0, 0, 0, 0, 0},
	{0, 1, 1, 1, 1, 0, 0, 0},
	{1, 0, 0, 1, 1, 1, 1, 1},
}

var triangleTable = []byte{
	15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
}

var noiseTable = []uint16{
	4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
}

var dmcTable = []byte{
	214, 190, 170, 160, 143, 127, 113, 107, 95, 80, 71, 64, 53, 42, 36, 27,
}

var pulseTable [31]float32
var tndTable [203]float32

func init() {
	for i := 0; i < 31; i++ {
		pulseTable[i] = 95.52 / (8128.0/float32(i) + 100)
	}
	for i := 0; i < 203; i++ {
		tndTable[i] = 163.67 / (24329.0/float32(i) + 100)
	}
}

// APU

type APU struct {
	console     *Console
	channel     chan float32
	sampleRate  float64
	pulse1      Pulse
	pulse2      Pulse
	triangle    Triangle
	noise       Noise
	dmc         DMC
	cycle       uint64
	framePeriod byte
	frameValue  byte
	frameIRQ    bool
	filterChain FilterChain
}

func NewAPU(console *Console) *APU {
	apu := APU{}
	apu.console = console
	apu.noise.shiftRegister = 1
	apu.pulse1.channel = 1
	apu.pulse2.channel = 2
	apu.framePeriod = 4
	apu.dmc.cpu = console.CPU
	return &apu
}

func (apu *APU) Step() {
	cycle1 := apu.cycle
	apu.cycle++
	cycle2 := apu.cycle
	apu.stepTimer()
	f1 := int(float64(cycle1) / frameCounterRate)
	f2 := int(float64(cycle2) / frameCounterRate)
	if f1 != f2 {
		apu.stepFrameCounter()
	}
	s1 := int(float64(cycle1) / apu.sampleRate)
	s2 := int(float64(cycle2) / apu.sampleRate)
	if s1 != s2 {
		apu.sendSample()
	}
}

func (apu *APU) sendSample() {
	output := apu.filterChain.Step(apu.output())
	select {
	case apu.channel <- output:
	default:
	}
}

func (apu *APU) output() float32 {
	p1 := apu.pulse1.output()
	p2 := apu.pulse2.output()
	t := apu.triangle.output()
	n := apu.noise.output()
	d := apu.dmc.output()
	pulseOut := pulseTable[p1+p2]
	tndOut := tndTable[3*t+2*n+d]
	return pulseOut + tndOut
}

// mode 0:    mode 1:       function
// ---------  -----------  -----------------------------
//   - - - f    - - - - -    IRQ (if bit 6 is clear)
//   - l - l    l - l - -    Length counter and sweep
//     e e e e    e e e e -    Envelope and linear counter
func (apu *APU) stepFrameCounter() {
	switch apu.framePeriod {
	case 4:
		apu.frameValue = (apu.frameValue + 1) % 4
		switch apu.frameValue {
		case 0, 2:
			apu.stepEnvelope()
		case 1:
			apu.stepEnvelope()
			apu.stepSweep()
			apu.stepLength()
		case 3:
			apu.stepEnvelope()
			apu.stepSweep()
			apu.stepLength()
			apu.fireIRQ()
		}
	case 5:
		apu.frameValue = (apu.frameValue + 1) % 5
		switch apu.frameValue {
		case 0, 2:
			apu.stepEnvelope()
		case 1, 3:
			apu.stepEnvelope()
			apu.stepSweep()
			apu.stepLength()
		}
	}
}

func (apu *APU) stepTimer() {
	if apu.cycle%2 == 0 {
		apu.pulse1.stepTimer()
		apu.pulse2.stepTimer()
		apu.noise.stepTimer()
		apu.dmc.stepTimer()
	}
	apu.triangle.stepTimer()
}

func (apu *APU) stepEnvelope() {
	apu.pulse1.stepEnvelope()
	apu.pulse2.stepEnvelope()
	apu.triangle.stepCounter()
	apu.noise.stepEnvelope()
}

func (apu *APU) stepSweep() {
	apu.pulse1.stepSweep()
	apu.pulse2.stepSweep()
}

func (apu *APU) stepLength() {
	apu.pulse1.stepLength()
	apu.pulse2.stepLength()
	apu.triangle.stepLength()
	apu.noise.stepLength()
}

func (apu *APU) fireIRQ() {
	if apu.frameIRQ {
		apu.console.CPU.triggerIRQ()
	}
}

func (apu *APU) readRegister(address uint16) byte {
	switch address {
	case 0x4015:
		return apu.readStatus()
		// default:
		// 	log.Fatalf("unhandled apu register read at address: 0x%04X", address)
	}
	return 0
}

func (apu *APU) writeRegister(address uint16, value byte) {
	switch address {
	case 0x4000:
		apu.pulse1.writeControl(value)
	case 0x4001:
		apu.pulse1.writeSweep(value)
	case 0x4002:
		apu.pulse1.writeTimerLow(value)
	case 0x4003:
		apu.pulse1.writeTimerHigh(value)
	case 0x4004:
		apu.pulse2.writeControl(value)
	case 0x4005:
		apu.pulse2.writeSweep(value)
	case 0x4006:
		apu.pulse2.writeTimerLow(value)
	case 0x4007:
		apu.pulse2.writeTimerHigh(value)
	case 0x4008:
		apu.triangle.writeControl(value)
	case 0x4009:
	case 0x4010:
		apu.dmc.writeControl(value)
	case 0x4011:
		apu.dmc.writeValue(value)
	case 0x4012:
		apu.dmc.writeAddress(value)
	case 0x4013:
		apu.dmc.writeLength(value)
	case 0x400A:
		apu.triangle.writeTimerLow(value)
	case 0x400B:
		apu.triangle.writeTimerHigh(value)
	case 0x400C:
		apu.noise.writeControl(value)
	case 0x400D:
	case 0x400E:
		apu.noise.writePeriod(value)
	case 0x400F:
		apu.noise.writeLength(value)
	case 0x4015:
		apu.writeControl(value)
	case 0x4017:
		apu.writeFrameCounter(value)
		// default:
		// 	log.Fatalf("unhandled apu register write at address: 0x%04X", address)
	}
}

func (apu *APU) readStatus() byte {
	var result byte
	if apu.pulse1.lengthValue > 0 {
		result |= 1
	}
	if apu.pulse2.lengthValue > 0 {
		result |= 2
	}
	if apu.triangle.lengthValue > 0 {
		result |= 4
	}
	if apu.noise.lengthValue > 0 {
		result |= 8
	}
	if apu.dmc.currentLength > 0 {
		result |= 16
	}
	return result
}

func (apu *APU) writeControl(value byte) {
	apu.pulse1.enabled = value&1 == 1
	apu.pulse2.enabled = value&2 == 2
	apu.triangle.enabled = value&4 == 4
	apu.noise.enabled = value&8 == 8
	apu.dmc.enabled = value&16 == 16
	if !apu.pulse1.enabled {
		apu.pulse1.lengthValue = 0
	}
	if !apu.pulse2.enabled {
		apu.pulse2.lengthValue = 0
	}
	if !apu.triangle.enabled {
		apu.triangle.lengthValue = 0
	}
	if !apu.noise.enabled {
		apu.noise.lengthValue = 0
	}
	if !apu.dmc.enabled {
		apu.dmc.currentLength = 0
	} else {
		if apu.dmc.currentLength == 0 {
			apu.dmc.restart()
		}
	}
}

func (apu *APU) writeFrameCounter(value byte) {
	apu.framePeriod = 4 + (value>>7)&1
	apu.frameIRQ = (value>>6)&1 == 0
	// apu.frameValue = 0
	if apu.framePeriod == 5 {
		apu.stepEnvelope()
		apu.stepSweep()
		apu.stepLength()
	}
}
