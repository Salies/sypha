package nes

import (
	"image"
	"image/color"

	"github.com/Salies/sypha/nes/apu"
	"github.com/Salies/sypha/nes/cartridge"
)

type Console struct {
	CPU         *CPU
	APU         *apu.APU
	PPU         *PPU
	Cartridge   *cartridge.Cartridge
	Controller1 *Controller
	Controller2 *Controller
	Mapper      cartridge.Mapper
	RAM         []byte
}

func NewConsole(path string) (*Console, error) {
	c, err := cartridge.LoadNESFile(path)
	if err != nil {
		return nil, err
	}
	ram := make([]byte, 2048)
	controller1 := NewController()
	controller2 := NewController()
	console := Console{
		nil, nil, nil, c, controller1, controller2, nil, ram}
	mapper, err := cartridge.NewMapper(c)
	if err != nil {
		return nil, err
	}
	console.Mapper = mapper
	console.CPU = NewCPU(&console)
	console.APU = apu.NewAPU(console.IRQcallback, console.CPU.dmcStepReaderCallBack)
	console.PPU = NewPPU(&console)
	return &console, nil
}

func (console *Console) IRQcallback() {
	console.CPU.triggerIRQ()
}

func (console *Console) Reset() {
	console.CPU.Reset()
}

func (console *Console) Step() int {
	cpuCycles := console.CPU.Step()
	ppuCycles := cpuCycles * 3
	for i := 0; i < ppuCycles; i++ {
		console.PPU.Step()
		console.Mapper.Step()
	}
	for i := 0; i < cpuCycles; i++ {
		console.APU.Step()
	}
	return cpuCycles
}

func (console *Console) StepFrame() int {
	cpuCycles := 0
	frame := console.PPU.Frame
	for frame == console.PPU.Frame {
		cpuCycles += console.Step()
	}
	return cpuCycles
}

func (console *Console) StepSeconds(seconds float64) {
	cycles := int(CPUFrequency * seconds)
	for cycles > 0 {
		cycles -= console.Step()
	}
}

func (console *Console) Buffer() *image.RGBA {
	return console.PPU.front
}

func (console *Console) BackgroundColor() color.RGBA {
	return Palette[console.PPU.readPalette(0)%64]
}

func (console *Console) SetButtons1(buttons [8]bool) {
	console.Controller1.SetButtons(buttons)
}

func (console *Console) SetButtons2(buttons [8]bool) {
	console.Controller2.SetButtons(buttons)
}

func (console *Console) SetAudioChannel(channel chan float32) {
	console.APU.Channel = channel
}

func (console *Console) SetAudioSampleRate(sampleRate float64) {
	if sampleRate != 0 {
		// Convert samples per second to cpu steps per sample
		console.APU.SampleRate = CPUFrequency / sampleRate
		// Initialize filters
		console.APU.FilterChain = apu.FilterChain{
			apu.HighPassFilter(float32(sampleRate), 90),
			apu.HighPassFilter(float32(sampleRate), 440),
			apu.LowPassFilter(float32(sampleRate), 14000),
		}
	} else {
		console.APU.FilterChain = nil
	}
}
