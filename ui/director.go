package ui

import (
	"log"

	"github.com/Salies/sypha/nes"
	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

type View interface {
	Enter()
	Exit()
	Update(t, dt float64)
}

type Director struct {
	window    *glfw.Window
	audio     *Audio
	view      View
	timestamp float64
}

func NewDirector(window *glfw.Window, audio *Audio) *Director {
	director := Director{}
	director.window = window
	director.audio = audio
	return &director
}

func (d *Director) SetView(view View) {
	if d.view != nil {
		d.view.Exit()
	}
	d.view = view
	if d.view != nil {
		d.view.Enter()
	}
	d.timestamp = glfw.GetTime()
}

func (d *Director) Step() {
	gl.Clear(gl.COLOR_BUFFER_BIT)
	timestamp := glfw.GetTime()
	dt := timestamp - d.timestamp
	d.timestamp = timestamp
	if d.view != nil {
		d.view.Update(timestamp, dt)
	}
}

func (d *Director) Start(path string) {
	d.PlayGame(path)
	d.Run()
}

func (d *Director) Run() {
	for !d.window.ShouldClose() {
		d.Step()
		d.window.SwapBuffers()
		glfw.PollEvents()
	}
	d.SetView(nil)
}

func (d *Director) PlayGame(path string) {
	console, err := nes.NewConsole(path)
	if err != nil {
		log.Fatalln(err)
	}
	d.SetView(NewGameView(d, console))
}
