package ui

import (
	"github.com/go-gl/gl/v2.1/gl"
)

const textureSize = 4096

type Texture struct {
	texture uint32
	lookup  map[string]int
	ch      chan string
}

func NewTexture() *Texture {
	texture := createTexture()
	gl.BindTexture(gl.TEXTURE_2D, texture)
	gl.TexImage2D(
		gl.TEXTURE_2D, 0, gl.RGBA,
		textureSize, textureSize,
		0, gl.RGBA, gl.UNSIGNED_BYTE, nil)
	gl.BindTexture(gl.TEXTURE_2D, 0)
	t := Texture{}
	t.texture = texture
	t.lookup = make(map[string]int)
	t.ch = make(chan string, 1024)
	return &t
}

func (t *Texture) Purge() {
	for {
		select {
		case path := <-t.ch:
			delete(t.lookup, path)
		default:
			return
		}
	}
}

func (t *Texture) Bind() {
	gl.BindTexture(gl.TEXTURE_2D, t.texture)
}

func (t *Texture) Unbind() {
	gl.BindTexture(gl.TEXTURE_2D, 0)
}
