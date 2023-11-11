package nes

import (
	"fmt"
)

type Mapper interface {
	Read(address uint16) byte
	Write(address uint16, value byte)
	Step()
}

func NewMapper(console *Console) (Mapper, error) {
	cartridge := console.Cartridge
	switch cartridge.Mapper {
	case 0:
		return NewMapper2(cartridge), nil
	case 2:
		return NewMapper2(cartridge), nil
	}
	err := fmt.Errorf("unsupported mapper: %d", cartridge.Mapper)
	return nil, err
}
