package cartridge

import (
	"fmt"
)

type Mapper interface {
	Read(address uint16) byte
	Write(address uint16, value byte)
	Step()
}

func NewMapper(cartridge *Cartridge) (Mapper, error) {
	switch cartridge.Mapper {
	case 0:
		return NewMapper2(cartridge), nil
	case 2:
		return NewMapper2(cartridge), nil
	}
	err := fmt.Errorf("unsupported mapper: %d", cartridge.Mapper)
	return nil, err
}
