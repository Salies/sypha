package main

import (
	"log"
	"os"

	"github.com/Salies/sypha/ui"
)

func main() {
	// É necessário passar o caminho para uma ROM como argumento
	if len(os.Args) == 1 {
		log.Fatalln("ERRO: por favor, informe o caminho para uma ROM.")
	}
	romPath := os.Args[1]
	ui.Run(romPath)
}
