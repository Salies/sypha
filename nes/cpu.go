package nes

import (
	"fmt"
)

const CPUFrequency = 1789773

// interrupt types
const (
	_ = iota
	interruptNone
	interruptNMI
	interruptIRQ
)

// addressing modes
const (
	_ = iota
	modeAbsolute
	modeAbsoluteX
	modeAbsoluteY
	modeAccumulator
	modeImmediate
	modeImplied
	modeIndexedIndirect
	modeIndirect
	modeIndirectIndexed
	modeRelative
	modeZeroPage
	modeZeroPageX
	modeZeroPageY
)

type Instruction struct {
	mode       byte
	size       byte
	cycles     byte
	pageCycles byte
	name       string
	call       func(*stepInfo)
}

type CPU struct {
	Memory              // memory interface
	Cycles       uint64 // number of cycles
	PC           uint16 // program counter
	SP           byte   // stack pointer
	A            byte   // accumulator
	X            byte   // x register
	Y            byte   // y register
	C            byte   // carry flag
	Z            byte   // zero flag
	I            byte   // interrupt disable flag
	D            byte   // decimal mode flag
	B            byte   // break command flag
	U            byte   // unused flag
	V            byte   // overflow flag
	N            byte   // negative flag
	interrupt    byte   // interrupt type to perform
	stall        int    // number of cycles to stall
	instructions [256]Instruction
}

func NewCPU(console *Console) *CPU {
	cpu := CPU{Memory: NewCPUMemory(console)}
	cpu.instructions = [256]Instruction{
		{6, 2, 7, 0, "BRK", cpu.brk},
		{7, 2, 6, 0, "ORA", cpu.ora},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{7, 0, 8, 0, "SLO", cpu.slo},
		{11, 2, 3, 0, "NOP", cpu.nop},
		{11, 2, 3, 0, "ORA", cpu.ora},
		{11, 2, 5, 0, "ASL", cpu.asl},
		{11, 0, 5, 0, "SLO", cpu.slo},
		{6, 1, 3, 0, "PHP", cpu.php},
		{5, 2, 2, 0, "ORA", cpu.ora},
		{4, 1, 2, 0, "ASL", cpu.asl},
		{5, 0, 2, 0, "ANC", cpu.anc},
		{1, 3, 4, 0, "NOP", cpu.nop},
		{1, 3, 4, 0, "ORA", cpu.ora},
		{1, 3, 6, 0, "ASL", cpu.asl},
		{1, 0, 6, 0, "SLO", cpu.slo},
		{10, 2, 2, 1, "BPL", cpu.bpl},
		{9, 2, 5, 1, "ORA", cpu.ora},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "SLO", cpu.slo},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "ORA", cpu.ora},
		{12, 2, 6, 0, "ASL", cpu.asl},
		{12, 0, 6, 0, "SLO", cpu.slo},
		{6, 1, 2, 0, "CLC", cpu.clc},
		{3, 3, 4, 1, "ORA", cpu.ora},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "SLO", cpu.slo},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "ORA", cpu.ora},
		{2, 3, 7, 0, "ASL", cpu.asl},
		{2, 0, 7, 0, "SLO", cpu.slo},
		{1, 3, 6, 0, "JSR", cpu.jsr},
		{7, 2, 6, 0, "AND", cpu.and},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{7, 0, 8, 0, "RLA", cpu.rla},
		{11, 2, 3, 0, "BIT", cpu.bit},
		{11, 2, 3, 0, "AND", cpu.and},
		{11, 2, 5, 0, "ROL", cpu.rol},
		{11, 0, 5, 0, "RLA", cpu.rla},
		{6, 1, 4, 0, "PLP", cpu.plp},
		{5, 2, 2, 0, "AND", cpu.and},
		{4, 1, 2, 0, "ROL", cpu.rol},
		{5, 0, 2, 0, "ANC", cpu.anc},
		{1, 3, 4, 0, "BIT", cpu.bit},
		{1, 3, 4, 0, "AND", cpu.and},
		{1, 3, 6, 0, "ROL", cpu.rol},
		{1, 0, 6, 0, "RLA", cpu.rla},
		{10, 2, 2, 1, "BMI", cpu.bmi},
		{9, 2, 5, 1, "AND", cpu.and},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "RLA", cpu.rla},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "AND", cpu.and},
		{12, 2, 6, 0, "ROL", cpu.rol},
		{12, 0, 6, 0, "RLA", cpu.rla},
		{6, 1, 2, 0, "SEC", cpu.sec},
		{3, 3, 4, 1, "AND", cpu.and},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "RLA", cpu.rla},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "AND", cpu.and},
		{2, 3, 7, 0, "ROL", cpu.rol},
		{2, 0, 7, 0, "RLA", cpu.rla},
		{6, 1, 6, 0, "RTI", cpu.rti},
		{7, 2, 6, 0, "EOR", cpu.eor},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{7, 0, 8, 0, "SRE", cpu.sre},
		{11, 2, 3, 0, "NOP", cpu.nop},
		{11, 2, 3, 0, "EOR", cpu.eor},
		{11, 2, 5, 0, "LSR", cpu.lsr},
		{11, 0, 5, 0, "SRE", cpu.sre},
		{6, 1, 3, 0, "PHA", cpu.pha},
		{5, 2, 2, 0, "EOR", cpu.eor},
		{4, 1, 2, 0, "LSR", cpu.lsr},
		{5, 0, 2, 0, "ALR", cpu.alr},
		{1, 3, 3, 0, "JMP", cpu.jmp},
		{1, 3, 4, 0, "EOR", cpu.eor},
		{1, 3, 6, 0, "LSR", cpu.lsr},
		{1, 0, 6, 0, "SRE", cpu.sre},
		{10, 2, 2, 1, "BVC", cpu.bvc},
		{9, 2, 5, 1, "EOR", cpu.eor},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "SRE", cpu.sre},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "EOR", cpu.eor},
		{12, 2, 6, 0, "LSR", cpu.lsr},
		{12, 0, 6, 0, "SRE", cpu.sre},
		{6, 1, 2, 0, "CLI", cpu.cli},
		{3, 3, 4, 1, "EOR", cpu.eor},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "SRE", cpu.sre},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "EOR", cpu.eor},
		{2, 3, 7, 0, "LSR", cpu.lsr},
		{2, 0, 7, 0, "SRE", cpu.sre},
		{6, 1, 6, 0, "RTS", cpu.rts},
		{7, 2, 6, 0, "ADC", cpu.adc},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{7, 0, 8, 0, "RRA", cpu.rra},
		{11, 2, 3, 0, "NOP", cpu.nop},
		{11, 2, 3, 0, "ADC", cpu.adc},
		{11, 2, 5, 0, "ROR", cpu.ror},
		{11, 0, 5, 0, "RRA", cpu.rra},
		{6, 1, 4, 0, "PLA", cpu.pla},
		{5, 2, 2, 0, "ADC", cpu.adc},
		{4, 1, 2, 0, "ROR", cpu.ror},
		{5, 0, 2, 0, "ARR", cpu.arr},
		{8, 3, 5, 0, "JMP", cpu.jmp},
		{1, 3, 4, 0, "ADC", cpu.adc},
		{1, 3, 6, 0, "ROR", cpu.ror},
		{1, 0, 6, 0, "RRA", cpu.rra},
		{10, 2, 2, 1, "BVS", cpu.bvs},
		{9, 2, 5, 1, "ADC", cpu.adc},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "RRA", cpu.rra},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "ADC", cpu.adc},
		{12, 2, 6, 0, "ROR", cpu.ror},
		{12, 0, 6, 0, "RRA", cpu.rra},
		{6, 1, 2, 0, "SEI", cpu.sei},
		{3, 3, 4, 1, "ADC", cpu.adc},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "RRA", cpu.rra},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "ADC", cpu.adc},
		{2, 3, 7, 0, "ROR", cpu.ror},
		{2, 0, 7, 0, "RRA", cpu.rra},
		{5, 2, 2, 0, "NOP", cpu.nop},
		{7, 2, 6, 0, "STA", cpu.sta},
		{5, 0, 2, 0, "NOP", cpu.nop},
		{7, 0, 6, 0, "SAX", cpu.sax},
		{11, 2, 3, 0, "STY", cpu.sty},
		{11, 2, 3, 0, "STA", cpu.sta},
		{11, 2, 3, 0, "STX", cpu.stx},
		{11, 0, 3, 0, "SAX", cpu.sax},
		{6, 1, 2, 0, "DEY", cpu.dey},
		{5, 0, 2, 0, "NOP", cpu.nop},
		{6, 1, 2, 0, "TXA", cpu.txa},
		{5, 0, 2, 0, "XAA", cpu.xaa},
		{1, 3, 4, 0, "STY", cpu.sty},
		{1, 3, 4, 0, "STA", cpu.sta},
		{1, 3, 4, 0, "STX", cpu.stx},
		{1, 0, 4, 0, "SAX", cpu.sax},
		{10, 2, 2, 1, "BCC", cpu.bcc},
		{9, 2, 6, 0, "STA", cpu.sta},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 6, 0, "AHX", cpu.ahx},
		{12, 2, 4, 0, "STY", cpu.sty},
		{12, 2, 4, 0, "STA", cpu.sta},
		{13, 2, 4, 0, "STX", cpu.stx},
		{13, 0, 4, 0, "SAX", cpu.sax},
		{6, 1, 2, 0, "TYA", cpu.tya},
		{3, 3, 5, 0, "STA", cpu.sta},
		{6, 1, 2, 0, "TXS", cpu.txs},
		{3, 0, 5, 0, "TAS", cpu.tas},
		{2, 0, 5, 0, "SHY", cpu.shy},
		{2, 3, 5, 0, "STA", cpu.sta},
		{3, 0, 5, 0, "SHX", cpu.shx},
		{3, 0, 5, 0, "AHX", cpu.ahx},
		{5, 2, 2, 0, "LDY", cpu.ldy},
		{7, 2, 6, 0, "LDA", cpu.lda},
		{5, 2, 2, 0, "LDX", cpu.ldx},
		{7, 0, 6, 0, "LAX", cpu.lax},
		{11, 2, 3, 0, "LDY", cpu.ldy},
		{11, 2, 3, 0, "LDA", cpu.lda},
		{11, 2, 3, 0, "LDX", cpu.ldx},
		{11, 0, 3, 0, "LAX", cpu.lax},
		{6, 1, 2, 0, "TAY", cpu.tay},
		{5, 2, 2, 0, "LDA", cpu.lda},
		{6, 1, 2, 0, "TAX", cpu.tax},
		{5, 0, 2, 0, "LAX", cpu.lax},
		{1, 3, 4, 0, "LDY", cpu.ldy},
		{1, 3, 4, 0, "LDA", cpu.lda},
		{1, 3, 4, 0, "LDX", cpu.ldx},
		{1, 0, 4, 0, "LAX", cpu.lax},
		{10, 2, 2, 1, "BCS", cpu.bcs},
		{9, 2, 5, 1, "LDA", cpu.lda},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 5, 1, "LAX", cpu.lax},
		{12, 2, 4, 0, "LDY", cpu.ldy},
		{12, 2, 4, 0, "LDA", cpu.lda},
		{13, 2, 4, 0, "LDX", cpu.ldx},
		{13, 0, 4, 0, "LAX", cpu.lax},
		{6, 1, 2, 0, "CLV", cpu.clv},
		{3, 3, 4, 1, "LDA", cpu.lda},
		{6, 1, 2, 0, "TSX", cpu.tsx},
		{3, 0, 4, 1, "LAS", cpu.las},
		{2, 3, 4, 1, "LDY", cpu.ldy},
		{2, 3, 4, 1, "LDA", cpu.lda},
		{3, 3, 4, 1, "LDX", cpu.ldx},
		{3, 0, 4, 1, "LAX", cpu.lax},
		{5, 2, 2, 0, "CPY", cpu.cpy},
		{7, 2, 6, 0, "CMP", cpu.cmp},
		{5, 0, 2, 0, "NOP", cpu.nop},
		{7, 0, 8, 0, "DCP", cpu.dcp},
		{11, 2, 3, 0, "CPY", cpu.cpy},
		{11, 2, 3, 0, "CMP", cpu.cmp},
		{11, 2, 5, 0, "DEC", cpu.dec},
		{11, 0, 5, 0, "DCP", cpu.dcp},
		{6, 1, 2, 0, "INY", cpu.iny},
		{5, 2, 2, 0, "CMP", cpu.cmp},
		{6, 1, 2, 0, "DEX", cpu.dex},
		{5, 0, 2, 0, "AXS", cpu.axs},
		{1, 3, 4, 0, "CPY", cpu.cpy},
		{1, 3, 4, 0, "CMP", cpu.cmp},
		{1, 3, 6, 0, "DEC", cpu.dec},
		{1, 0, 6, 0, "DCP", cpu.dcp},
		{10, 2, 2, 1, "BNE", cpu.bne},
		{9, 2, 5, 1, "CMP", cpu.cmp},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "DCP", cpu.dcp},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "CMP", cpu.cmp},
		{12, 2, 6, 0, "DEC", cpu.dec},
		{12, 0, 6, 0, "DCP", cpu.dcp},
		{6, 1, 2, 0, "CLD", cpu.cld},
		{3, 3, 4, 1, "CMP", cpu.cmp},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "DCP", cpu.dcp},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "CMP", cpu.cmp},
		{2, 3, 7, 0, "DEC", cpu.dec},
		{2, 0, 7, 0, "DCP", cpu.dcp},
		{5, 2, 2, 0, "CPX", cpu.cpx},
		{7, 2, 6, 0, "SBC", cpu.sbc},
		{5, 0, 2, 0, "NOP", cpu.nop},
		{7, 0, 8, 0, "ISC", cpu.isc},
		{11, 2, 3, 0, "CPX", cpu.cpx},
		{11, 2, 3, 0, "SBC", cpu.sbc},
		{11, 2, 5, 0, "INC", cpu.inc},
		{11, 0, 5, 0, "ISC", cpu.isc},
		{6, 1, 2, 0, "INX", cpu.inx},
		{5, 2, 2, 0, "SBC", cpu.sbc},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{5, 0, 2, 0, "SBC", cpu.sbc},
		{1, 3, 4, 0, "CPX", cpu.cpx},
		{1, 3, 4, 0, "SBC", cpu.sbc},
		{1, 3, 6, 0, "INC", cpu.inc},
		{1, 0, 6, 0, "ISC", cpu.isc},
		{10, 2, 2, 1, "BEQ", cpu.beq},
		{9, 2, 5, 1, "SBC", cpu.sbc},
		{6, 0, 2, 0, "KIL", cpu.kil},
		{9, 0, 8, 0, "ISC", cpu.isc},
		{12, 2, 4, 0, "NOP", cpu.nop},
		{12, 2, 4, 0, "SBC", cpu.sbc},
		{12, 2, 6, 0, "INC", cpu.inc},
		{12, 0, 6, 0, "ISC", cpu.isc},
		{6, 1, 2, 0, "SED", cpu.sed},
		{3, 3, 4, 1, "SBC", cpu.sbc},
		{6, 1, 2, 0, "NOP", cpu.nop},
		{3, 0, 7, 0, "ISC", cpu.isc},
		{2, 3, 4, 1, "NOP", cpu.nop},
		{2, 3, 4, 1, "SBC", cpu.sbc},
		{2, 3, 7, 0, "INC", cpu.inc},
		{2, 0, 7, 0, "ISC", cpu.isc},
	}
	cpu.Reset()
	return &cpu
}

// Reset resets the CPU to its initial powerup state
func (cpu *CPU) Reset() {
	cpu.PC = cpu.Read16(0xFFFC)
	cpu.SP = 0xFD
	cpu.SetFlags(0x24)
}

// PrintInstruction prints the current CPU state
func (cpu *CPU) PrintInstruction() {
	opcode := cpu.Read(cpu.PC)
	bytes := cpu.instructions[opcode].size
	name := cpu.instructions[opcode].name
	w0 := fmt.Sprintf("%02X", cpu.Read(cpu.PC+0))
	w1 := fmt.Sprintf("%02X", cpu.Read(cpu.PC+1))
	w2 := fmt.Sprintf("%02X", cpu.Read(cpu.PC+2))
	if bytes < 2 {
		w1 = "  "
	}
	if bytes < 3 {
		w2 = "  "
	}
	fmt.Printf(
		"%4X  %s %s %s  %s %28s"+
			"A:%02X X:%02X Y:%02X P:%02X SP:%02X CYC:%3d\n",
		cpu.PC, w0, w1, w2, name, "",
		cpu.A, cpu.X, cpu.Y, cpu.Flags(), cpu.SP, (cpu.Cycles*3)%341)
}

// pagesDiffer returns true if the two addresses reference different pages
func pagesDiffer(a, b uint16) bool {
	return a&0xFF00 != b&0xFF00
}

// addBranchCycles adds a cycle for taking a branch and adds another cycle
// if the branch jumps to a new page
func (cpu *CPU) addBranchCycles(info *stepInfo) {
	cpu.Cycles++
	if pagesDiffer(info.pc, info.address) {
		cpu.Cycles++
	}
}

func (cpu *CPU) compare(a, b byte) {
	cpu.setZN(a - b)
	if a >= b {
		cpu.C = 1
	} else {
		cpu.C = 0
	}
}

// Read16 reads two bytes using Read to return a double-word value
func (cpu *CPU) Read16(address uint16) uint16 {
	lo := uint16(cpu.Read(address))
	hi := uint16(cpu.Read(address + 1))
	return hi<<8 | lo
}

// read16bug emulates a 6502 bug that caused the low byte to wrap without
// incrementing the high byte
func (cpu *CPU) read16bug(address uint16) uint16 {
	a := address
	b := (a & 0xFF00) | uint16(byte(a)+1)
	lo := cpu.Read(a)
	hi := cpu.Read(b)
	return uint16(hi)<<8 | uint16(lo)
}

// push pushes a byte onto the stack
func (cpu *CPU) push(value byte) {
	cpu.Write(0x100|uint16(cpu.SP), value)
	cpu.SP--
}

// pull pops a byte from the stack
func (cpu *CPU) pull() byte {
	cpu.SP++
	return cpu.Read(0x100 | uint16(cpu.SP))
}

// push16 pushes two bytes onto the stack
func (cpu *CPU) push16(value uint16) {
	hi := byte(value >> 8)
	lo := byte(value & 0xFF)
	cpu.push(hi)
	cpu.push(lo)
}

// pull16 pops two bytes from the stack
func (cpu *CPU) pull16() uint16 {
	lo := uint16(cpu.pull())
	hi := uint16(cpu.pull())
	return hi<<8 | lo
}

// Flags returns the processor status flags
func (cpu *CPU) Flags() byte {
	var flags byte
	flags |= cpu.C << 0
	flags |= cpu.Z << 1
	flags |= cpu.I << 2
	flags |= cpu.D << 3
	flags |= cpu.B << 4
	flags |= cpu.U << 5
	flags |= cpu.V << 6
	flags |= cpu.N << 7
	return flags
}

// SetFlags sets the processor status flags
func (cpu *CPU) SetFlags(flags byte) {
	cpu.C = (flags >> 0) & 1
	cpu.Z = (flags >> 1) & 1
	cpu.I = (flags >> 2) & 1
	cpu.D = (flags >> 3) & 1
	cpu.B = (flags >> 4) & 1
	cpu.U = (flags >> 5) & 1
	cpu.V = (flags >> 6) & 1
	cpu.N = (flags >> 7) & 1
}

// setZ sets the zero flag if the argument is zero
func (cpu *CPU) setZ(value byte) {
	if value == 0 {
		cpu.Z = 1
	} else {
		cpu.Z = 0
	}
}

// setN sets the negative flag if the argument is negative (high bit is set)
func (cpu *CPU) setN(value byte) {
	if value&0x80 != 0 {
		cpu.N = 1
	} else {
		cpu.N = 0
	}
}

// setZN sets the zero flag and the negative flag
func (cpu *CPU) setZN(value byte) {
	cpu.setZ(value)
	cpu.setN(value)
}

// triggerNMI causes a non-maskable interrupt to occur on the next cycle
func (cpu *CPU) triggerNMI() {
	cpu.interrupt = interruptNMI
}

// triggerIRQ causes an IRQ interrupt to occur on the next cycle
func (cpu *CPU) triggerIRQ() {
	if cpu.I == 0 {
		cpu.interrupt = interruptIRQ
	}
}

// Assistant function for the APU's DMC
func (cpu *CPU) dmcStepReaderCallBack(dmcCurrentAddress uint16) byte {
	cpu.stall += 4
	return cpu.Read(dmcCurrentAddress)
}

// stepInfo contains information that the instruction functions use
type stepInfo struct {
	address uint16
	pc      uint16
	mode    byte
}

// Step executes a single CPU instruction
func (cpu *CPU) Step() int {
	if cpu.stall > 0 {
		cpu.stall--
		return 1
	}

	cycles := cpu.Cycles

	switch cpu.interrupt {
	case interruptNMI:
		cpu.nmi()
	case interruptIRQ:
		cpu.irq()
	}
	cpu.interrupt = interruptNone

	opcode := cpu.Read(cpu.PC)
	mode := cpu.instructions[opcode].mode

	var address uint16
	var pageCrossed bool
	switch mode {
	case modeAbsolute:
		address = cpu.Read16(cpu.PC + 1)
	case modeAbsoluteX:
		address = cpu.Read16(cpu.PC+1) + uint16(cpu.X)
		pageCrossed = pagesDiffer(address-uint16(cpu.X), address)
	case modeAbsoluteY:
		address = cpu.Read16(cpu.PC+1) + uint16(cpu.Y)
		pageCrossed = pagesDiffer(address-uint16(cpu.Y), address)
	case modeAccumulator:
		address = 0
	case modeImmediate:
		address = cpu.PC + 1
	case modeImplied:
		address = 0
	case modeIndexedIndirect:
		address = cpu.read16bug(uint16(cpu.Read(cpu.PC+1) + cpu.X))
	case modeIndirect:
		address = cpu.read16bug(cpu.Read16(cpu.PC + 1))
	case modeIndirectIndexed:
		address = cpu.read16bug(uint16(cpu.Read(cpu.PC+1))) + uint16(cpu.Y)
		pageCrossed = pagesDiffer(address-uint16(cpu.Y), address)
	case modeRelative:
		offset := uint16(cpu.Read(cpu.PC + 1))
		if offset < 0x80 {
			address = cpu.PC + 2 + offset
		} else {
			address = cpu.PC + 2 + offset - 0x100
		}
	case modeZeroPage:
		address = uint16(cpu.Read(cpu.PC + 1))
	case modeZeroPageX:
		address = uint16(cpu.Read(cpu.PC+1)+cpu.X) & 0xff
	case modeZeroPageY:
		address = uint16(cpu.Read(cpu.PC+1)+cpu.Y) & 0xff
	}

	cpu.PC += uint16(cpu.instructions[opcode].size)
	cpu.Cycles += uint64(cpu.instructions[opcode].cycles)
	if pageCrossed {
		cpu.Cycles += uint64(cpu.instructions[opcode].pageCycles)
	}
	info := &stepInfo{address, cpu.PC, mode}
	cpu.instructions[opcode].call(info)

	return int(cpu.Cycles - cycles)
}

// NMI - Non-Maskable Interrupt
func (cpu *CPU) nmi() {
	cpu.push16(cpu.PC)
	cpu.php(nil)
	cpu.PC = cpu.Read16(0xFFFA)
	cpu.I = 1
	cpu.Cycles += 7
}

// IRQ - IRQ Interrupt
func (cpu *CPU) irq() {
	cpu.push16(cpu.PC)
	cpu.php(nil)
	cpu.PC = cpu.Read16(0xFFFE)
	cpu.I = 1
	cpu.Cycles += 7
}

// ADC - Add with Carry
func (cpu *CPU) adc(info *stepInfo) {
	a := cpu.A
	b := cpu.Read(info.address)
	c := cpu.C
	cpu.A = a + b + c
	cpu.setZN(cpu.A)
	if int(a)+int(b)+int(c) > 0xFF {
		cpu.C = 1
	} else {
		cpu.C = 0
	}
	if (a^b)&0x80 == 0 && (a^cpu.A)&0x80 != 0 {
		cpu.V = 1
	} else {
		cpu.V = 0
	}
}

// AND - Logical AND
func (cpu *CPU) and(info *stepInfo) {
	cpu.A = cpu.A & cpu.Read(info.address)
	cpu.setZN(cpu.A)
}

// ASL - Arithmetic Shift Left
func (cpu *CPU) asl(info *stepInfo) {
	if info.mode == modeAccumulator {
		cpu.C = (cpu.A >> 7) & 1
		cpu.A <<= 1
		cpu.setZN(cpu.A)
	} else {
		value := cpu.Read(info.address)
		cpu.C = (value >> 7) & 1
		value <<= 1
		cpu.Write(info.address, value)
		cpu.setZN(value)
	}
}

// BCC - Branch if Carry Clear
func (cpu *CPU) bcc(info *stepInfo) {
	if cpu.C == 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BCS - Branch if Carry Set
func (cpu *CPU) bcs(info *stepInfo) {
	if cpu.C != 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BEQ - Branch if Equal
func (cpu *CPU) beq(info *stepInfo) {
	if cpu.Z != 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BIT - Bit Test
func (cpu *CPU) bit(info *stepInfo) {
	value := cpu.Read(info.address)
	cpu.V = (value >> 6) & 1
	cpu.setZ(value & cpu.A)
	cpu.setN(value)
}

// BMI - Branch if Minus
func (cpu *CPU) bmi(info *stepInfo) {
	if cpu.N != 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BNE - Branch if Not Equal
func (cpu *CPU) bne(info *stepInfo) {
	if cpu.Z == 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BPL - Branch if Positive
func (cpu *CPU) bpl(info *stepInfo) {
	if cpu.N == 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BRK - Force Interrupt
func (cpu *CPU) brk(info *stepInfo) {
	cpu.push16(cpu.PC)
	cpu.php(info)
	cpu.sei(info)
	cpu.PC = cpu.Read16(0xFFFE)
}

// BVC - Branch if Overflow Clear
func (cpu *CPU) bvc(info *stepInfo) {
	if cpu.V == 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// BVS - Branch if Overflow Set
func (cpu *CPU) bvs(info *stepInfo) {
	if cpu.V != 0 {
		cpu.PC = info.address
		cpu.addBranchCycles(info)
	}
}

// CLC - Clear Carry Flag
func (cpu *CPU) clc(info *stepInfo) {
	cpu.C = 0
}

// CLD - Clear Decimal Mode
func (cpu *CPU) cld(info *stepInfo) {
	cpu.D = 0
}

// CLI - Clear Interrupt Disable
func (cpu *CPU) cli(info *stepInfo) {
	cpu.I = 0
}

// CLV - Clear Overflow Flag
func (cpu *CPU) clv(info *stepInfo) {
	cpu.V = 0
}

// CMP - Compare
func (cpu *CPU) cmp(info *stepInfo) {
	value := cpu.Read(info.address)
	cpu.compare(cpu.A, value)
}

// CPX - Compare X Register
func (cpu *CPU) cpx(info *stepInfo) {
	value := cpu.Read(info.address)
	cpu.compare(cpu.X, value)
}

// CPY - Compare Y Register
func (cpu *CPU) cpy(info *stepInfo) {
	value := cpu.Read(info.address)
	cpu.compare(cpu.Y, value)
}

// DEC - Decrement Memory
func (cpu *CPU) dec(info *stepInfo) {
	value := cpu.Read(info.address) - 1
	cpu.Write(info.address, value)
	cpu.setZN(value)
}

// DEX - Decrement X Register
func (cpu *CPU) dex(info *stepInfo) {
	cpu.X--
	cpu.setZN(cpu.X)
}

// DEY - Decrement Y Register
func (cpu *CPU) dey(info *stepInfo) {
	cpu.Y--
	cpu.setZN(cpu.Y)
}

// EOR - Exclusive OR
func (cpu *CPU) eor(info *stepInfo) {
	cpu.A = cpu.A ^ cpu.Read(info.address)
	cpu.setZN(cpu.A)
}

// INC - Increment Memory
func (cpu *CPU) inc(info *stepInfo) {
	value := cpu.Read(info.address) + 1
	cpu.Write(info.address, value)
	cpu.setZN(value)
}

// INX - Increment X Register
func (cpu *CPU) inx(info *stepInfo) {
	cpu.X++
	cpu.setZN(cpu.X)
}

// INY - Increment Y Register
func (cpu *CPU) iny(info *stepInfo) {
	cpu.Y++
	cpu.setZN(cpu.Y)
}

// JMP - Jump
func (cpu *CPU) jmp(info *stepInfo) {
	cpu.PC = info.address
}

// JSR - Jump to Subroutine
func (cpu *CPU) jsr(info *stepInfo) {
	cpu.push16(cpu.PC - 1)
	cpu.PC = info.address
}

// LDA - Load Accumulator
func (cpu *CPU) lda(info *stepInfo) {
	cpu.A = cpu.Read(info.address)
	cpu.setZN(cpu.A)
}

// LDX - Load X Register
func (cpu *CPU) ldx(info *stepInfo) {
	cpu.X = cpu.Read(info.address)
	cpu.setZN(cpu.X)
}

// LDY - Load Y Register
func (cpu *CPU) ldy(info *stepInfo) {
	cpu.Y = cpu.Read(info.address)
	cpu.setZN(cpu.Y)
}

// LSR - Logical Shift Right
func (cpu *CPU) lsr(info *stepInfo) {
	if info.mode == modeAccumulator {
		cpu.C = cpu.A & 1
		cpu.A >>= 1
		cpu.setZN(cpu.A)
	} else {
		value := cpu.Read(info.address)
		cpu.C = value & 1
		value >>= 1
		cpu.Write(info.address, value)
		cpu.setZN(value)
	}
}

// NOP - No Operation
func (cpu *CPU) nop(info *stepInfo) {
}

// ORA - Logical Inclusive OR
func (cpu *CPU) ora(info *stepInfo) {
	cpu.A = cpu.A | cpu.Read(info.address)
	cpu.setZN(cpu.A)
}

// PHA - Push Accumulator
func (cpu *CPU) pha(info *stepInfo) {
	cpu.push(cpu.A)
}

// PHP - Push Processor Status
func (cpu *CPU) php(info *stepInfo) {
	cpu.push(cpu.Flags() | 0x10)
}

// PLA - Pull Accumulator
func (cpu *CPU) pla(info *stepInfo) {
	cpu.A = cpu.pull()
	cpu.setZN(cpu.A)
}

// PLP - Pull Processor Status
func (cpu *CPU) plp(info *stepInfo) {
	cpu.SetFlags(cpu.pull()&0xEF | 0x20)
}

// ROL - Rotate Left
func (cpu *CPU) rol(info *stepInfo) {
	if info.mode == modeAccumulator {
		c := cpu.C
		cpu.C = (cpu.A >> 7) & 1
		cpu.A = (cpu.A << 1) | c
		cpu.setZN(cpu.A)
	} else {
		c := cpu.C
		value := cpu.Read(info.address)
		cpu.C = (value >> 7) & 1
		value = (value << 1) | c
		cpu.Write(info.address, value)
		cpu.setZN(value)
	}
}

// ROR - Rotate Right
func (cpu *CPU) ror(info *stepInfo) {
	if info.mode == modeAccumulator {
		c := cpu.C
		cpu.C = cpu.A & 1
		cpu.A = (cpu.A >> 1) | (c << 7)
		cpu.setZN(cpu.A)
	} else {
		c := cpu.C
		value := cpu.Read(info.address)
		cpu.C = value & 1
		value = (value >> 1) | (c << 7)
		cpu.Write(info.address, value)
		cpu.setZN(value)
	}
}

// RTI - Return from Interrupt
func (cpu *CPU) rti(info *stepInfo) {
	cpu.SetFlags(cpu.pull()&0xEF | 0x20)
	cpu.PC = cpu.pull16()
}

// RTS - Return from Subroutine
func (cpu *CPU) rts(info *stepInfo) {
	cpu.PC = cpu.pull16() + 1
}

// SBC - Subtract with Carry
func (cpu *CPU) sbc(info *stepInfo) {
	a := cpu.A
	b := cpu.Read(info.address)
	c := cpu.C
	cpu.A = a - b - (1 - c)
	cpu.setZN(cpu.A)
	if int(a)-int(b)-int(1-c) >= 0 {
		cpu.C = 1
	} else {
		cpu.C = 0
	}
	if (a^b)&0x80 != 0 && (a^cpu.A)&0x80 != 0 {
		cpu.V = 1
	} else {
		cpu.V = 0
	}
}

// SEC - Set Carry Flag
func (cpu *CPU) sec(info *stepInfo) {
	cpu.C = 1
}

// SED - Set Decimal Flag
func (cpu *CPU) sed(info *stepInfo) {
	cpu.D = 1
}

// SEI - Set Interrupt Disable
func (cpu *CPU) sei(info *stepInfo) {
	cpu.I = 1
}

// STA - Store Accumulator
func (cpu *CPU) sta(info *stepInfo) {
	cpu.Write(info.address, cpu.A)
}

// STX - Store X Register
func (cpu *CPU) stx(info *stepInfo) {
	cpu.Write(info.address, cpu.X)
}

// STY - Store Y Register
func (cpu *CPU) sty(info *stepInfo) {
	cpu.Write(info.address, cpu.Y)
}

// TAX - Transfer Accumulator to X
func (cpu *CPU) tax(info *stepInfo) {
	cpu.X = cpu.A
	cpu.setZN(cpu.X)
}

// TAY - Transfer Accumulator to Y
func (cpu *CPU) tay(info *stepInfo) {
	cpu.Y = cpu.A
	cpu.setZN(cpu.Y)
}

// TSX - Transfer Stack Pointer to X
func (cpu *CPU) tsx(info *stepInfo) {
	cpu.X = cpu.SP
	cpu.setZN(cpu.X)
}

// TXA - Transfer X to Accumulator
func (cpu *CPU) txa(info *stepInfo) {
	cpu.A = cpu.X
	cpu.setZN(cpu.A)
}

// TXS - Transfer X to Stack Pointer
func (cpu *CPU) txs(info *stepInfo) {
	cpu.SP = cpu.X
}

// TYA - Transfer Y to Accumulator
func (cpu *CPU) tya(info *stepInfo) {
	cpu.A = cpu.Y
	cpu.setZN(cpu.A)
}

// illegal opcodes below

func (cpu *CPU) ahx(info *stepInfo) {
}

func (cpu *CPU) alr(info *stepInfo) {
}

func (cpu *CPU) anc(info *stepInfo) {
}

func (cpu *CPU) arr(info *stepInfo) {
}

func (cpu *CPU) axs(info *stepInfo) {
}

func (cpu *CPU) dcp(info *stepInfo) {
}

func (cpu *CPU) isc(info *stepInfo) {
}

func (cpu *CPU) kil(info *stepInfo) {
}

func (cpu *CPU) las(info *stepInfo) {
}

func (cpu *CPU) lax(info *stepInfo) {
}

func (cpu *CPU) rla(info *stepInfo) {
}

func (cpu *CPU) rra(info *stepInfo) {
}

func (cpu *CPU) sax(info *stepInfo) {
}

func (cpu *CPU) shx(info *stepInfo) {
}

func (cpu *CPU) shy(info *stepInfo) {
}

func (cpu *CPU) slo(info *stepInfo) {
}

func (cpu *CPU) sre(info *stepInfo) {
}

func (cpu *CPU) tas(info *stepInfo) {
}

func (cpu *CPU) xaa(info *stepInfo) {
}
