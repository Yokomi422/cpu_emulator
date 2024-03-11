#include <stdio.h>

// Instruction Code
#define MOV 0b0
#define ADD 0b1
#define SUB 0b10
#define AND 0b11
#define OR  0b100
#define SL  0b1001
#define SR  0b1010
#define SRA 0b1011
#define LDL 0b1100
#define LDH 0b1101
#define CMP 0b1110
#define JE  0b1111
#define JMP 0b10000
#define LD  0b10001
#define ST  0b10010
#define HLT 0b10011

// Register Code
#define REG0 0b0
#define REG1 0b1
#define REG2 0b10
#define REG3 0b11
#define REG4 0b100
#define REG5 0b101
#define REG6 0b110
#define REG7 0b111

// Register
short reg[8];
// Program Memory
short rom[256];
// Data Memory
short ram[256];

// Simple Assembler
void assembler(void);

short mov(short, short);
short add(short, short);
short sub(short, short);
short and(short, short);
short or(short, short);
short sl(short);
short sr(short);
short sra(short);
short ldl(short, short);
short ldh(short, short);
short cmp(short, short);
short je(short);
short jmp(short);
short ld(short, short);
short st(short, short);
short hlt(void);

short op_code(short);
short op_regA(short);
short op_regB(short);
short op_data(short);
short op_addr(short);

int main() {
	short pc;
	short ir;
	short flag_eq;

	assembler();

	pc = 0;
	flag_eq = 0;

	do {
		ir = rom[pc];
		printf("PC: %d, IR: %d\n, Reg0: %d, Reg1: %d Reg2: %d, Reg3: %d", pc, ir, reg[0], reg[1], reg[2], reg[3]); 

		pc++;
		switch(op_code(ir)) {
			case MOV:
				reg[op_regA(ir)] = reg[op_regB(ir)];
				break;
			case ADD:
				reg[op_regA(ir)] += reg[op_regB(ir)];
				break;
			case SUB:
				reg[op_regA(ir)] -= reg[op_regB(ir)];
				break;
			case AND:
				reg[op_regA(ir)] &= reg[op_regB(ir)];
				break;
			case OR:
				reg[op_regA(ir)] |= reg[op_regB(ir)];
				break;
			case SL:
				reg[op_regA(ir)] <<= 1;
				break;
			case SR:
				reg[op_regA(ir)] >>= 1;
				break;
			case SRA:
				reg[op_regA(ir)] = (reg[op_regA(ir)] & 0x8000) | (reg[op_regA(ir)] >> 1);
				break;
			case LDL:
				reg[op_regA(ir)] = (reg[op_regA(ir)] & 0xFF00) | op_data(ir);
				break;
			case LDH:
				reg[op_regA(ir)] = (reg[op_regA(ir)] & 0x00FF) | (op_data(ir) << 8);
				break;
			case CMP:
				if (reg[op_regA(ir)] == reg[op_regB(ir)]) {
					flag_eq = 1;
				} else {
					flag_eq = 0;
				}
				break;
			case JE:
				if (flag_eq == 1) {
					pc = op_addr(ir);
				}
				break;
			case JMP:
				pc = op_addr(ir);
				break;
			case LD:
				reg[op_regA(ir)] = ram[op_addr(ir)];
				break;
			case ST:
			ram[op_addr(ir)] = reg[op_regA(ir)];
			break;
			default:
				break;
		}
	} while (op_code(ir) != HLT);
	// display the result
	printf("ram[64] = %d\n", ram[64]);

	return 0;
}

void assembler(void) {
	rom[0] = ldh(REG0, 0);
	rom[1] = ldl(REG0, 0);
	rom[2] = ldh(REG1, 0);
	rom[3] = ldl(REG1, 1);
	rom[4] = ldh(REG2, 0);
	rom[5] = ldl(REG2, 0);
	rom[6] = ldh(REG3, 0);
	rom[7] = ldl(REG3, 10);
	rom[8] = add(REG2, REG1);
	rom[9] = add(REG0, REG2);
	rom[10] = st(REG0, 64);
	rom[11] = cmp(REG2, REG3);
	rom[12] = je(14);
	rom[13] = jmp(8);
	rom[14] = hlt();
}


// define the instruction code. There are machine code for each instruction.
short mov(short regA, short regB) {
	return (MOV << 11) | (regA << 8) | (regB << 5);
}

short add(short regA, short regB) {
	return (ADD << 11) | (regA << 8) | (regB << 5);
}

short sub(short regA, short regB) {
	return (SUB << 11) | (regA << 8) | (regB << 5);
}

short and(short regA, short regB) {
	return (AND << 11) | (regA << 8) | (regB << 5);
}

short or(short regA, short regB) {
	return (OR << 11) | (regA << 8) | (regB << 5);
}

short sl(short regA) {
	return (SL << 11) | (regA << 8);
}

short sr(short regA) {
	return (SR << 11) | (regA << 8);
}

short sra(short regA) {
	return (SRA << 11) | (regA << 8);
}

short ldl(short regA, short data) {
	return (LDL << 11) | (regA << 8) | (data & 0x00FF);
}

short ldh(short regA, short data) {
	return (LDH << 11) | (regA << 8) | (data & 0x00FF);
}

short cmp(short regA, short regB) {
	return (CMP << 11) | (regA << 8) | (regB << 5);
}

short je(short addr) {
	return (JE << 11) | (addr & 0x00FF);
}

short jmp(short addr) {
	return (JMP << 11) | (addr & 0x00FF);
}

short ld(short regA, short regB) {
	return (LD << 11) | (regA << 8) | (regB << 5);
}

short st(short regA, short regB) {
	return (ST << 11) | (regA << 8) | (regB << 5);
}

short hlt(void) {
	return (short)(HLT << 11);
}

// define the function to get the instruction code
short op_code(short ir) {
	return (ir >> 11) & 0b11111;
}

short op_regA(short ir) {
	return (ir >> 8) & 0b111;
}

short op_regB(short ir) {
	return (ir >> 5) & 0b111;
}

short op_data(short ir) {
    return ir & 0b11111111;
}
short op_addr(short ir) {
	return ir & 0b11111111;
}

// 解説
/*
Reg0は答え
Reg1は更新式
Reg2はi
Reg3は10
*/