/*
ESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

This file is part of ESPeccy.

Based on previous work by:
- Víctor Iborra [Eremus] and David Crespo [dcrespo3d] (ESPectrum)
  https://github.com/EremusOne/ZX-ESPectrum-IDF
- David Crespo [dcrespo3d] (ZX-ESPectrum-Wiimote)
  https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote
- Ramón Martinez and Jorge Fuertes (ZX-ESPectrum)
  https://github.com/rampa069/ZX-ESPectrum
- Pete Todd (paseVGA)
  https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////
//
// z80cpp - Z80 emulator core
//
// Copyright (c) 2017, 2018, 2019, 2020 jsanchezv - https://github.com/jsanchezv
//
// Heretic optimizations and minor adaptations
// Copyright (c) 2021 dcrespo3d - https://github.com/dcrespo3d
//

// Converted to C++ from Java at
//... https://github.com/jsanchezv/Z80Core
//... commit c4f267e3564fa89bd88fd2d1d322f4d6b0069dbd
//... GPL 3
//... v1.0.0 (13/02/2017)
//    quick & dirty conversion by dddddd (AKA deesix)

#ifndef Z80CPP_H
#define Z80CPP_H

#include <stdint.h>

#define Z80CPP_IS_LITTLE_ENDIAN 1

/* Union allowing a register pair to be accessed as bytes or as a word */
typedef union {
#ifdef Z80CPP_IS_LITTLE_ENDIAN
	struct {
		uint8_t lo, hi;
	} byte8;
#else
    struct {
        uint8_t hi, lo;
    } byte8;
#endif
    uint16_t word;
} RegisterPair;

#define REG_B   regBC.byte8.hi
#define REG_C   regBC.byte8.lo
#define REG_BC  regBC.word
#define REG_Bx  regBCx.byte8.hi
#define REG_Cx  regBCx.byte8.lo
#define REG_BCx regBCx.word

#define REG_D   regDE.byte8.hi
#define REG_E   regDE.byte8.lo
#define REG_DE  regDE.word
#define REG_Dx  regDEx.byte8.hi
#define REG_Ex  regDEx.byte8.lo
#define REG_DEx regDEx.word

#define REG_H   regHL.byte8.hi
#define REG_L   regHL.byte8.lo
#define REG_HL  regHL.word
#define REG_Hx  regHLx.byte8.hi
#define REG_Lx  regHLx.byte8.lo
#define REG_HLx regHLx.word

#define REG_IXh regIX.byte8.hi
#define REG_IXl regIX.byte8.lo
#define REG_IX  regIX.word

#define REG_IYh regIY.byte8.hi
#define REG_IYl regIY.byte8.lo
#define REG_IY  regIY.word

#define REG_Ax  regAFx.byte8.hi
#define REG_Fx  regAFx.byte8.lo
#define REG_AFx regAFx.word

#define REG_PCh regPC.byte8.hi
#define REG_PCl regPC.byte8.lo
#define REG_PC  regPC.word

#define REG_S   regSP.byte8.hi
#define REG_P   regSP.byte8.lo
#define REG_SP  regSP.word

#define REG_W   memptr.byte8.hi
#define REG_Z   memptr.byte8.lo
#define REG_WZ  memptr.word

class Z80 {
public:
    // Modos de interrupción
    enum IntMode {
        IM0, IM1, IM2
    };
private:
    // Código de instrucción a ejecutar
    // Poner esta variable como local produce peor rendimiento
    // ZEXALL test: (local) 1:54 vs 1:47 (visitante)
    static uint8_t opCode;
    // Se está ejecutando una instrucción prefijada con DD, ED o FD
    // Los valores permitidos son [0x00, 0xDD, 0xED, 0xFD]
    // El prefijo 0xCB queda al margen porque, detrás de 0xCB, siempre
    // viene un código de instrucción válido, tanto si delante va un
    // 0xDD o 0xFD como si no.
    static uint8_t prefixOpcode;
    // Subsistema de notificaciones
    // static bool execDone;
    // Posiciones de los flags
    const static uint8_t CARRY_MASK = 0x01;
    const static uint8_t ADDSUB_MASK = 0x02;
    const static uint8_t PARITY_MASK = 0x04;
    const static uint8_t OVERFLOW_MASK = 0x04; // alias de PARITY_MASK
    const static uint8_t BIT3_MASK = 0x08;
    const static uint8_t HALFCARRY_MASK = 0x10;
    const static uint8_t BIT5_MASK = 0x20;
    const static uint8_t ZERO_MASK = 0x40;
    const static uint8_t SIGN_MASK = 0x80;
    // Máscaras de conveniencia
    const static uint8_t FLAG_53_MASK = BIT5_MASK | BIT3_MASK;
    const static uint8_t FLAG_SZ_MASK = SIGN_MASK | ZERO_MASK;
    const static uint8_t FLAG_SZHN_MASK = FLAG_SZ_MASK | HALFCARRY_MASK | ADDSUB_MASK;
    const static uint8_t FLAG_SZP_MASK = FLAG_SZ_MASK | PARITY_MASK;
    const static uint8_t FLAG_SZHP_MASK = FLAG_SZP_MASK | HALFCARRY_MASK;
    // Acumulador y resto de registros de 8 bits
    static uint8_t regA;
    // Flags sIGN, zERO, 5, hALFCARRY, 3, pARITY y ADDSUB (n)
    static uint8_t sz5h3pnFlags;
    // El flag Carry es el único que se trata aparte
    static bool carryFlag;
    // Registros principales y alternativos
    static RegisterPair regBC, regBCx, regDE, regDEx, regHL, regHLx;
    /* Flags para indicar la modificación del registro F en la instrucción actual
     * y en la anterior.
     * Son necesarios para emular el comportamiento de los bits 3 y 5 del
     * registro F con las instrucciones CCF/SCF.
     *
     * http://www.worldofspectrum.org/forums/showthread.php?t=41834
     * http://www.worldofspectrum.org/forums/showthread.php?t=41704
     *
     * Thanks to Patrik Rak for his tests and investigations.
     */
    static bool flagQ, lastFlagQ;

    // Acumulador alternativo y flags -- 8 bits
    static RegisterPair regAFx;

    // Registros de propósito específico
    // *PC -- Program Counter -- 16 bits*
    static RegisterPair regPC;
    // *IX -- Registro de índice -- 16 bits*
    static RegisterPair regIX;
    // *IY -- Registro de índice -- 16 bits*
    static RegisterPair regIY;
    // *SP -- Stack Pointer -- 16 bits*
    static RegisterPair regSP;
    // *I -- Vector de interrupción -- 8 bits*
    static uint8_t regI;
    // *R -- Refresco de memoria -- 7 bits*
    static uint8_t regR;
    // *R7 -- Refresco de memoria -- 1 bit* (bit superior de R)
    static bool regRbit7;
    //Flip-flops de interrupción
    static bool ffIFF1;
    static bool ffIFF2;
    // EI solo habilita las interrupciones DESPUES de ejecutar la
    // siguiente instrucción (excepto si la siguiente instrucción es un EI...)
    static bool pendingEI;
    // Estado de la línea NMI
    static bool activeNMI;
    // Modo de interrupción
    static IntMode modeINT;
    // halted == true cuando la CPU está ejecutando un HALT (28/03/2010)
    static bool halted;
    // pinReset == true, se ha producido un reset a través de la patilla
    static bool pinReset;

    /*
     * Registro interno que usa la CPU de la siguiente forma
     *
     * ADD HL,xx      = Valor del registro H antes de la suma
     * LD r,(IX/IY+d) = Byte superior de la suma de IX/IY+d
     * JR d           = Byte superior de la dirección de destino del salto
     *
     * 04/12/2008     No se vayan todavía, aún hay más. Con lo que se ha
     *                implementado hasta ahora parece que funciona. El resto de
     *                la historia está contada en:
     *                http://zx.pk.ru/attachment.php?attachmentid=2989
     *
     * 25/09/2009     Se ha completado la emulación de MEMPTR. A señalar que
     *                no se puede comprobar si MEMPTR se ha emulado bien hasta
     *                que no se emula el comportamiento del registro en las
     *                instrucciones CPI y CPD. Sin ello, todos los tests de
     *                z80tests.tap fallarán aunque se haya emulado bien al
     *                registro en TODAS las otras instrucciones.
     *                Shit yourself, little parrot.
     */

    static RegisterPair memptr;
    // I and R registers
    static inline RegisterPair getPairIR(void);

    /* Algunos flags se precalculan para un tratamiento más rápido
     * Concretamente, SIGN, ZERO, los bits 3, 5, PARITY y ADDSUB:
     * sz53n_addTable tiene el ADDSUB flag a 0 y paridad sin calcular
     * sz53pn_addTable tiene el ADDSUB flag a 0 y paridad calculada
     * sz53n_subTable tiene el ADDSUB flag a 1 y paridad sin calcular
     * sz53pn_subTable tiene el ADDSUB flag a 1 y paridad calculada
     * El resto de bits están a 0 en las cuatro tablas lo que es
     * importante para muchas operaciones que ponen ciertos flags a 0 por real
     * decreto. Si lo ponen a 1 por el mismo método basta con hacer un OR con
     * la máscara correspondiente.
     */
    static const uint8_t sz53n_addTable[256];
    static const uint8_t sz53pn_addTable[256];
    static const uint8_t sz53n_subTable[256];
    static const uint8_t sz53pn_subTable[256];

    // Un true en una dirección indica que se debe notificar que se va a
    // ejecutar la instrucción que está en esa direción.
#ifdef WITH_BREAKPOINT_SUPPORT
    static bool breakpointEnabled {false};
#endif
    static void copyToRegister(uint8_t value);

public:
    // Constructor de la clase
    static void create();
    static void destroy();

    // Acceso a registros de 8 bits
    // Access to 8-bit registers
    static inline uint8_t getRegA(void) { return regA; }
    static inline void setRegA(uint8_t value) { regA = value; }

    static inline uint8_t getRegB(void) { return REG_B; }
    static inline void setRegB(uint8_t value) { REG_B = value; }

    static inline uint8_t getRegC(void) { return REG_C; }
    static inline void setRegC(uint8_t value) { REG_C = value; }

    static inline uint8_t getRegD(void) { return REG_D; }
    static inline void setRegD(uint8_t value) { REG_D = value; }

    static inline uint8_t getRegE(void) { return REG_E; }
    static inline void setRegE(uint8_t value) { REG_E = value; }

    static inline uint8_t getRegH(void) { return REG_H; }
    static inline void setRegH(uint8_t value) { REG_H = value; }

    static inline uint8_t getRegL(void) { return REG_L; }
    static inline void setRegL(uint8_t value) { REG_L = value; }

    // Acceso a registros alternativos de 8 bits
    // Access to alternate 8-bit registers
    static inline uint8_t getRegAx(void) { return REG_Ax; }
    static inline void setRegAx(uint8_t value) { REG_Ax = value; }

    static inline uint8_t getRegFx(void) { return REG_Fx; }
    static inline void setRegFx(uint8_t value) { REG_Fx = value; }

    static inline uint8_t getRegBx(void) { return REG_Bx; }
    static inline void setRegBx(uint8_t value) { REG_Bx = value; }

    static inline uint8_t getRegCx(void) { return REG_Cx; }
    static inline void setRegCx(uint8_t value) { REG_Cx = value; }

    static inline uint8_t getRegDx(void) { return REG_Dx; }
    static inline void setRegDx(uint8_t value) { REG_Dx = value; }

    static inline uint8_t getRegEx(void) { return REG_Ex; }
    static inline void setRegEx(uint8_t value) { REG_Ex = value; }

    static inline uint8_t getRegHx(void) { return REG_Hx; }
    static inline void setRegHx(uint8_t value) { REG_Hx = value; }

    static inline uint8_t getRegLx(void) { return REG_Lx; }
    static inline void setRegLx(uint8_t value) { REG_Lx = value; }

    // Acceso a registros de 16 bits
    // Access to registers pairs
    static inline uint16_t getRegAF(void) { return (regA << 8) | (carryFlag ? sz5h3pnFlags | CARRY_MASK : sz5h3pnFlags); }
    static inline void setRegAF(uint16_t word) { regA = word >> 8; sz5h3pnFlags = word & 0xfe; carryFlag = (word & CARRY_MASK) != 0; }

    static inline uint16_t getRegAFx(void) { return REG_AFx; }
    static inline void setRegAFx(uint16_t word) { REG_AFx = word; }

    static inline uint16_t getRegBC(void) { return REG_BC; }
    static inline void setRegBC(uint16_t word) { REG_BC = word; }

    static inline uint16_t getRegBCx(void) { return REG_BCx; }
    static inline void setRegBCx(uint16_t word) { REG_BCx = word; }

    static inline uint16_t getRegDE(void) { return REG_DE; }
    static inline void setRegDE(uint16_t word) { REG_DE = word; }

    static inline uint16_t getRegDEx(void) { return REG_DEx; }
    static inline void setRegDEx(uint16_t word) { REG_DEx = word; }

    static inline uint16_t getRegHL(void) { return REG_HL; }
    static inline void setRegHL(uint16_t word) { REG_HL = word; }

    static inline uint16_t getRegHLx(void) { return REG_HLx; }
    static inline void setRegHLx(uint16_t word) { REG_HLx = word; }

    // Acceso a registros de propósito específico
    // Access to special purpose registers
    static inline uint16_t getRegPC(void) { return REG_PC; }
    static inline void setRegPC(uint16_t address) { REG_PC = address; }

    static inline uint16_t getRegSP(void) { return REG_SP; }
    static inline void setRegSP(uint16_t word) { REG_SP = word; }

    static inline uint16_t getRegIX(void) { return REG_IX; }
    static inline void setRegIX(uint16_t word) { REG_IX = word; }

    static inline uint16_t getRegIY(void) { return REG_IY; }
    static inline void setRegIY(uint16_t word) { REG_IY = word; }

    static inline uint8_t getRegI(void) { return regI; }
    static inline void setRegI(uint8_t value) { regI = value; }

    static inline uint8_t getRegR(void) { return regRbit7 ? regR | SIGN_MASK : regR & 0x7f; }
    static inline void setRegR(uint8_t value) { regR = value & 0x7f; regRbit7 = (value > 0x7f); }

    // Acceso al registro oculto MEMPTR
    // Hidden register MEMPTR (known as WZ at Zilog doc?)
    static inline uint16_t getMemPtr(void) { return REG_WZ; }
    static inline void setMemPtr(uint16_t word) { REG_WZ = word; }

    // Acceso a los flags uno a uno
    // Access to single flags from F register
    static inline bool isCarryFlag(void) { return carryFlag; }
    static inline void setCarryFlag(bool state) { carryFlag = state; }

    static inline bool isAddSubFlag(void) { return (sz5h3pnFlags & ADDSUB_MASK) != 0; }
    static inline void setAddSubFlag(bool state);

    static inline bool isParOverFlag(void) { return (sz5h3pnFlags & PARITY_MASK) != 0; }
    static inline void setParOverFlag(bool state);

    /* Undocumented flag */
    static inline bool isBit3Flag(void) { return (sz5h3pnFlags & BIT3_MASK) != 0; }
    static inline void setBit3Fag(bool state);

    static inline bool isHalfCarryFlag(void) { return (sz5h3pnFlags & HALFCARRY_MASK) != 0; }
    static inline void setHalfCarryFlag(bool state);

    /* Undocumented flag */
    static inline bool isBit5Flag(void) { return (sz5h3pnFlags & BIT5_MASK) != 0; }
    static inline void setBit5Flag(bool state);

    static inline bool isZeroFlag(void) { return (sz5h3pnFlags & ZERO_MASK) != 0; }
    static inline void setZeroFlag(bool state);

    static inline bool isSignFlag(void) { return sz5h3pnFlags >= SIGN_MASK; }
    static inline void setSignFlag(bool state);

    // Acceso a los flags F
    // Access to F register
    static inline uint8_t getFlags(void) { return carryFlag ? sz5h3pnFlags | CARRY_MASK : sz5h3pnFlags; }
    static inline void setFlags(uint8_t regF) { sz5h3pnFlags = regF & 0xfe; carryFlag = (regF & CARRY_MASK) != 0; }

    // Acceso a los flip-flops de interrupción
    // Interrupt flip-flops
    static inline bool isIFF1(void) { return ffIFF1; }
    static inline void setIFF1(bool state) { ffIFF1 = state; }

    static inline bool isIFF2(void) { return ffIFF2; }
    static inline void setIFF2(bool state) { ffIFF2 = state; }

    static inline bool isNMI(void) { return activeNMI; }
    static inline void setNMI(bool nmi) { activeNMI = nmi; }

    // /NMI is negative level triggered.
    static inline void triggerNMI(void) { activeNMI = true; }

    //Acceso al modo de interrupción
    // Maskable interrupt mode
    static inline IntMode getIM(void) { return modeINT; }
    static inline void setIM(IntMode mode) { modeINT = mode; }

    static inline bool isHalted(void) { return halted; }
    static inline void setHalted(bool state) { halted = state; }

    // Reset requested by /RESET signal (not power-on)
    static inline void setPinReset(void) { pinReset = true; }

    static inline bool isPendingEI(void) { return pendingEI; }
    static inline void setPendingEI(bool state) { pendingEI = state; }

    // Reset
    static void reset(void);

    // Execute one instruction
    static void execute();
    static void exec_nocheck();
    static void exec_nocheck_2A3();

    // Check INT
    static void checkINT(void);

    // Do NMI
    static void doNMI(void);

    static void incRegR(uint8_t inc);

    static void Xor(uint8_t oper8);

    static void Cp(uint8_t oper8);

#ifdef WITH_BREAKPOINT_SUPPORT
    static bool isBreakpoint(void) { return breakpointEnabled; }
    static void setBreakpoint(bool state) { breakpointEnabled = state; }
#endif

#ifdef WITH_EXEC_DONE
    static void setExecDone(bool status) { execDone = status; }
#endif

private:
    // Rota a la izquierda el valor del argumento
    static inline void rlc(uint8_t &oper8);

    // Rota a la izquierda el valor del argumento
    static inline void rl(uint8_t &oper8);

    // Rota a la izquierda el valor del argumento
    static inline void sla(uint8_t &oper8);

    // Rota a la izquierda el valor del argumento (como sla salvo por el bit 0)
    static inline void sll(uint8_t &oper8);

    // Rota a la derecha el valor del argumento
    static inline void rrc(uint8_t &oper8);

    // Rota a la derecha el valor del argumento
    static inline void rr(uint8_t &oper8);

    // Rota a la derecha 1 bit el valor del argumento
    static inline void sra(uint8_t &oper8);

    // Rota a la derecha 1 bit el valor del argumento
    static inline void srl(uint8_t &oper8);

    // Incrementa un valor de 8 bits modificando los flags oportunos
    static inline void inc8(uint8_t &oper8);

    // Decrementa un valor de 8 bits modificando los flags oportunos
    static inline void dec8(uint8_t &oper8);

    // Suma de 8 bits afectando a los flags
    static inline void add(uint8_t oper8);

    // Suma con acarreo de 8 bits
    static inline void adc(uint8_t oper8);

    // Suma dos operandos de 16 bits sin carry afectando a los flags
    static inline void add16(RegisterPair &reg16, uint16_t oper16);

    // Suma con acarreo de 16 bits
    static inline void adc16(uint16_t reg16);

    // Resta de 8 bits
    static inline void sub(uint8_t oper8);

    // Resta con acarreo de 8 bits
    static inline void sbc(uint8_t oper8);

    // Resta con acarreo de 16 bits
    static inline void sbc16(uint16_t reg16);

    // Operación AND lógica
    // Simple 'and' is C++ reserved keyword
    static inline void and_(uint8_t oper8);

    // Operación XOR lógica
    // Simple 'xor' is C++ reserved keyword
    static inline void xor_(uint8_t oper8);

    // Operación OR lógica
    // Simple 'or' is C++ reserved keyword
    static inline void or_(uint8_t oper8);

    // Operación de comparación con el registro A
    // es como SUB, pero solo afecta a los flags
    // Los flags SIGN y ZERO se calculan a partir del resultado
    // Los flags 3 y 5 se copian desde el operando (sigh!)
    static inline void cp(uint8_t oper8);

    // DAA
    static inline void daa(void);

    // POP
    static inline uint16_t pop(void);

    // PUSH
    static inline void push(uint16_t word);

    // LDI
    static void ldi(void);

    // LDD
    static void ldd(void);

    // CPI
    static void cpi(void);

    // CPD
    static void cpd(void);

    // INI
    static void ini(void);

    // IND
    static void ind(void);

    // OUTI
    static void outi(void);

    // OUTD
    static void outd(void);

    static void SetAbortedINxR_OTxRFlags();

    // BIT n,r
    static inline void bitTest(uint8_t mask, uint8_t reg);

    //Interrupción
    static void interrupt(void);

    //Interrupción NMI
    static void nmi(void);

    // Decode main opcodes
    // static void decodeOpcode();

    // Subconjunto de instrucciones 0xCB
    // decode CBXX opcodes
    // static void decodeCB(void);

    // Subconjunto de instrucciones 0xDD / 0xFD 0xCB
    // Decode DD / FD CB opcodes
    static void decodeDDFDCB(uint16_t address);

    //Subconjunto de instrucciones 0xED
    // Decode EDXX opcodes
    static void decodeED(void);

    static void (*dcOpcode[256])();
    static void (*dcCB[256])();
    static void (*dcDDFD[256])(RegisterPair& regIXY);

    //Subconjunto de instrucciones 0xDD / 0xFD
    // Decode DD/FD opcodes

    static void dcDDFD09(RegisterPair& regIXY);
    static void dcDDFD19(RegisterPair& regIXY);
    static void dcDDFD21(RegisterPair& regIXY);
    static void dcDDFD22(RegisterPair& regIXY);
    static void dcDDFD23(RegisterPair& regIXY);
    static void dcDDFD24(RegisterPair& regIXY);
    static void dcDDFD25(RegisterPair& regIXY);
    static void dcDDFD26(RegisterPair& regIXY);
    static void dcDDFD29(RegisterPair& regIXY);
    static void dcDDFD2A(RegisterPair& regIXY);
    static void dcDDFD2B(RegisterPair& regIXY);
    static void dcDDFD2C(RegisterPair& regIXY);
    static void dcDDFD2D(RegisterPair& regIXY);
    static void dcDDFD2E(RegisterPair& regIXY);
    static void dcDDFD34(RegisterPair& regIXY);
    static void dcDDFD35(RegisterPair& regIXY);
    static void dcDDFD36(RegisterPair& regIXY);
    static void dcDDFD39(RegisterPair& regIXY);
    static void dcDDFD44(RegisterPair& regIXY);
    static void dcDDFD45(RegisterPair& regIXY);
    static void dcDDFD46(RegisterPair& regIXY);
    static void dcDDFD4C(RegisterPair& regIXY);
    static void dcDDFD4D(RegisterPair& regIXY);
    static void dcDDFD4E(RegisterPair& regIXY);
    static void dcDDFD54(RegisterPair& regIXY);
    static void dcDDFD55(RegisterPair& regIXY);
    static void dcDDFD56(RegisterPair& regIXY);
    static void dcDDFD5C(RegisterPair& regIXY);
    static void dcDDFD5D(RegisterPair& regIXY);
    static void dcDDFD5E(RegisterPair& regIXY);
    static void dcDDFD60(RegisterPair& regIXY);
    static void dcDDFD61(RegisterPair& regIXY);
    static void dcDDFD62(RegisterPair& regIXY);
    static void dcDDFD63(RegisterPair& regIXY);
    static void dcDDFD64(RegisterPair& regIXY);
    static void dcDDFD65(RegisterPair& regIXY);
    static void dcDDFD66(RegisterPair& regIXY);
    static void dcDDFD67(RegisterPair& regIXY);
    static void dcDDFD68(RegisterPair& regIXY);
    static void dcDDFD69(RegisterPair& regIXY);
    static void dcDDFD6A(RegisterPair& regIXY);
    static void dcDDFD6B(RegisterPair& regIXY);
    static void dcDDFD6C(RegisterPair& regIXY);
    static void dcDDFD6D(RegisterPair& regIXY);
    static void dcDDFD6E(RegisterPair& regIXY);
    static void dcDDFD6F(RegisterPair& regIXY);
    static void dcDDFD70(RegisterPair& regIXY);
    static void dcDDFD71(RegisterPair& regIXY);
    static void dcDDFD72(RegisterPair& regIXY);
    static void dcDDFD73(RegisterPair& regIXY);
    static void dcDDFD74(RegisterPair& regIXY);
    static void dcDDFD75(RegisterPair& regIXY);
    static void dcDDFD77(RegisterPair& regIXY);
    static void dcDDFD7C(RegisterPair& regIXY);
    static void dcDDFD7D(RegisterPair& regIXY);
    static void dcDDFD7E(RegisterPair& regIXY);
    static void dcDDFD84(RegisterPair& regIXY);
    static void dcDDFD85(RegisterPair& regIXY);
    static void dcDDFD86(RegisterPair& regIXY);
    static void dcDDFD8C(RegisterPair& regIXY);
    static void dcDDFD8D(RegisterPair& regIXY);
    static void dcDDFD8E(RegisterPair& regIXY);
    static void dcDDFD94(RegisterPair& regIXY);
    static void dcDDFD95(RegisterPair& regIXY);
    static void dcDDFD96(RegisterPair& regIXY);
    static void dcDDFD9C(RegisterPair& regIXY);
    static void dcDDFD9D(RegisterPair& regIXY);
    static void dcDDFD9E(RegisterPair& regIXY);
    static void dcDDFDA4(RegisterPair& regIXY);
    static void dcDDFDA5(RegisterPair& regIXY);
    static void dcDDFDA6(RegisterPair& regIXY);
    static void dcDDFDAC(RegisterPair& regIXY);
    static void dcDDFDAD(RegisterPair& regIXY);
    static void dcDDFDAE(RegisterPair& regIXY);
    static void dcDDFDB4(RegisterPair& regIXY);
    static void dcDDFDB5(RegisterPair& regIXY);
    static void dcDDFDB6(RegisterPair& regIXY);
    static void dcDDFDBC(RegisterPair& regIXY);
    static void dcDDFDBD(RegisterPair& regIXY);
    static void dcDDFDBE(RegisterPair& regIXY);
    static void dcDDFDCB(RegisterPair& regIXY);
    static void dcDDFDDD(RegisterPair& regIXY);
    static void dcDDFDE1(RegisterPair& regIXY);
    static void dcDDFDE3(RegisterPair& regIXY);
    static void dcDDFDE5(RegisterPair& regIXY);
    static void dcDDFDE9(RegisterPair& regIXY);
    static void dcDDFDED(RegisterPair& regIXY);
    static void dcDDFDF9(RegisterPair& regIXY);
    static void dcDDFDFD(RegisterPair& regIXY);
    static void dcDDFDdefault(RegisterPair& regIXY);

    static void decodeOpcode00(void);
    static void decodeOpcode01(void);
    static void decodeOpcode02(void);
    static void decodeOpcode03(void);
    static void decodeOpcode04(void);
    static void decodeOpcode05(void);
    static void decodeOpcode06(void);
    static void decodeOpcode07(void);
    static void decodeOpcode08(void);
    static void decodeOpcode09(void);
    static void decodeOpcode0A(void);
    static void decodeOpcode0B(void);
    static void decodeOpcode0C(void);
    static void decodeOpcode0D(void);
    static void decodeOpcode0E(void);
    static void decodeOpcode0F(void);

    static void decodeOpcode10(void);
    static void decodeOpcode11(void);
    static void decodeOpcode12(void);
    static void decodeOpcode13(void);
    static void decodeOpcode14(void);
    static void decodeOpcode15(void);
    static void decodeOpcode16(void);
    static void decodeOpcode17(void);
    static void decodeOpcode18(void);
    static void decodeOpcode19(void);
    static void decodeOpcode1A(void);
    static void decodeOpcode1B(void);
    static void decodeOpcode1C(void);
    static void decodeOpcode1D(void);
    static void decodeOpcode1E(void);
    static void decodeOpcode1F(void);

    static void decodeOpcode20(void);
    static void decodeOpcode21(void);
    static void decodeOpcode22(void);
    static void decodeOpcode23(void);
    static void decodeOpcode24(void);
    static void decodeOpcode25(void);
    static void decodeOpcode26(void);
    static void decodeOpcode27(void);
    static void decodeOpcode28(void);
    static void decodeOpcode29(void);
    static void decodeOpcode2A(void);
    static void decodeOpcode2B(void);
    static void decodeOpcode2C(void);
    static void decodeOpcode2D(void);
    static void decodeOpcode2E(void);
    static void decodeOpcode2F(void);

    static void decodeOpcode30(void);
    static void decodeOpcode31(void);
    static void decodeOpcode32(void);
    static void decodeOpcode33(void);
    static void decodeOpcode34(void);
    static void decodeOpcode35(void);
    static void decodeOpcode36(void);
    static void decodeOpcode37(void);
    static void decodeOpcode38(void);
    static void decodeOpcode39(void);
    static void decodeOpcode3A(void);
    static void decodeOpcode3B(void);
    static void decodeOpcode3C(void);
    static void decodeOpcode3D(void);
    static void decodeOpcode3E(void);
    static void decodeOpcode3F(void);

    static void decodeOpcode40(void);
    static void decodeOpcode41(void);
    static void decodeOpcode42(void);
    static void decodeOpcode43(void);
    static void decodeOpcode44(void);
    static void decodeOpcode45(void);
    static void decodeOpcode46(void);
    static void decodeOpcode47(void);
    static void decodeOpcode48(void);
    static void decodeOpcode49(void);
    static void decodeOpcode4A(void);
    static void decodeOpcode4B(void);
    static void decodeOpcode4C(void);
    static void decodeOpcode4D(void);
    static void decodeOpcode4E(void);
    static void decodeOpcode4F(void);

    static void decodeOpcode50(void);
    static void decodeOpcode51(void);
    static void decodeOpcode52(void);
    static void decodeOpcode53(void);
    static void decodeOpcode54(void);
    static void decodeOpcode55(void);
    static void decodeOpcode56(void);
    static void decodeOpcode57(void);
    static void decodeOpcode58(void);
    static void decodeOpcode59(void);
    static void decodeOpcode5A(void);
    static void decodeOpcode5B(void);
    static void decodeOpcode5C(void);
    static void decodeOpcode5D(void);
    static void decodeOpcode5E(void);
    static void decodeOpcode5F(void);

    static void decodeOpcode60(void);
    static void decodeOpcode61(void);
    static void decodeOpcode62(void);
    static void decodeOpcode63(void);
    static void decodeOpcode64(void);
    static void decodeOpcode65(void);
    static void decodeOpcode66(void);
    static void decodeOpcode67(void);
    static void decodeOpcode68(void);
    static void decodeOpcode69(void);
    static void decodeOpcode6A(void);
    static void decodeOpcode6B(void);
    static void decodeOpcode6C(void);
    static void decodeOpcode6D(void);
    static void decodeOpcode6E(void);
    static void decodeOpcode6F(void);

    static void decodeOpcode70(void);
    static void decodeOpcode71(void);
    static void decodeOpcode72(void);
    static void decodeOpcode73(void);
    static void decodeOpcode74(void);
    static void decodeOpcode75(void);
    static void decodeOpcode76(void);
    static void decodeOpcode77(void);
    static void decodeOpcode78(void);
    static void decodeOpcode79(void);
    static void decodeOpcode7A(void);
    static void decodeOpcode7B(void);
    static void decodeOpcode7C(void);
    static void decodeOpcode7D(void);
    static void decodeOpcode7E(void);
    static void decodeOpcode7F(void);

    static void decodeOpcode80(void);
    static void decodeOpcode81(void);
    static void decodeOpcode82(void);
    static void decodeOpcode83(void);
    static void decodeOpcode84(void);
    static void decodeOpcode85(void);
    static void decodeOpcode86(void);
    static void decodeOpcode87(void);
    static void decodeOpcode88(void);
    static void decodeOpcode89(void);
    static void decodeOpcode8A(void);
    static void decodeOpcode8B(void);
    static void decodeOpcode8C(void);
    static void decodeOpcode8D(void);
    static void decodeOpcode8E(void);
    static void decodeOpcode8F(void);

    static void decodeOpcode90(void);
    static void decodeOpcode91(void);
    static void decodeOpcode92(void);
    static void decodeOpcode93(void);
    static void decodeOpcode94(void);
    static void decodeOpcode95(void);
    static void decodeOpcode96(void);
    static void decodeOpcode97(void);
    static void decodeOpcode98(void);
    static void decodeOpcode99(void);
    static void decodeOpcode9A(void);
    static void decodeOpcode9B(void);
    static void decodeOpcode9C(void);
    static void decodeOpcode9D(void);
    static void decodeOpcode9E(void);
    static void decodeOpcode9F(void);

    static void decodeOpcodeA0(void);
    static void decodeOpcodeA1(void);
    static void decodeOpcodeA2(void);
    static void decodeOpcodeA3(void);
    static void decodeOpcodeA4(void);
    static void decodeOpcodeA5(void);
    static void decodeOpcodeA6(void);
    static void decodeOpcodeA7(void);
    static void decodeOpcodeA8(void);
    static void decodeOpcodeA9(void);
    static void decodeOpcodeAA(void);
    static void decodeOpcodeAB(void);
    static void decodeOpcodeAC(void);
    static void decodeOpcodeAD(void);
    static void decodeOpcodeAE(void);
    static void decodeOpcodeAF(void);

    static void decodeOpcodeB0(void);
    static void decodeOpcodeB1(void);
    static void decodeOpcodeB2(void);
    static void decodeOpcodeB3(void);
    static void decodeOpcodeB4(void);
    static void decodeOpcodeB5(void);
    static void decodeOpcodeB6(void);
    static void decodeOpcodeB7(void);
    static void decodeOpcodeB8(void);
    static void decodeOpcodeB9(void);
    static void decodeOpcodeBA(void);
    static void decodeOpcodeBB(void);
    static void decodeOpcodeBC(void);
    static void decodeOpcodeBD(void);
    static void decodeOpcodeBE(void);

    static void decodeOpcodeBF(void); // Used for LOAD TRAP

    static void decodeOpcodeC0(void);
    static void decodeOpcodeC1(void);
    static void decodeOpcodeC2(void);
    static void decodeOpcodeC3(void);
    static void decodeOpcodeC4(void);
    static void decodeOpcodeC5(void);
    static void decodeOpcodeC6(void);
    static void decodeOpcodeC7(void);
    static void decodeOpcodeC8(void);
    static void decodeOpcodeC9(void);
    static void decodeOpcodeCA(void);
    static void decodeOpcodeCB(void);
    static void decodeOpcodeCC(void);
    static void decodeOpcodeCD(void);
    static void decodeOpcodeCE(void);
    static void decodeOpcodeCF(void);

    static void decodeOpcodeD0(void);
    static void decodeOpcodeD1(void);
    static void decodeOpcodeD2(void);
    static void decodeOpcodeD3(void);
    static void decodeOpcodeD4(void);
    static void decodeOpcodeD5(void);
    static void decodeOpcodeD6(void);
    static void decodeOpcodeD7(void);
    static void decodeOpcodeD8(void);
    static void decodeOpcodeD9(void);
    static void decodeOpcodeDA(void);
    static void decodeOpcodeDB(void);
    static void decodeOpcodeDC(void);
    static void decodeOpcodeDD(void);
    static void decodeOpcodeDE(void);
    static void decodeOpcodeDF(void);

    static void decodeOpcodeE0(void);
    static void decodeOpcodeE1(void);
    static void decodeOpcodeE2(void);
    static void decodeOpcodeE3(void);
    static void decodeOpcodeE4(void);
    static void decodeOpcodeE5(void);
    static void decodeOpcodeE6(void);
    static void decodeOpcodeE7(void);
    static void decodeOpcodeE8(void);
    static void decodeOpcodeE9(void);
    static void decodeOpcodeEA(void);
    static void decodeOpcodeEB(void);
    static void decodeOpcodeEC(void);
    static void decodeOpcodeED(void);
    static void decodeOpcodeEE(void);
    static void decodeOpcodeEF(void);

    static void decodeOpcodeF0(void);
    static void decodeOpcodeF1(void);
    static void decodeOpcodeF2(void);
    static void decodeOpcodeF3(void);
    static void decodeOpcodeF4(void);
    static void decodeOpcodeF5(void);
    static void decodeOpcodeF6(void);
    static void decodeOpcodeF7(void);
    static void decodeOpcodeF8(void);
    static void decodeOpcodeF9(void);
    static void decodeOpcodeFA(void);
    static void decodeOpcodeFB(void);
    static void decodeOpcodeFC(void);
    static void decodeOpcodeFD(void);
    static void decodeOpcodeFE(void);
    static void decodeOpcodeFF(void);

    static void dcCB00(void);
    static void dcCB01(void);
    static void dcCB02(void);
    static void dcCB03(void);
    static void dcCB04(void);
    static void dcCB05(void);
    static void dcCB06(void);
    static void dcCB07(void);
    static void dcCB08(void);
    static void dcCB09(void);
    static void dcCB0A(void);
    static void dcCB0B(void);
    static void dcCB0C(void);
    static void dcCB0D(void);
    static void dcCB0E(void);
    static void dcCB0F(void);

    static void dcCB10(void);
    static void dcCB11(void);
    static void dcCB12(void);
    static void dcCB13(void);
    static void dcCB14(void);
    static void dcCB15(void);
    static void dcCB16(void);
    static void dcCB17(void);
    static void dcCB18(void);
    static void dcCB19(void);
    static void dcCB1A(void);
    static void dcCB1B(void);
    static void dcCB1C(void);
    static void dcCB1D(void);
    static void dcCB1E(void);
    static void dcCB1F(void);

    static void dcCB20(void);
    static void dcCB21(void);
    static void dcCB22(void);
    static void dcCB23(void);
    static void dcCB24(void);
    static void dcCB25(void);
    static void dcCB26(void);
    static void dcCB27(void);
    static void dcCB28(void);
    static void dcCB29(void);
    static void dcCB2A(void);
    static void dcCB2B(void);
    static void dcCB2C(void);
    static void dcCB2D(void);
    static void dcCB2E(void);
    static void dcCB2F(void);

    static void dcCB30(void);
    static void dcCB31(void);
    static void dcCB32(void);
    static void dcCB33(void);
    static void dcCB34(void);
    static void dcCB35(void);
    static void dcCB36(void);
    static void dcCB37(void);
    static void dcCB38(void);
    static void dcCB39(void);
    static void dcCB3A(void);
    static void dcCB3B(void);
    static void dcCB3C(void);
    static void dcCB3D(void);
    static void dcCB3E(void);
    static void dcCB3F(void);

    static void dcCB40(void);
    static void dcCB41(void);
    static void dcCB42(void);
    static void dcCB43(void);
    static void dcCB44(void);
    static void dcCB45(void);
    static void dcCB46(void);
    static void dcCB47(void);
    static void dcCB48(void);
    static void dcCB49(void);
    static void dcCB4A(void);
    static void dcCB4B(void);
    static void dcCB4C(void);
    static void dcCB4D(void);
    static void dcCB4E(void);
    static void dcCB4F(void);

    static void dcCB50(void);
    static void dcCB51(void);
    static void dcCB52(void);
    static void dcCB53(void);
    static void dcCB54(void);
    static void dcCB55(void);
    static void dcCB56(void);
    static void dcCB57(void);
    static void dcCB58(void);
    static void dcCB59(void);
    static void dcCB5A(void);
    static void dcCB5B(void);
    static void dcCB5C(void);
    static void dcCB5D(void);
    static void dcCB5E(void);
    static void dcCB5F(void);

    static void dcCB60(void);
    static void dcCB61(void);
    static void dcCB62(void);
    static void dcCB63(void);
    static void dcCB64(void);
    static void dcCB65(void);
    static void dcCB66(void);
    static void dcCB67(void);
    static void dcCB68(void);
    static void dcCB69(void);
    static void dcCB6A(void);
    static void dcCB6B(void);
    static void dcCB6C(void);
    static void dcCB6D(void);
    static void dcCB6E(void);
    static void dcCB6F(void);

    static void dcCB70(void);
    static void dcCB71(void);
    static void dcCB72(void);
    static void dcCB73(void);
    static void dcCB74(void);
    static void dcCB75(void);
    static void dcCB76(void);
    static void dcCB77(void);
    static void dcCB78(void);
    static void dcCB79(void);
    static void dcCB7A(void);
    static void dcCB7B(void);
    static void dcCB7C(void);
    static void dcCB7D(void);
    static void dcCB7E(void);
    static void dcCB7F(void);

    static void dcCB80(void);
    static void dcCB81(void);
    static void dcCB82(void);
    static void dcCB83(void);
    static void dcCB84(void);
    static void dcCB85(void);
    static void dcCB86(void);
    static void dcCB87(void);
    static void dcCB88(void);
    static void dcCB89(void);
    static void dcCB8A(void);
    static void dcCB8B(void);
    static void dcCB8C(void);
    static void dcCB8D(void);
    static void dcCB8E(void);
    static void dcCB8F(void);

    static void dcCB90(void);
    static void dcCB91(void);
    static void dcCB92(void);
    static void dcCB93(void);
    static void dcCB94(void);
    static void dcCB95(void);
    static void dcCB96(void);
    static void dcCB97(void);
    static void dcCB98(void);
    static void dcCB99(void);
    static void dcCB9A(void);
    static void dcCB9B(void);
    static void dcCB9C(void);
    static void dcCB9D(void);
    static void dcCB9E(void);
    static void dcCB9F(void);

    static void dcCBA0(void);
    static void dcCBA1(void);
    static void dcCBA2(void);
    static void dcCBA3(void);
    static void dcCBA4(void);
    static void dcCBA5(void);
    static void dcCBA6(void);
    static void dcCBA7(void);
    static void dcCBA8(void);
    static void dcCBA9(void);
    static void dcCBAA(void);
    static void dcCBAB(void);
    static void dcCBAC(void);
    static void dcCBAD(void);
    static void dcCBAE(void);
    static void dcCBAF(void);

    static void dcCBB0(void);
    static void dcCBB1(void);
    static void dcCBB2(void);
    static void dcCBB3(void);
    static void dcCBB4(void);
    static void dcCBB5(void);
    static void dcCBB6(void);
    static void dcCBB7(void);
    static void dcCBB8(void);
    static void dcCBB9(void);
    static void dcCBBA(void);
    static void dcCBBB(void);
    static void dcCBBC(void);
    static void dcCBBD(void);
    static void dcCBBE(void);
    static void dcCBBF(void);

    static void dcCBC0(void);
    static void dcCBC1(void);
    static void dcCBC2(void);
    static void dcCBC3(void);
    static void dcCBC4(void);
    static void dcCBC5(void);
    static void dcCBC6(void);
    static void dcCBC7(void);
    static void dcCBC8(void);
    static void dcCBC9(void);
    static void dcCBCA(void);
    static void dcCBCB(void);
    static void dcCBCC(void);
    static void dcCBCD(void);
    static void dcCBCE(void);
    static void dcCBCF(void);

    static void dcCBD0(void);
    static void dcCBD1(void);
    static void dcCBD2(void);
    static void dcCBD3(void);
    static void dcCBD4(void);
    static void dcCBD5(void);
    static void dcCBD6(void);
    static void dcCBD7(void);
    static void dcCBD8(void);
    static void dcCBD9(void);
    static void dcCBDA(void);
    static void dcCBDB(void);
    static void dcCBDC(void);
    static void dcCBDD(void);
    static void dcCBDE(void);
    static void dcCBDF(void);

    static void dcCBE0(void);
    static void dcCBE1(void);
    static void dcCBE2(void);
    static void dcCBE3(void);
    static void dcCBE4(void);
    static void dcCBE5(void);
    static void dcCBE6(void);
    static void dcCBE7(void);
    static void dcCBE8(void);
    static void dcCBE9(void);
    static void dcCBEA(void);
    static void dcCBEB(void);
    static void dcCBEC(void);
    static void dcCBED(void);
    static void dcCBEE(void);
    static void dcCBEF(void);

    static void dcCBF0(void);
    static void dcCBF1(void);
    static void dcCBF2(void);
    static void dcCBF3(void);
    static void dcCBF4(void);
    static void dcCBF5(void);
    static void dcCBF6(void);
    static void dcCBF7(void);
    static void dcCBF8(void);
    static void dcCBF9(void);
    static void dcCBFA(void);
    static void dcCBFB(void);
    static void dcCBFC(void);
    static void dcCBFD(void);
    static void dcCBFE(void);
    static void dcCBFF(void);

    static void check_trdos();
    // static void check_trdos_unpage();
};

#include "z80operations.h"

#endif // Z80CPP_H
