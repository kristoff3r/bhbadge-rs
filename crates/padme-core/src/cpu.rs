use crate::bus::Bus;
use crate::interrupt::InterruptFlag;
use crate::region::*;

pub const CLOCK_SPEED: u32 = 4194304;

// Vector table
const IR_VBLANK_ADDR: u16 = 0x0040;
const IR_LCDC_STATUS_ADDR: u16 = 0x0048;
const IR_TIMER_OVERFLOW_ADDR: u16 = 0x0050;
const IR_SERIAL_TRANSFER_ADDR: u16 = 0x0058;
const IR_JOYPAD_PRESS_ADDR: u16 = 0x0060;

// Flags for register F
const FLAG_ZERO: u8 = 0x80;
const FLAG_SUBSTRACT: u8 = 0x40;
const FLAG_HALF_CARRY: u8 = 0x20;
const FLAG_CARRY: u8 = 0x10;

// Default register values
const DEFAULT_REG_A: u8 = 0x01;
const DEFAULT_REG_F: u8 = 0xB0;
const DEFAULT_REG_B: u8 = 0x00;
const DEFAULT_REG_C: u8 = 0x13;
const DEFAULT_REG_D: u8 = 0x00;
const DEFAULT_REG_E: u8 = 0xD8;
const DEFAULT_REG_H: u8 = 0x01;
const DEFAULT_REG_L: u8 = 0x4D;

const DEFAULT_SP: u16 = 0xFFFE;
const DEFAULT_PC: u16 = 0x0100;

pub struct Cpu {
    // Registers
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    pc: u16,
    sp: u16,
    // CPU halted
    halted: bool,
    // CPU stopped until button is pressed
    stopped: bool,
    // Master Interrupt Enable
    master_ie: bool,
    enabling_ie: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            a: DEFAULT_REG_A,
            f: DEFAULT_REG_F,
            b: DEFAULT_REG_B,
            c: DEFAULT_REG_C,
            d: DEFAULT_REG_D,
            e: DEFAULT_REG_E,
            h: DEFAULT_REG_H,
            l: DEFAULT_REG_L,
            sp: DEFAULT_SP,
            pc: DEFAULT_PC,
            halted: false,
            stopped: false,
            master_ie: true,
            enabling_ie: false,
        }
    }

    fn af(&self) -> u16 {
        make_u16!(self.a, self.f)
    }

    fn bc(&self) -> u16 {
        make_u16!(self.b, self.c)
    }

    fn de(&self) -> u16 {
        make_u16!(self.d, self.e)
    }

    fn hl(&self) -> u16 {
        make_u16!(self.h, self.l)
    }

    fn set_af(&mut self, value: u16) {
        self.a = (value >> 8) as u8;
        self.f = value as u8;
    }

    fn set_bc(&mut self, value: u16) {
        self.b = (value >> 8) as u8;
        self.c = value as u8;
    }

    fn set_de(&mut self, value: u16) {
        self.d = (value >> 8) as u8;
        self.e = value as u8;
    }

    fn set_hl(&mut self, value: u16) {
        self.h = (value >> 8) as u8;
        self.l = value as u8;
    }

    fn inc_hl(&mut self) {
        let hl = self.hl();
        self.set_hl(hl.wrapping_add(1));
    }

    fn dec_hl(&mut self) {
        let hl = self.hl();
        self.set_hl(hl.wrapping_sub(1));
    }

    /// Set or reset flag in F register
    fn set_flag(&mut self, flag: u8, set: bool) {
        if set {
            self.f |= flag;
        } else {
            self.f &= !flag;
        }
    }

    fn set_all_flags(&mut self, zero: bool, subtract: bool, carry: bool, half_carry: bool) {
        let mut f = 0;
        if zero {
            f |= FLAG_ZERO;
        }
        if subtract {
            f |= FLAG_SUBSTRACT;
        }
        if carry {
            f |= FLAG_CARRY;
        }
        if half_carry {
            f |= FLAG_HALF_CARRY;
        }
        self.f = f;
    }

    /// Retrieve next byte
    fn fetch(&mut self, bus: &Bus<'_>) -> u8 {
        let byte = bus.read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        byte
    }

    /// Retrieve next 2 bytes as a u16
    fn fetch16(&mut self, bus: &Bus<'_>) -> u16 {
        let l = self.fetch(bus);
        let h = self.fetch(bus);
        make_u16!(h, l)
    }

    /// Put SP + n into HL
    fn ld_hl_spn(&mut self, bus: &Bus<'_>) {
        let n = self.fetch(bus);
        let res = (self.sp as i32).wrapping_add((n as i8) as i32) as u16;

        self.set_all_flags(
            false,
            false,
            (res & 0xff) < (self.sp & 0xff),
            (res & 0xf) < (self.sp & 0xf),
        );
        self.set_hl(res);
    }

    /// PUSH element on top of the stack
    fn push(&mut self, bus: &mut Bus<'_>, value: u16) {
        self.sp = self.sp.wrapping_sub(1);
        bus.write(self.sp, (value >> 8) as u8);
        self.sp = self.sp.wrapping_sub(1);
        bus.write(self.sp, value as u8);
    }

    /// POP top element of the stack
    fn pop(&mut self, bus: &Bus<'_>) -> u16 {
        let l = bus.read(self.sp);
        self.sp = self.sp.wrapping_add(1);
        let h = bus.read(self.sp);
        self.sp = self.sp.wrapping_add(1);

        ((h as u16) << 8) | l as u16
    }

    /// Add value to register A with provided carry
    fn addc(&mut self, value: u8, carry: u8) {
        let (r, overflow1) = self.a.overflowing_add(value);
        let (r, overflow2) = r.overflowing_add(carry);
        self.set_all_flags(
            r == 0,
            false,
            overflow1 | overflow2,
            (self.a & 0xF) + (value & 0xF) + carry > 0xF,
        );
        self.a = r;
    }

    /// Add value to register A, no carry
    fn add(&mut self, value: u8) {
        self.addc(value, 0u8)
    }

    /// Add value to register A using carry flag
    fn adc(&mut self, value: u8) {
        self.addc(value, (self.f & FLAG_CARRY == FLAG_CARRY) as u8)
    }

    /// Subtract value to register A with provided carry
    fn subc(&mut self, value: u8, carry: u8) -> u8 {
        let (n, overflow1) = self.a.overflowing_sub(value);
        let (n, overflow2) = n.overflowing_sub(carry);

        let hc = ((self.a & 0xF) as i16 - (value & 0xF) as i16 - carry as i16) < 0x0;
        self.set_all_flags(n == 0, true, overflow1 | overflow2, hc);
        n
    }

    /// Subtract value to register A no carry
    fn sub(&mut self, value: u8) {
        self.a = self.subc(value, 0u8)
    }

    /// Subtract value to register A using carry flag
    fn sbc(&mut self, value: u8) {
        self.a = self.subc(value, (self.f & FLAG_CARRY != 0) as u8)
    }

    /// Logical AND value with register A
    fn and(&mut self, value: u8) {
        self.a &= value;
        self.set_all_flags(self.a == 0, false, false, true);
    }

    /// Logical OR value with register A
    fn or(&mut self, value: u8) {
        self.a |= value;
        self.set_all_flags(self.a == 0, false, false, false);
    }

    /// XOR value with register A
    fn xor(&mut self, value: u8) {
        self.a ^= value;
        self.set_all_flags(self.a == 0, false, false, false);
    }

    /// Compare A with value
    fn cp(&mut self, value: u8) {
        self.subc(value, 0); // discard result
    }

    /// Increment a register value
    fn inc(&mut self, value: u8) -> u8 {
        let r = value.wrapping_add(1);
        self.set_flag(FLAG_ZERO, r == 0);
        self.set_flag(FLAG_SUBSTRACT, false);
        self.set_flag(FLAG_HALF_CARRY, (r & 0xf) < (value & 0xf));
        r
    }

    /// Decrement a register value
    fn dec(&mut self, value: u8) -> u8 {
        let r = value.wrapping_sub(1);
        self.set_flag(FLAG_ZERO, r == 0);
        self.set_flag(FLAG_SUBSTRACT, true);
        self.set_flag(FLAG_HALF_CARRY, (r & 0xf) > (value & 0xf));
        r
    }

    /// Add value to HL register
    fn add16(&mut self, n: u16) {
        let hl = self.hl();
        let result = hl.wrapping_add(n);
        self.set_flag(FLAG_CARRY, hl > 0xFFFF - n);
        self.set_flag(
            FLAG_HALF_CARRY,
            (((hl & 0xFFF) + (n & 0xFFF)) & 0x1000) != 0,
        );
        self.set_flag(FLAG_SUBSTRACT, false);
        self.set_hl(result);
    }

    /// Swap upper & lower nibbles of value
    fn swap(&mut self, value: u8) -> u8 {
        let r = (value << 4) | (value >> 4);
        self.set_all_flags(r == 0, false, false, false);

        r
    }

    /// Decimal adjust register A
    /// Adjust A to get a BCD representation of A
    fn daa(&mut self) {
        let mut adj = 0x00;
        let is_sub = (self.f & FLAG_SUBSTRACT) == FLAG_SUBSTRACT;
        let mut is_carry = false;

        if (self.f & FLAG_HALF_CARRY) != 0 || (!is_sub && (self.a & 0x0F) > 0x09) {
            adj |= 0x06;
        }
        if (self.f & FLAG_CARRY) != 0 || (!is_sub && self.a > 0x99) {
            adj |= 0x60;
            is_carry = true;
        }

        if is_sub {
            self.a = self.a.wrapping_sub(adj);
        } else {
            self.a = self.a.wrapping_add(adj);
        }
        self.set_flag(FLAG_ZERO, self.a == 0);
        self.set_flag(FLAG_HALF_CARRY, false);
        self.set_flag(FLAG_CARRY, is_carry);
    }

    /// Complement
    /// Flip all bits of register A
    fn cpl(&mut self) {
        self.a = !self.a;
        self.set_flag(FLAG_SUBSTRACT, true);
        self.set_flag(FLAG_HALF_CARRY, true);
    }

    /// Complement carry flag
    fn ccf(&mut self) {
        self.f ^= FLAG_CARRY;
        self.set_flag(FLAG_SUBSTRACT, false);
        self.set_flag(FLAG_HALF_CARRY, false);
    }

    /// Set carry flag
    fn scf(&mut self) {
        self.set_flag(FLAG_CARRY, true);
        self.set_flag(FLAG_SUBSTRACT, false);
        self.set_flag(FLAG_HALF_CARRY, false);
    }

    /// Rotate A left with old bit 7 to carry flag
    fn rl(&mut self, n: u8, with_carry: bool, set_zero: bool) -> u8 {
        let res = if with_carry {
            let carry_bit = ((self.f & FLAG_CARRY) == FLAG_CARRY) as u8;
            (n << 1) | carry_bit
        } else {
            n.rotate_left(1)
        };
        self.set_all_flags(set_zero && res == 0, false, (n >> 7) == 0x01, false);
        res
    }

    /// Rotate A right with old bit 7 to carry flag
    fn rr(&mut self, n: u8, with_carry: bool, set_zero: bool) -> u8 {
        let res = if with_carry {
            let carry_bit = ((self.f & FLAG_CARRY) == FLAG_CARRY) as u8;
            (carry_bit << 7) | (n >> 1)
        } else {
            n.rotate_right(1)
        };
        self.set_all_flags(set_zero && res == 0, false, (n & 0x01) == 0x01, false);
        res
    }

    /// Shift n left into carry
    fn sl(&mut self, n: u8) -> u8 {
        let res = n << 1;
        self.set_all_flags(res == 0, false, (n >> 7) == 0x01, false);
        res
    }

    /// Shift n right into carry
    fn sr(&mut self, n: u8, keep_msb: bool) -> u8 {
        let res = if keep_msb {
            (n & 0x80) | (n >> 1)
        } else {
            n >> 1
        };
        self.set_all_flags(res == 0, false, (n & 0x01) == 0x01, false);
        res
    }

    /// Test if bit b is set in n
    fn bit(&mut self, n: u8, b: u8) {
        self.set_flag(FLAG_ZERO, (n & b) == 0);
        self.set_flag(FLAG_SUBSTRACT, false);
        self.set_flag(FLAG_HALF_CARRY, true);
    }

    /// Jump to address if flag is set / reset
    fn jump_if(&mut self, address: u16, condition: bool) -> u8 {
        if condition {
            self.pc = address;
            16
        } else {
            12
        }
    }

    /// Jump to pc + n if flag is set / reset
    fn jump_if_rel(&mut self, n: u8, condition: bool) -> u8 {
        if condition {
            self.pc = ((self.pc as i32) + ((n as i8) as i32)) as u16;
            12
        } else {
            8
        }
    }

    /// Save PC and jump to address
    fn call(&mut self, bus: &mut Bus<'_>, address: u16) {
        self.push(bus, self.pc);
        self.pc = address;
    }

    /// Save PC and jump to address if condition is true
    fn call_if(&mut self, bus: &mut Bus<'_>, nn: u16, condition: bool) -> u8 {
        if condition {
            self.call(bus, nn);
            24
        } else {
            12
        }
    }

    /// Return if condition is true
    fn ret_if(&mut self, bus: &Bus<'_>, condition: bool) -> u8 {
        if condition {
            self.pc = self.pop(bus);
            20
        } else {
            8
        }
    }

    /// Decode the provided op code and execute the instruction
    fn decode_execute(&mut self, bus: &mut Bus<'_>, op: u8) -> u8 {
        match op {
            // --- Misc
            // NOP
            0x00 => 4,
            // DAA
            0x27 => {
                self.daa();
                4
            }
            // CPL
            0x2F => {
                self.cpl();
                4
            }
            // SCF
            0x37 => {
                self.scf();
                4
            }
            // CCF
            0x3F => {
                self.ccf();
                4
            }
            // HALT
            0x76 => {
                self.halted = true;
                4
            }
            // STOP
            0x10 => {
                self.fetch(bus);
                self.stopped = true;
                4
            }
            // --- LD
            // LD BC, nn
            0x01 => {
                let nn = self.fetch16(bus);
                self.set_bc(nn);
                12
            }
            // LD DE, nn
            0x11 => {
                let nn = self.fetch16(bus);
                self.set_de(nn);
                12
            }
            // LD HL, nn
            0x21 => {
                let nn = self.fetch16(bus);
                self.set_hl(nn);
                12
            }
            // LD SP, nn
            0x31 => {
                let nn = self.fetch16(bus);
                self.sp = nn;
                12
            }
            // LD r, n
            0x06 => {
                self.b = self.fetch(bus);
                8
            }
            0x0E => {
                self.c = self.fetch(bus);
                8
            }
            0x16 => {
                self.d = self.fetch(bus);
                8
            }
            0x1E => {
                self.e = self.fetch(bus);
                8
            }
            0x26 => {
                self.h = self.fetch(bus);
                8
            }
            0x2E => {
                self.l = self.fetch(bus);
                8
            }
            0x3E => {
                self.a = self.fetch(bus);
                8
            }
            0x40..=0x7F => {
                let mut count = 4;
                let value = match op & 0x7 {
                    0 => self.b,
                    1 => self.c,
                    2 => self.d,
                    3 => self.e,
                    4 => self.h,
                    5 => self.l,
                    6 => {
                        count += 4;
                        bus.read(self.hl())
                    }
                    _ => self.a,
                };
                match (op >> 3) & 0x7 {
                    0 => {
                        self.b = value;
                    }
                    1 => {
                        self.c = value;
                    }
                    2 => {
                        self.d = value;
                    }
                    3 => {
                        self.e = value;
                    }
                    4 => {
                        self.h = value;
                    }
                    5 => {
                        self.l = value;
                    }
                    6 => {
                        count += 4;
                        bus.write(self.hl(), value);
                    }
                    _ => {
                        self.a = value;
                    }
                }
                count
            }
            0x80..=0xBF => {
                let mut count = 4;
                let value = match op & 0x7 {
                    0 => self.b,
                    1 => self.c,
                    2 => self.d,
                    3 => self.e,
                    4 => self.h,
                    5 => self.l,
                    6 => {
                        count += 4;
                        bus.read(self.hl())
                    }
                    _ => self.a,
                };
                match (op >> 3) & 0x7 {
                    0 => {
                        self.add(value);
                    }
                    1 => {
                        self.adc(value);
                    }
                    2 => {
                        self.sub(value);
                    }
                    3 => {
                        self.sbc(value);
                    }
                    4 => {
                        self.and(value);
                    }
                    5 => {
                        self.xor(value);
                    }
                    6 => {
                        self.or(value);
                    }
                    _ => {
                        self.cp(value);
                    }
                }

                count
            }
            // LD A, (HL+)
            0x2A => {
                self.a = bus.read(self.hl());
                self.inc_hl();
                8
            }
            // LD A, (HL-)
            0x3A => {
                self.a = bus.read(self.hl());
                self.dec_hl();
                8
            }
            // LD A, (BC)
            0x0A => {
                self.a = bus.read(self.bc());
                8
            }
            // LD A, (DE)
            0x1A => {
                self.a = bus.read(self.de());
                8
            }
            // LD A, (nn)
            0xFA => {
                let nn = self.fetch16(bus);
                self.a = bus.read(nn);
                16
            }
            // LD (nn), A
            0xEA => {
                let nn = self.fetch16(bus);
                bus.write(nn, self.a);
                16
            }
            // LD (HL), n
            0x36 => {
                let n = self.fetch(bus);
                bus.write(self.hl(), n);
                12
            }
            // LD (HL+), A
            0x22 => {
                bus.write(self.hl(), self.a);
                self.inc_hl();
                8
            }
            // LD (HL-), A
            0x32 => {
                bus.write(self.hl(), self.a);
                self.dec_hl();
                8
            }
            // LD (BC), A
            0x02 => {
                bus.write(self.bc(), self.a);
                8
            }
            // LD (DE), A
            0x12 => {
                bus.write(self.de(), self.a);
                8
            }
            // LD ($FF00 + C), A
            0xE2 => {
                bus.write(IO_REGION_START + self.c as u16, self.a);
                8
            }
            // LD A, ($FF00 + C)
            0xF2 => {
                self.a = bus.read(IO_REGION_START + self.c as u16);
                8
            }
            // LD ($FF00 + n), A
            0xE0 => {
                let n = self.fetch(bus);
                bus.write(IO_REGION_START + n as u16, self.a);
                12
            }
            // LD A, ($FF00 + n)
            0xF0 => {
                let n = self.fetch(bus);
                self.a = bus.read(IO_REGION_START + n as u16);
                12
            }
            // LD HL, SP+n
            0xF8 => {
                self.ld_hl_spn(bus);
                12
            }
            // LD (nn), SP
            0x08 => {
                let nn = self.fetch16(bus);
                bus.write(nn, self.sp as u8);
                bus.write(nn + 1, (self.sp >> 8) as u8);
                20
            }
            // LD SP, HL
            0xF9 => {
                self.sp = self.hl();
                8
            }
            // ---
            // PUSH rr
            0xF5 => {
                self.push(bus, self.af());
                16
            }
            0xC5 => {
                self.push(bus, self.bc());
                16
            }
            0xD5 => {
                self.push(bus, self.de());
                16
            }
            0xE5 => {
                self.push(bus, self.hl());
                16
            }
            // POP rr
            0xF1 => {
                let rr = self.pop(bus);
                self.set_af(rr & 0xFFF0);
                12
            }
            0xC1 => {
                let rr = self.pop(bus);
                self.set_bc(rr);
                12
            }
            0xD1 => {
                let rr = self.pop(bus);
                self.set_de(rr);
                12
            }
            0xE1 => {
                let rr = self.pop(bus);
                self.set_hl(rr);
                12
            }
            // ---
            // JP nn
            0xC3 => {
                let nn = self.fetch16(bus);
                self.pc = nn;
                12
            }
            // JP cc, nn
            0xC2 => {
                let nn = self.fetch16(bus);
                self.jump_if(nn, (self.f & FLAG_ZERO) == 0)
            }
            0xCA => {
                let nn = self.fetch16(bus);
                self.jump_if(nn, (self.f & FLAG_ZERO) == FLAG_ZERO)
            }
            0xD2 => {
                let nn = self.fetch16(bus);
                self.jump_if(nn, (self.f & FLAG_CARRY) == 0)
            }
            0xDA => {
                let nn = self.fetch16(bus);
                self.jump_if(nn, (self.f & FLAG_CARRY) == FLAG_CARRY)
            }
            // JP HL
            0xE9 => {
                self.pc = self.hl();
                4
            }
            // JR n
            0x18 => {
                let n = self.fetch(bus);
                self.pc = ((self.pc as i32) + ((n as i8) as i32)) as u16;
                8
            }
            // JR cc, n
            0x20 => {
                let n = self.fetch(bus);
                self.jump_if_rel(n, (self.f & FLAG_ZERO) == 0)
            }
            0x28 => {
                let n = self.fetch(bus);
                self.jump_if_rel(n, (self.f & FLAG_ZERO) == FLAG_ZERO)
            }
            0x30 => {
                let n = self.fetch(bus);
                self.jump_if_rel(n, (self.f & FLAG_CARRY) == 0)
            }
            0x38 => {
                let n = self.fetch(bus);
                self.jump_if_rel(n, (self.f & FLAG_CARRY) == FLAG_CARRY)
            }
            // CALL nn
            0xCD => {
                let nn = self.fetch16(bus);
                self.call(bus, nn);
                24
            }
            // CALL cc, nn
            0xC4 => {
                let nn = self.fetch16(bus);
                self.call_if(bus, nn, (self.f & FLAG_ZERO) == 0)
            }
            0xCC => {
                let nn = self.fetch16(bus);
                self.call_if(bus, nn, (self.f & FLAG_ZERO) == FLAG_ZERO)
            }
            0xD4 => {
                let nn = self.fetch16(bus);
                self.call_if(bus, nn, (self.f & FLAG_CARRY) == 0)
            }
            0xDC => {
                let nn = self.fetch16(bus);
                self.call_if(bus, nn, (self.f & FLAG_CARRY) == FLAG_CARRY)
            }
            // RST n
            0xC7 => {
                self.call(bus, 0x00u16);
                16
            }
            0xCF => {
                self.call(bus, 0x08u16);
                16
            }
            0xD7 => {
                self.call(bus, 0x10u16);
                16
            }
            0xDF => {
                self.call(bus, 0x18u16);
                16
            }
            0xE7 => {
                self.call(bus, 0x20u16);
                16
            }
            0xEF => {
                self.call(bus, 0x28u16);
                16
            }
            0xF7 => {
                self.call(bus, 0x30u16);
                16
            }
            0xFF => {
                self.call(bus, 0x38u16);
                16
            }
            // RET
            0xC9 => {
                self.pc = self.pop(bus);
                16
            }
            // RET cc
            0xC0 => self.ret_if(bus, (self.f & FLAG_ZERO) == 0),
            0xC8 => self.ret_if(bus, (self.f & FLAG_ZERO) == FLAG_ZERO),
            0xD0 => self.ret_if(bus, (self.f & FLAG_CARRY) == 0),
            0xD8 => self.ret_if(bus, (self.f & FLAG_CARRY) == FLAG_CARRY),
            // RETI
            0xD9 => {
                self.pc = self.pop(bus);
                self.master_ie = true;
                8
            }
            // --- 8-bit arithmetic
            // ADD A, n
            0xC6 => {
                let n = self.fetch(bus);
                self.add(n);
                8
            }
            // ADC A, n
            0xCE => {
                let n = self.fetch(bus);
                self.adc(n);
                8
            }
            // SUB A, n
            0xD6 => {
                let n = self.fetch(bus);
                self.sub(n);
                8
            }
            // SBC A, n
            0xDE => {
                let n = self.fetch(bus);
                self.sbc(n);
                8
            }
            // AND n
            0xE6 => {
                let n = self.fetch(bus);
                self.and(n);
                8
            }
            // OR n
            0xF6 => {
                let n = self.fetch(bus);
                self.or(n);
                8
            }
            // XOR n
            0xEE => {
                let n = self.fetch(bus);
                self.xor(n);
                8
            }
            // CP n
            0xFE => {
                let n = self.fetch(bus);
                self.cp(n);
                8
            }
            // INC n
            0x3C => {
                self.a = self.inc(self.a);
                4
            }
            0x04 => {
                self.b = self.inc(self.b);
                4
            }
            0x0C => {
                self.c = self.inc(self.c);
                4
            }
            0x14 => {
                self.d = self.inc(self.d);
                4
            }
            0x1C => {
                self.e = self.inc(self.e);
                4
            }
            0x24 => {
                self.h = self.inc(self.h);
                4
            }
            0x2C => {
                self.l = self.inc(self.l);
                4
            }
            0x34 => {
                let hl = self.hl();
                let n = bus.read(hl);
                let r = self.inc(n);
                bus.write(hl, r);
                12
            }
            // DEC n
            0x3D => {
                self.a = self.dec(self.a);
                4
            }
            0x05 => {
                self.b = self.dec(self.b);
                4
            }
            0x0D => {
                self.c = self.dec(self.c);
                4
            }
            0x15 => {
                self.d = self.dec(self.d);
                4
            }
            0x1D => {
                self.e = self.dec(self.e);
                4
            }
            0x25 => {
                self.h = self.dec(self.h);
                4
            }
            0x2D => {
                self.l = self.dec(self.l);
                4
            }
            0x35 => {
                let hl = self.hl();
                let n = bus.read(hl);
                let r = self.dec(n);
                bus.write(hl, r);
                12
            }
            // --- 16-bit arithmetic
            // ADD HL, r
            0x09 => {
                self.add16(self.bc());
                8
            }
            0x19 => {
                self.add16(self.de());
                8
            }
            0x29 => {
                self.add16(self.hl());
                8
            }
            0x39 => {
                self.add16(self.sp);
                8
            }
            // ADD SP, n
            0xE8 => {
                let n = self.fetch(bus);
                let r = (self.sp as i32).wrapping_add((n as i8) as i32) as u16;
                self.set_all_flags(
                    false,
                    false,
                    (r & 0xFF) < (self.sp & 0xFF),
                    (r & 0xF) < (self.sp & 0xF),
                );
                self.sp = r as u16;
                16
            }
            // INC rr
            0x03 => {
                let rr = self.bc().wrapping_add(1);
                self.set_bc(rr);
                8
            }
            0x13 => {
                let rr = self.de().wrapping_add(1);
                self.set_de(rr);
                8
            }
            0x23 => {
                let rr = self.hl().wrapping_add(1);
                self.set_hl(rr);
                8
            }
            0x33 => {
                self.sp = self.sp.wrapping_add(1);
                8
            }
            // DEC rr
            0x0B => {
                let rr = self.bc().wrapping_sub(1);
                self.set_bc(rr);
                8
            }
            0x1B => {
                let rr = self.de().wrapping_sub(1);
                self.set_de(rr);
                8
            }
            0x2B => {
                let rr = self.hl().wrapping_sub(1);
                self.set_hl(rr);
                8
            }
            0x3B => {
                self.sp = self.sp.wrapping_sub(1);
                8
            }
            // DI
            0xF3 => {
                self.enabling_ie = false;
                self.master_ie = false;
                4
            }
            // EI
            0xFB => {
                self.enabling_ie = true;
                4
            }
            // Rotates
            0x07 => {
                self.a = self.rl(self.a, false, false);
                4
            }
            0x17 => {
                self.a = self.rl(self.a, true, false);
                4
            }
            0x0F => {
                self.a = self.rr(self.a, false, false);
                4
            }
            0x1F => {
                self.a = self.rr(self.a, true, false);
                4
            }
            // --- CB prefixed commands
            0xCB => {
                let op2 = self.fetch(bus);

                match op2 {
                    // SWAP n
                    0x37 => {
                        self.a = self.swap(self.a);
                        8
                    }
                    0x30 => {
                        self.b = self.swap(self.b);
                        8
                    }
                    0x31 => {
                        self.c = self.swap(self.c);
                        8
                    }
                    0x32 => {
                        self.d = self.swap(self.d);
                        8
                    }
                    0x33 => {
                        self.e = self.swap(self.e);
                        8
                    }
                    0x34 => {
                        self.h = self.swap(self.h);
                        8
                    }
                    0x35 => {
                        self.l = self.swap(self.l);
                        8
                    }
                    0x36 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let r = self.swap(n);
                        bus.write(hl, r);
                        16
                    }
                    // RLC n
                    0x07 => {
                        self.a = self.rl(self.a, false, true);
                        8
                    }
                    0x00 => {
                        self.b = self.rl(self.b, false, true);
                        8
                    }
                    0x01 => {
                        self.c = self.rl(self.c, false, true);
                        8
                    }
                    0x02 => {
                        self.d = self.rl(self.d, false, true);
                        8
                    }
                    0x03 => {
                        self.e = self.rl(self.e, false, true);
                        8
                    }
                    0x04 => {
                        self.h = self.rl(self.h, false, true);
                        8
                    }
                    0x05 => {
                        self.l = self.rl(self.l, false, true);
                        8
                    }
                    0x06 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.rl(n, false, true);
                        bus.write(hl, res);
                        16
                    }
                    // RL n
                    0x17 => {
                        self.a = self.rl(self.a, true, true);
                        8
                    }
                    0x10 => {
                        self.b = self.rl(self.b, true, true);
                        8
                    }
                    0x11 => {
                        self.c = self.rl(self.c, true, true);
                        8
                    }
                    0x12 => {
                        self.d = self.rl(self.d, true, true);
                        8
                    }
                    0x13 => {
                        self.e = self.rl(self.e, true, true);
                        8
                    }
                    0x14 => {
                        self.h = self.rl(self.h, true, true);
                        8
                    }
                    0x15 => {
                        self.l = self.rl(self.l, true, true);
                        8
                    }
                    0x16 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.rl(n, true, true);
                        bus.write(hl, res);
                        16
                    }
                    // RRC n
                    0x0F => {
                        self.a = self.rr(self.a, false, true);
                        8
                    }
                    0x08 => {
                        self.b = self.rr(self.b, false, true);
                        8
                    }
                    0x09 => {
                        self.c = self.rr(self.c, false, true);
                        8
                    }
                    0x0A => {
                        self.d = self.rr(self.d, false, true);
                        8
                    }
                    0x0B => {
                        self.e = self.rr(self.e, false, true);
                        8
                    }
                    0x0C => {
                        self.h = self.rr(self.h, false, true);
                        8
                    }
                    0x0D => {
                        self.l = self.rr(self.l, false, true);
                        8
                    }
                    0x0E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.rr(n, false, true);
                        bus.write(hl, res);
                        16
                    }
                    // RRC n
                    0x1F => {
                        self.a = self.rr(self.a, true, true);
                        8
                    }
                    0x18 => {
                        self.b = self.rr(self.b, true, true);
                        8
                    }
                    0x19 => {
                        self.c = self.rr(self.c, true, true);
                        8
                    }
                    0x1A => {
                        self.d = self.rr(self.d, true, true);
                        8
                    }
                    0x1B => {
                        self.e = self.rr(self.e, true, true);
                        8
                    }
                    0x1C => {
                        self.h = self.rr(self.h, true, true);
                        8
                    }
                    0x1D => {
                        self.l = self.rr(self.l, true, true);
                        8
                    }
                    0x1E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.rr(n, true, true);
                        bus.write(hl, res);
                        16
                    }
                    // SLA n
                    0x27 => {
                        self.a = self.sl(self.a);
                        8
                    }
                    0x20 => {
                        self.b = self.sl(self.b);
                        8
                    }
                    0x21 => {
                        self.c = self.sl(self.c);
                        8
                    }
                    0x22 => {
                        self.d = self.sl(self.d);
                        8
                    }
                    0x23 => {
                        self.e = self.sl(self.e);
                        8
                    }
                    0x24 => {
                        self.h = self.sl(self.h);
                        8
                    }
                    0x25 => {
                        self.l = self.sl(self.l);
                        8
                    }
                    0x26 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.sl(n);
                        bus.write(hl, res);
                        16
                    }
                    // SRA n
                    0x2F => {
                        self.a = self.sr(self.a, true);
                        8
                    }
                    0x28 => {
                        self.b = self.sr(self.b, true);
                        8
                    }
                    0x29 => {
                        self.c = self.sr(self.c, true);
                        8
                    }
                    0x2A => {
                        self.d = self.sr(self.d, true);
                        8
                    }
                    0x2B => {
                        self.e = self.sr(self.e, true);
                        8
                    }
                    0x2C => {
                        self.h = self.sr(self.h, true);
                        8
                    }
                    0x2D => {
                        self.l = self.sr(self.l, true);
                        8
                    }
                    0x2E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.sr(n, true);
                        bus.write(hl, res);
                        16
                    }
                    // SRL n
                    0x3F => {
                        self.a = self.sr(self.a, false);
                        8
                    }
                    0x38 => {
                        self.b = self.sr(self.b, false);
                        8
                    }
                    0x39 => {
                        self.c = self.sr(self.c, false);
                        8
                    }
                    0x3A => {
                        self.d = self.sr(self.d, false);
                        8
                    }
                    0x3B => {
                        self.e = self.sr(self.e, false);
                        8
                    }
                    0x3C => {
                        self.h = self.sr(self.h, false);
                        8
                    }
                    0x3D => {
                        self.l = self.sr(self.l, false);
                        8
                    }
                    0x3E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        let res = self.sr(n, false);
                        bus.write(hl, res);
                        16
                    }
                    // BIT 0, r
                    0x47 => {
                        self.bit(self.a, 0x01);
                        8
                    }
                    0x40 => {
                        self.bit(self.b, 0x01);
                        8
                    }
                    0x41 => {
                        self.bit(self.c, 0x01);
                        8
                    }
                    0x42 => {
                        self.bit(self.d, 0x01);
                        8
                    }
                    0x43 => {
                        self.bit(self.e, 0x01);
                        8
                    }
                    0x44 => {
                        self.bit(self.h, 0x01);
                        8
                    }
                    0x45 => {
                        self.bit(self.l, 0x01);
                        8
                    }
                    0x46 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01);
                        16
                    }
                    // BIT 1, r
                    0x4F => {
                        self.bit(self.a, 0x01 << 1);
                        8
                    }
                    0x48 => {
                        self.bit(self.b, 0x01 << 1);
                        8
                    }
                    0x49 => {
                        self.bit(self.c, 0x01 << 1);
                        8
                    }
                    0x4A => {
                        self.bit(self.d, 0x01 << 1);
                        8
                    }
                    0x4B => {
                        self.bit(self.e, 0x01 << 1);
                        8
                    }
                    0x4C => {
                        self.bit(self.h, 0x01 << 1);
                        8
                    }
                    0x4D => {
                        self.bit(self.l, 0x01 << 1);
                        8
                    }
                    0x4E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 1);
                        16
                    }
                    // BIT 2, r
                    0x57 => {
                        self.bit(self.a, 0x01 << 2);
                        8
                    }
                    0x50 => {
                        self.bit(self.b, 0x01 << 2);
                        8
                    }
                    0x51 => {
                        self.bit(self.c, 0x01 << 2);
                        8
                    }
                    0x52 => {
                        self.bit(self.d, 0x01 << 2);
                        8
                    }
                    0x53 => {
                        self.bit(self.e, 0x01 << 2);
                        8
                    }
                    0x54 => {
                        self.bit(self.h, 0x01 << 2);
                        8
                    }
                    0x55 => {
                        self.bit(self.l, 0x01 << 2);
                        8
                    }
                    0x56 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 2);
                        16
                    }
                    // BIT 3, r
                    0x5F => {
                        self.bit(self.a, 0x01 << 3);
                        8
                    }
                    0x58 => {
                        self.bit(self.b, 0x01 << 3);
                        8
                    }
                    0x59 => {
                        self.bit(self.c, 0x01 << 3);
                        8
                    }
                    0x5A => {
                        self.bit(self.d, 0x01 << 3);
                        8
                    }
                    0x5B => {
                        self.bit(self.e, 0x01 << 3);
                        8
                    }
                    0x5C => {
                        self.bit(self.h, 0x01 << 3);
                        8
                    }
                    0x5D => {
                        self.bit(self.l, 0x01 << 3);
                        8
                    }
                    0x5E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 3);
                        16
                    }
                    // BIT 4, r
                    0x67 => {
                        self.bit(self.a, 0x01 << 4);
                        8
                    }
                    0x60 => {
                        self.bit(self.b, 0x01 << 4);
                        8
                    }
                    0x61 => {
                        self.bit(self.c, 0x01 << 4);
                        8
                    }
                    0x62 => {
                        self.bit(self.d, 0x01 << 4);
                        8
                    }
                    0x63 => {
                        self.bit(self.e, 0x01 << 4);
                        8
                    }
                    0x64 => {
                        self.bit(self.h, 0x01 << 4);
                        8
                    }
                    0x65 => {
                        self.bit(self.l, 0x01 << 4);
                        8
                    }
                    0x66 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 4);
                        16
                    }
                    // BIT 5, r
                    0x6F => {
                        self.bit(self.a, 0x01 << 5);
                        8
                    }
                    0x68 => {
                        self.bit(self.b, 0x01 << 5);
                        8
                    }
                    0x69 => {
                        self.bit(self.c, 0x01 << 5);
                        8
                    }
                    0x6A => {
                        self.bit(self.d, 0x01 << 5);
                        8
                    }
                    0x6B => {
                        self.bit(self.e, 0x01 << 5);
                        8
                    }
                    0x6C => {
                        self.bit(self.h, 0x01 << 5);
                        8
                    }
                    0x6D => {
                        self.bit(self.l, 0x01 << 5);
                        8
                    }
                    0x6E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 5);
                        16
                    }
                    // BIT 6, r
                    0x77 => {
                        self.bit(self.a, 0x01 << 6);
                        8
                    }
                    0x70 => {
                        self.bit(self.b, 0x01 << 6);
                        8
                    }
                    0x71 => {
                        self.bit(self.c, 0x01 << 6);
                        8
                    }
                    0x72 => {
                        self.bit(self.d, 0x01 << 6);
                        8
                    }
                    0x73 => {
                        self.bit(self.e, 0x01 << 6);
                        8
                    }
                    0x74 => {
                        self.bit(self.h, 0x01 << 6);
                        8
                    }
                    0x75 => {
                        self.bit(self.l, 0x01 << 6);
                        8
                    }
                    0x76 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 6);
                        16
                    }
                    // BIT 7, r
                    0x7F => {
                        self.bit(self.a, 0x01 << 7);
                        8
                    }
                    0x78 => {
                        self.bit(self.b, 0x01 << 7);
                        8
                    }
                    0x79 => {
                        self.bit(self.c, 0x01 << 7);
                        8
                    }
                    0x7A => {
                        self.bit(self.d, 0x01 << 7);
                        8
                    }
                    0x7B => {
                        self.bit(self.e, 0x01 << 7);
                        8
                    }
                    0x7C => {
                        self.bit(self.h, 0x01 << 7);
                        8
                    }
                    0x7D => {
                        self.bit(self.l, 0x01 << 7);
                        8
                    }
                    0x7E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        self.bit(n, 0x01 << 7);
                        16
                    }
                    // RES 0, r
                    0x87 => {
                        self.a &= !0x01;
                        8
                    }
                    0x80 => {
                        self.b &= !0x01;
                        8
                    }
                    0x81 => {
                        self.c &= !0x01;
                        8
                    }
                    0x82 => {
                        self.d &= !0x01;
                        8
                    }
                    0x83 => {
                        self.e &= !0x01;
                        8
                    }
                    0x84 => {
                        self.h &= !0x01;
                        8
                    }
                    0x85 => {
                        self.l &= !0x01;
                        8
                    }
                    0x86 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !0x01);
                        16
                    }
                    // RES 1, r
                    0x8F => {
                        self.a &= !(0x01 << 1);
                        8
                    }
                    0x88 => {
                        self.b &= !(0x01 << 1);
                        8
                    }
                    0x89 => {
                        self.c &= !(0x01 << 1);
                        8
                    }
                    0x8A => {
                        self.d &= !(0x01 << 1);
                        8
                    }
                    0x8B => {
                        self.e &= !(0x01 << 1);
                        8
                    }
                    0x8C => {
                        self.h &= !(0x01 << 1);
                        8
                    }
                    0x8D => {
                        self.l &= !(0x01 << 1);
                        8
                    }
                    0x8E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 1));
                        16
                    }
                    // RES 2, r
                    0x97 => {
                        self.a &= !(0x01 << 2);
                        8
                    }
                    0x90 => {
                        self.b &= !(0x01 << 2);
                        8
                    }
                    0x91 => {
                        self.c &= !(0x01 << 2);
                        8
                    }
                    0x92 => {
                        self.d &= !(0x01 << 2);
                        8
                    }
                    0x93 => {
                        self.e &= !(0x01 << 2);
                        8
                    }
                    0x94 => {
                        self.h &= !(0x01 << 2);
                        8
                    }
                    0x95 => {
                        self.l &= !(0x01 << 2);
                        8
                    }
                    0x96 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 2));
                        16
                    }
                    // RES 3, r
                    0x9F => {
                        self.a &= !(0x01 << 3);
                        8
                    }
                    0x98 => {
                        self.b &= !(0x01 << 3);
                        8
                    }
                    0x99 => {
                        self.c &= !(0x01 << 3);
                        8
                    }
                    0x9A => {
                        self.d &= !(0x01 << 3);
                        8
                    }
                    0x9B => {
                        self.e &= !(0x01 << 3);
                        8
                    }
                    0x9C => {
                        self.h &= !(0x01 << 3);
                        8
                    }
                    0x9D => {
                        self.l &= !(0x01 << 3);
                        8
                    }
                    0x9E => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 3));
                        16
                    }
                    // RES 4, r
                    0xA7 => {
                        self.a &= !(0x01 << 4);
                        8
                    }
                    0xA0 => {
                        self.b &= !(0x01 << 4);
                        8
                    }
                    0xA1 => {
                        self.c &= !(0x01 << 4);
                        8
                    }
                    0xA2 => {
                        self.d &= !(0x01 << 4);
                        8
                    }
                    0xA3 => {
                        self.e &= !(0x01 << 4);
                        8
                    }
                    0xA4 => {
                        self.h &= !(0x01 << 4);
                        8
                    }
                    0xA5 => {
                        self.l &= !(0x01 << 4);
                        8
                    }
                    0xA6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 4));
                        16
                    }
                    // RES 5, r
                    0xAF => {
                        self.a &= !(0x01 << 5);
                        8
                    }
                    0xA8 => {
                        self.b &= !(0x01 << 5);
                        8
                    }
                    0xA9 => {
                        self.c &= !(0x01 << 5);
                        8
                    }
                    0xAA => {
                        self.d &= !(0x01 << 5);
                        8
                    }
                    0xAB => {
                        self.e &= !(0x01 << 5);
                        8
                    }
                    0xAC => {
                        self.h &= !(0x01 << 5);
                        8
                    }
                    0xAD => {
                        self.l &= !(0x01 << 5);
                        8
                    }
                    0xAE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 5));
                        16
                    }
                    // RES 6, r
                    0xB7 => {
                        self.a &= !(0x01 << 6);
                        8
                    }
                    0xB0 => {
                        self.b &= !(0x01 << 6);
                        8
                    }
                    0xB1 => {
                        self.c &= !(0x01 << 6);
                        8
                    }
                    0xB2 => {
                        self.d &= !(0x01 << 6);
                        8
                    }
                    0xB3 => {
                        self.e &= !(0x01 << 6);
                        8
                    }
                    0xB4 => {
                        self.h &= !(0x01 << 6);
                        8
                    }
                    0xB5 => {
                        self.l &= !(0x01 << 6);
                        8
                    }
                    0xB6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 6));
                        16
                    }
                    // RES 7, r
                    0xBF => {
                        self.a &= !(0x01 << 7);
                        8
                    }
                    0xB8 => {
                        self.b &= !(0x01 << 7);
                        8
                    }
                    0xB9 => {
                        self.c &= !(0x01 << 7);
                        8
                    }
                    0xBA => {
                        self.d &= !(0x01 << 7);
                        8
                    }
                    0xBB => {
                        self.e &= !(0x01 << 7);
                        8
                    }
                    0xBC => {
                        self.h &= !(0x01 << 7);
                        8
                    }
                    0xBD => {
                        self.l &= !(0x01 << 7);
                        8
                    }
                    0xBE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n & !(0x01 << 7));
                        16
                    }
                    // SET 0, r
                    0xC7 => {
                        self.a |= 0x01;
                        8
                    }
                    0xC0 => {
                        self.b |= 0x01;
                        8
                    }
                    0xC1 => {
                        self.c |= 0x01;
                        8
                    }
                    0xC2 => {
                        self.d |= 0x01;
                        8
                    }
                    0xC3 => {
                        self.e |= 0x01;
                        8
                    }
                    0xC4 => {
                        self.h |= 0x01;
                        8
                    }
                    0xC5 => {
                        self.l |= 0x01;
                        8
                    }
                    0xC6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | 0x01);
                        16
                    }
                    // SET 1, r
                    0xCF => {
                        self.a |= 0x01 << 1;
                        8
                    }
                    0xC8 => {
                        self.b |= 0x01 << 1;
                        8
                    }
                    0xC9 => {
                        self.c |= 0x01 << 1;
                        8
                    }
                    0xCA => {
                        self.d |= 0x01 << 1;
                        8
                    }
                    0xCB => {
                        self.e |= 0x01 << 1;
                        8
                    }
                    0xCC => {
                        self.h |= 0x01 << 1;
                        8
                    }
                    0xCD => {
                        self.l |= 0x01 << 1;
                        8
                    }
                    0xCE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 1));
                        16
                    }
                    // SET 2, r
                    0xD7 => {
                        self.a |= 0x01 << 2;
                        8
                    }
                    0xD0 => {
                        self.b |= 0x01 << 2;
                        8
                    }
                    0xD1 => {
                        self.c |= 0x01 << 2;
                        8
                    }
                    0xD2 => {
                        self.d |= 0x01 << 2;
                        8
                    }
                    0xD3 => {
                        self.e |= 0x01 << 2;
                        8
                    }
                    0xD4 => {
                        self.h |= 0x01 << 2;
                        8
                    }
                    0xD5 => {
                        self.l |= 0x01 << 2;
                        8
                    }
                    0xD6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 2));
                        16
                    }
                    // SET 3, r
                    0xDF => {
                        self.a |= 0x01 << 3;
                        8
                    }
                    0xD8 => {
                        self.b |= 0x01 << 3;
                        8
                    }
                    0xD9 => {
                        self.c |= 0x01 << 3;
                        8
                    }
                    0xDA => {
                        self.d |= 0x01 << 3;
                        8
                    }
                    0xDB => {
                        self.e |= 0x01 << 3;
                        8
                    }
                    0xDC => {
                        self.h |= 0x01 << 3;
                        8
                    }
                    0xDD => {
                        self.l |= 0x01 << 3;
                        8
                    }
                    0xDE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 3));
                        16
                    }
                    // SET 4, r
                    0xE7 => {
                        self.a |= 0x01 << 4;
                        8
                    }
                    0xE0 => {
                        self.b |= 0x01 << 4;
                        8
                    }
                    0xE1 => {
                        self.c |= 0x01 << 4;
                        8
                    }
                    0xE2 => {
                        self.d |= 0x01 << 4;
                        8
                    }
                    0xE3 => {
                        self.e |= 0x01 << 4;
                        8
                    }
                    0xE4 => {
                        self.h |= 0x01 << 4;
                        8
                    }
                    0xE5 => {
                        self.l |= 0x01 << 4;
                        8
                    }
                    0xE6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 4));
                        16
                    }
                    // SET 5, r
                    0xEF => {
                        self.a |= 0x01 << 5;
                        8
                    }
                    0xE8 => {
                        self.b |= 0x01 << 5;
                        8
                    }
                    0xE9 => {
                        self.c |= 0x01 << 5;
                        8
                    }
                    0xEA => {
                        self.d |= 0x01 << 5;
                        8
                    }
                    0xEB => {
                        self.e |= 0x01 << 5;
                        8
                    }
                    0xEC => {
                        self.h |= 0x01 << 5;
                        8
                    }
                    0xED => {
                        self.l |= 0x01 << 5;
                        8
                    }
                    0xEE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 5));
                        16
                    }
                    // SET 6, r
                    0xF7 => {
                        self.a |= 0x01 << 6;
                        8
                    }
                    0xF0 => {
                        self.b |= 0x01 << 6;
                        8
                    }
                    0xF1 => {
                        self.c |= 0x01 << 6;
                        8
                    }
                    0xF2 => {
                        self.d |= 0x01 << 6;
                        8
                    }
                    0xF3 => {
                        self.e |= 0x01 << 6;
                        8
                    }
                    0xF4 => {
                        self.h |= 0x01 << 6;
                        8
                    }
                    0xF5 => {
                        self.l |= 0x01 << 6;
                        8
                    }
                    0xF6 => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 6));
                        16
                    }
                    // SET 7, r
                    0xFF => {
                        self.a |= 0x01 << 7;
                        8
                    }
                    0xF8 => {
                        self.b |= 0x01 << 7;
                        8
                    }
                    0xF9 => {
                        self.c |= 0x01 << 7;
                        8
                    }
                    0xFA => {
                        self.d |= 0x01 << 7;
                        8
                    }
                    0xFB => {
                        self.e |= 0x01 << 7;
                        8
                    }
                    0xFC => {
                        self.h |= 0x01 << 7;
                        8
                    }
                    0xFD => {
                        self.l |= 0x01 << 7;
                        8
                    }
                    0xFE => {
                        let hl = self.hl();
                        let n = bus.read(hl);
                        bus.write(hl, n | (0x01 << 7));
                        16
                    }
                }
            }
            // Unknown op code
            _ => {
                defmt::panic!("Unknown op code 0x{:02X}", op);
            }
        }
    }

    /// Reset all registers & state
    pub fn reset(&mut self) {
        self.a = DEFAULT_REG_A;
        self.f = DEFAULT_REG_F;
        self.b = DEFAULT_REG_B;
        self.c = DEFAULT_REG_C;
        self.d = DEFAULT_REG_D;
        self.e = DEFAULT_REG_E;
        self.h = DEFAULT_REG_H;
        self.l = DEFAULT_REG_L;
        self.sp = DEFAULT_SP;
        self.pc = DEFAULT_PC;
        self.halted = false;
        self.stopped = false;
        self.master_ie = true;
        self.enabling_ie = false;
    }

    /// Fetch, decode and execute next instruction
    /// Returns the number of ticks
    pub fn step(&mut self, bus: &mut Bus<'_>) -> u8 {
        let ticks = if !self.halted {
            // Fetch instruction
            let op = self.fetch(bus);
            // Decode & execute
            self.decode_execute(bus, op)
        } else {
            let pending_it = bus.read(REG_IF_ADDR);
            if pending_it != 0 {
                self.halted = false;
            }
            // If CPU is halted, we assume 4 cycles and return
            4
        };

        // Check for interrupts
        if self.master_ie {
            let int_enable = bus.read(REG_IE_ADDR);
            let int_flag = bus.read(REG_IF_ADDR);

            macro_rules! handle_interrupt {
                ($f:expr, $addr:expr) => {
                    if (int_enable & ($f as u8)) != 0 && (int_flag & ($f as u8)) != 0 {
                        self.call(bus, $addr);
                        bus.it.clear($f);
                        self.halted = false;
                        self.master_ie = false;
                        true
                    } else {
                        false
                    }
                };
            }

            let _ = handle_interrupt!(InterruptFlag::Vblank, IR_VBLANK_ADDR)
                || handle_interrupt!(InterruptFlag::Lcdc, IR_LCDC_STATUS_ADDR)
                || handle_interrupt!(InterruptFlag::TimerOverflow, IR_TIMER_OVERFLOW_ADDR)
                || handle_interrupt!(InterruptFlag::Serial, IR_SERIAL_TRANSFER_ADDR)
                || handle_interrupt!(InterruptFlag::Joypad, IR_JOYPAD_PRESS_ADDR);
        }

        // Enable / Disable interrupt if requested, after 1 instruction
        if self.enabling_ie {
            self.master_ie = true;
        }

        ticks
    }
}
