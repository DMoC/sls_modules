/* -*- c++ -*-
 *
 * SOCLIB_LGPL_HEADER_BEGIN
 * 
 * This file is part of SoCLib, GNU LGPLv2.1.
 * 
 * SoCLib is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * SoCLib is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with SoCLib; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 * 
 * SOCLIB_LGPL_HEADER_END
 *
 * Copyright (c) UPMC, Lip6
 *    Nicolas Pouillon <nipo@ssji.net>, 2008
 *
 * Maintainers: nipo
 *
 * $Id$
 */

#include "mips32.h"
#include "base_module.h"
#include "soclib_endian.h"
#include "arithmetics.h"
#include <cstring>
#include <cstdlib>

namespace soclib { namespace common {

Mips32Iss::Mips32Iss(const std::string &name, uint32_t ident, bool default_little_endian)
    : Iss2(name, ident),
      m_little_endian(default_little_endian)
{
    r_config.whole = 0;
    r_config.m = 1;
    r_config.be = m_little_endian ? 0 : 1;
    r_config.ar = 1;
    r_config.mt = 7; // Reserved, let's say it's soclib generic MMU :)

    r_config1.whole = 0;
    r_config1.m = 1;
    r_config1.c2 = 1; // Advertize for Cop2 presence, i.e. generic MMU access

    r_config2.whole = 0;
    r_config2.m = 1;

    r_config3.whole = 0;
    r_config3.ulri = 1; // Advertize for TLS register
}

void Mips32Iss::reset()
{
    struct DataRequest null_dreq = ISS_DREQ_INITIALIZER;
    r_ebase = 0x80000000 | m_ident;
    r_pc = RESET_ADDRESS;
    r_cpu_mode = MIPS32_KERNEL;
    r_npc = RESET_ADDRESS + 4;
    m_ibe = false;
    m_dbe = false;
    m_dreq = null_dreq;
    r_mem_dest = 0;
    m_skip_next_instruction = false;
    m_ins_delay = 0;
    r_status.whole = 0x400004;
    r_cause.whole = 0;
    m_exec_cycles = 0;
    r_gp[0] = 0;
    m_sleeping = false;
    r_count = 0;
    r_compare = 0;
    r_tls_base = 0;
    r_hwrena = 0;

    r_bus_mode = MODE_KERNEL;

    for(int i = 0; i<32; i++)
        r_gp[i] = 0;

    m_hazard=false;
    m_exception = NO_EXCEPTION;
    update_mode();
}

void Mips32Iss::dump() const
{
    std::cout
        << std::hex << std::showbase
        << m_name
        << " PC: " << r_pc
        << " NPC: " << r_npc
        << " Ins: " << m_ins.ins << std::endl
        << std::dec
        << " Cause.xcode: " << r_cause.xcode << std::endl
        << " Mode: " << r_cpu_mode
        << " Status.ksu " << r_status.ksu
        << " .exl: " << r_status.exl
        << " .erl: " << r_status.erl
        << " .whole: " << std::hex << r_status.whole
        << std::endl
        << " op:  " << m_ins.i.op << " (" << name_table[m_ins.i.op] << ")" << std::endl
        << " i rs: " << m_ins.i.rs
        << " rt: "<<m_ins.i.rt
        << " i: "<<std::hex << m_ins.i.imd
        << std::endl << std::dec
        << " r rs: " << m_ins.r.rs
        << " rt: "<<m_ins.r.rt
        << " rd: "<<m_ins.r.rd
        << " sh: "<<m_ins.r.sh << std::hex
        << " func: "<<m_ins.r.func
        << std::endl
        << " V rs: " << r_gp[m_ins.i.rs]
        << " rt: "<<r_gp[m_ins.i.rt]
        << std::endl;
    for ( size_t i=0; i<32; ++i ) {
        std::cout << " " << std::dec << i << ": " << std::hex << std::showbase << r_gp[i];
        if ( i%8 == 7 )
            std::cout << std::endl;
    }
}

uint32_t Mips32Iss::executeNCycles(
    uint32_t ncycle,
    const struct InstructionResponse &irsp,
    const struct DataResponse &drsp,
    uint32_t irq_bit_field )
{
#ifdef SOCLIB_MODULE_DEBUG
    std::cout << name() << " executeNCycles( " << ncycle << ", "<< irsp << ", " << drsp << ", " << irq_bit_field << ")" << std::endl;
#endif

    bool may_take_irq = r_status.ie && !r_status.exl && !r_status.erl;
    if ( m_little_endian )
        m_ins.ins = irsp.instruction;
    else
        m_ins.ins = soclib::endian::uint32_swap(irsp.instruction);
    m_ibe = irsp.error;
    m_ireq_ok = irsp.valid;

    _setData( drsp );

    m_exception = NO_EXCEPTION;

    if ( m_sleeping ) {
        if ( ((r_status.im>>2) & irq_bit_field)
             && may_take_irq ) {
            m_exception = X_INT;
            m_sleeping = false;
#ifdef SOCLIB_MODULE_DEBUG
            std::cout << name() << " IRQ while sleeping" << std::endl;
#endif
        } else {
            r_count += ncycle;
            return ncycle;
        }
    }
    if ( ! m_ireq_ok || ! m_dreq_ok || m_ins_delay ) {
        uint32_t t = ncycle;
        if ( m_ins_delay ) {
            if ( m_ins_delay < ncycle )
                t = m_ins_delay;
            m_ins_delay -= t;
        }
        m_hazard = false;
        r_count += t;
#ifdef SOCLIB_MODULE_DEBUG
        std::cout << name() << " Frozen " << m_ireq_ok << m_dreq_ok<< " " << m_ins_delay << std::endl;
#endif
        return t;
    }

    if ( m_hazard && ncycle > 1 ) {
        ncycle = 2;
        m_hazard = false;
    } else {
        ncycle = 1;
    }
    r_count += ncycle;

    // The current instruction is executed in case of interrupt, but
    // the next instruction will be delayed.
    // The current instruction is not executed in case of exception,
    // and there is three types of bus error events,
    // 1 - instruction bus errors are synchronous events, signaled in
    // the m_ibe variable
    // 2 - read data bus errors are synchronous events, signaled in
    // the m_dbe variable
    // 3 - write data bus errors are asynchonous events signaled in
    // the m_dbe flip-flop
    // Instuction bus errors are related to the current instruction:
    // lowest priority.
    // Read Data bus errors are related to the previous instruction:
    // intermediatz priority
    // Write Data bus error are related to an older instruction:
    // highest priority

    m_next_pc = r_npc+4;

    if (m_ibe) {
        m_exception = X_IBE;
        goto handle_except;
    }

    if ( m_dbe ) {
        m_exception = X_DBE;
        r_bar = m_dreq.addr;
        m_dbe = false;
        goto handle_except;
    }

#ifdef SOCLIB_MODULE_DEBUG
    dump();
#endif
    // run() can modify the following registers: r_gp[i], r_mem_req,
    // r_mem_type, m_dreq.addr; r_mem_wdata, r_mem_dest, r_hi, r_lo,
    // m_exception, m_next_pc
    if ( m_hazard ) {
#ifdef SOCLIB_MODULE_DEBUG
        std::cout << name() << " hazard, seeing next cycle" << std::endl;
#endif
        m_hazard = false;
        goto house_keeping;
    } else {
        m_exec_cycles++;
        run();
    }

    if ( m_exception == NO_EXCEPTION
         && ((r_status.im>>2) & irq_bit_field)
         && may_take_irq ) {
        m_exception = X_INT;
#ifdef SOCLIB_MODULE_DEBUG
        std::cout << name() << " Taking irqs " << irq_bit_field << std::endl;
    } else {
        if ( irq_bit_field )
            std::cout
                << name()
                << " Ignoring irqs " << std::hex << irq_bit_field
                << " cause " << r_cause.whole
                << " status " << r_status.whole
                << " ie " << r_status.ie
                << " exl " << r_status.exl
                << " erl " << r_status.erl
                << " mask " << (r_status.im>>2)
                << " exception " << m_exception
                << std::endl;
#endif
    }

    if (  m_exception == NO_EXCEPTION )
        goto no_except;

 handle_except:

    if ( debugExceptionBypassed( m_exception ) )
        goto no_except;

    {
        addr_t except_address = exceptBaseAddr();
        bool branch_taken = m_next_pc != r_npc+4;

        if ( r_status.exl ) {
            except_address += 0x180;
        } else {
            if ( m_sleeping && m_exception == X_INT ) {
                r_cause.bd = 0;
                r_epc = r_pc;
                m_sleeping = false;
            } else {
                r_cause.bd = branch_taken;
                addr_t epc;
                if ( m_exception == X_DBE ) {
                    // A synchronous DBE is signalled for the
                    // instruction following...
                    // If it is asynchronous, we're lost :'(
                    epc = r_pc-4;
                } else {
                    if ( branch_taken )
                        epc = r_pc;
                    else
                        epc = r_npc;
                }
                r_epc = epc;
            }
            except_address += exceptOffsetAddr(m_exception);
        }
        r_cause.ce = 0;
        r_cause.xcode = m_exception;
        r_cause.ip  = irq_bit_field<<2;
        r_status.exl = 1;
        update_mode();

#ifdef SOCLIB_MODULE_DEBUG
        std::cout
            << m_name <<" exception: "<<m_exception<<std::endl
            << " epc: " << r_epc
            << " error_epc: " << r_error_epc
            << " bar: " << m_dreq.addr
            << " cause.xcode: " << r_cause.xcode
            << " .bd: " << r_cause.bd
            << " .ip: " << r_cause.ip
            << " status.exl: " << r_status.exl
            << " .erl: " << r_status.erl
            << " exception address: " << except_address
            << std::endl;
#endif

        m_next_pc = except_address;
        m_skip_next_instruction = true;
    }
 no_except:
    if (m_skip_next_instruction) {
        r_pc = m_next_pc;
        r_npc = m_next_pc+4;
        m_skip_next_instruction = false;
    } else {
        r_pc = r_npc;
        r_npc = m_next_pc;
    }
 house_keeping:
    r_gp[0] = 0;
    return ncycle;
}

int Mips32Iss::debugCpuCauseToSignal( uint32_t cause ) const
{
    switch (cause) {
    case X_INT:
        return 2; // Interrupt
    case X_MOD:
    case X_TLBL:
    case X_TLBS:
        return 5; // Trap (nothing better)
    case X_ADEL:
    case X_ADES:
    case X_IBE:
    case X_DBE:
        return 11; // SEGV
    case X_SYS:
    case X_BP:
    case X_TR:
    case X_reserved:
        return 5; // Trap/breakpoint
    case X_RI:
    case X_CPU:
        return 4; // Illegal instruction
    case X_OV:
    case X_FPE:
        return 8; // Floating point exception
    };
    return 5;       // GDB SIGTRAP                                                                                                                                                                
}

// CDB
#ifdef CDB_COMPONENT_IF_H
const char *  Mips32Iss::local_GetModel()
{
    return("MIPS32");
}

int Mips32Iss::local_PrintResource(modelResource *res,char **p)
{
int i = 1;
int n;
char *s, *e;

    if (p==NULL){
        switch (res->usr1){
            case 1 : 
            std::cout << " $pc = 0x" << std::hex << r_pc << std::endl;
            return 0;
            break;
            case 2 :
            std::cout << " $v0 = 0x" << std::hex << debugGetRegisterValue(2) << std::endl;
            return 0;
            break;
            case 3 :
            std::cout << " $v1 = 0x" << std::hex << debugGetRegisterValue(3) << std::endl;
            return 0;
            break;
            case 4 :
            std::cout << " $a0 = 0x" << std::hex << debugGetRegisterValue(4) << std::endl;
            return 0;
            break;
            case 5 :
            std::cout << " $a1 = 0x" << std::hex << debugGetRegisterValue(5) << std::endl;
            return 0;
            break;
            case 6 :
            std::cout << " $a2 = 0x" << std::hex << debugGetRegisterValue(6) << std::endl;
            return 0;
            break;
            case 7 :
            std::cout << " $a3 = 0x" << std::hex << debugGetRegisterValue(7) << std::endl;
            return 0;
            break;
            case 8 :
            std::cout << " $t0 = 0x" << std::hex << debugGetRegisterValue(8) << std::endl;
            return 0;
            break;
            case 9 :
            std::cout << " $t1 = 0x" << std::hex << debugGetRegisterValue(9) << std::endl;
            return 0;
            break;
            case 10 :
            std::cout << " $t2 = 0x" << std::hex << debugGetRegisterValue(10) << std::endl;
            return 0;
            break;
            case 11 :
            std::cout << " $t3 = 0x" << std::hex << debugGetRegisterValue(11) << std::endl;
            return 0;
            break;
            case 12 :
            std::cout << " $t4 = 0x" << std::hex << debugGetRegisterValue(12) << std::endl;
            return 0;
            break;
            case 13 :
            std::cout << " $t5 = 0x" << std::hex << debugGetRegisterValue(13) << std::endl;
            return 0;
            break;
            case 14 :
            std::cout << " $t6 = 0x" << std::hex << debugGetRegisterValue(14) << std::endl;
            return 0;
            break;
            case 15 :
            std::cout << " $t7 = 0x" << std::hex << debugGetRegisterValue(15) << std::endl;
            return 0;
            break;
            case 16 :
            std::cout << " $s0 = 0x" << std::hex << debugGetRegisterValue(16) << std::endl;
            return 0;
            break;
            case 17 :
            std::cout << " $s1 = 0x" << std::hex << debugGetRegisterValue(17) << std::endl;
            return 0;
            break;
            case 18 :
            std::cout << " $s2 = 0x" << std::hex << debugGetRegisterValue(18) << std::endl;
            return 0;
            break;
            case 19 :
            std::cout << " $s3 = 0x" << std::hex << debugGetRegisterValue(19) << std::endl;
            return 0;
            break;
            case 20 :
            std::cout << " $s4 = 0x" << std::hex << debugGetRegisterValue(20) << std::endl;
            return 0;
            break;
            case 21 :
            std::cout << " $s5 = 0x" << std::hex << debugGetRegisterValue(21) << std::endl;
            return 0;
            break;
            case 22 :
            std::cout << " $s6 = 0x" << std::hex << debugGetRegisterValue(22) << std::endl;
            return 0;
            break;
            case 23 :
            std::cout << " $s7 = 0x" << std::hex << debugGetRegisterValue(23) << std::endl;
            return 0;
            break;
            case 24 :
            std::cout << " $t8 = 0x" << std::hex << debugGetRegisterValue(24) << std::endl;
            return 0;
            break;
            case 25 :
            std::cout << " $t9 = 0x" << std::hex << debugGetRegisterValue(25) << std::endl;
            return 0;
            break;
            case 26 :
            std::cout << " $k0 = 0x" << std::hex << debugGetRegisterValue(26) << std::endl;
            return 0;
            break;
            case 27 :
            std::cout << " $k1 = 0x" << std::hex << debugGetRegisterValue(27) << std::endl;
            return 0;
            break;
            case 28 :
            std::cout << " $gp = 0x" << std::hex << debugGetRegisterValue(28) << std::endl;
            return 0;
            break;
            case 29 :
            std::cout << " $sp = 0x" << std::hex << debugGetRegisterValue(29) << std::endl;
            return 0;
            break;
            case 31 :
            std::cout << " $sp = 0x" << std::hex << debugGetRegisterValue(31) << std::endl;
            return 0;
            break;
            case 40 :
            std::cout << " $cause.xcode = 0x" << std::hex << r_cause.xcode << std::endl;
            return 0;
            break;
            default:
            std::cerr << "Error MIPS, print non existing register" << std::endl;
            return -1;
            break;
        } 
    }else{
        if (*p[i] == (char)0) {
            fprintf(stderr, "needs a ressource argument!\n");
            return -1;
        }

        if (!strcasecmp(p[i],"proc")) {
            i++;
            if (*p[i] == (char)0) {
                fprintf(stderr, "needs a ressource argument!\n");
                return -1;
            }
            if (*p[i] == '$') {
                s = p[i] + 1; /* register number or name */
                n = strtol(s, &e, 0);
                if (!strcasecmp(s, "pc")) {
                    res->usr1 = 1;
                    return 0;
                } else if(!strcasecmp(s, "v0")) {
                    res->usr1 = 2;
                    return 0;
                } else if(!strcasecmp(s, "v1")) {
                    res->usr1 = 3;
                    return 0;
                } else if(!strcasecmp(s, "a0")) {
                    res->usr1 = 4;
                    return 0;
                } else if(!strcasecmp(s, "a1")) {
                    res->usr1 = 5;
                    return 0;
                } else if(!strcasecmp(s, "a2")) {
                    res->usr1 = 6;
                    return 0;
                } else if(!strcasecmp(s, "a3")) {
                    res->usr1 = 7;
                    return 0;
                } else if(!strcasecmp(s, "t0")) {
                    res->usr1 = 8;
                    return 0;
                } else if(!strcasecmp(s, "t1")) {
                    res->usr1 = 9;
                    return 0;
                } else if(!strcasecmp(s, "t2")) {
                    res->usr1 = 10;
                    return 0;
                } else if(!strcasecmp(s, "t3")) {
                    res->usr1 = 11;
                    return 0;
                } else if(!strcasecmp(s, "t4")) {
                    res->usr1 = 12;
                    return 0;
                } else if(!strcasecmp(s, "t5")) {
                    res->usr1 = 13;
                    return 0;
                } else if(!strcasecmp(s, "t6")) {
                    res->usr1 = 14;
                    return 0;
                } else if(!strcasecmp(s, "t7")) {
                    res->usr1 = 15;
                    return 0;
                } else if(!strcasecmp(s, "s0")) {
                    res->usr1 = 16;
                    return 0;
                } else if(!strcasecmp(s, "s1")) {
                    res->usr1 = 17;
                    return 0;
                } else if(!strcasecmp(s, "s2")) {
                    res->usr1 = 18;
                    return 0;
                } else if(!strcasecmp(s, "s3")) {
                    res->usr1 = 19;
                    return 0;
                } else if(!strcasecmp(s, "s4")) {
                    res->usr1 = 20;
                    return 0;
                } else if(!strcasecmp(s, "s5")) {
                    res->usr1 = 21;
                    return 0;
                } else if(!strcasecmp(s, "s6")) {
                    res->usr1 = 22;
                    return 0;
                } else if(!strcasecmp(s, "s7")) {
                    res->usr1 = 23;
                    return 0;
                } else if(!strcasecmp(s, "t8")) {
                    res->usr1 = 24;
                    return 0;
                } else if(!strcasecmp(s, "t9")) {
                    res->usr1 = 25;
                    return 0;
                } else if(!strcasecmp(s, "k0")) {
                    res->usr1 = 26;
                    return 0;
                } else if(!strcasecmp(s, "k1")) {
                    res->usr1 = 27;
                    return 0;
                } else if(!strcasecmp(s, "gp")) {
                    res->usr1 = 28;
                    return 0;
                } else if(!strcasecmp(s, "sp")) {
                    res->usr1 = 29;
                    return 0;
                } else if(!strcasecmp(s, "fp")) {
                    res->usr1 = 30;
                    return 0;
                } else if(!strcasecmp(s, "ra")) {
                    res->usr1 = 31;
                    return 0;
                } else if(!strcasecmp(s, "xcode")) {
                    res->usr1 = 40;
                    return 0;
                } else { /* finally */
                    fprintf(stderr, "illegal register name %s\n", s);
                    return -1;
                }
            } else
                fprintf(stderr, "Mips32 No such ressource\n"); // todo fprintf cerr
            return -1;
        }else
                fprintf(stderr, "Mips32 No such ressource\n"); // todo fprintf cerr
        return -1;

    }
}


int Mips32Iss::local_TestResource(modelResource *res,char **p){
    int i = 1;
    int n;
    char *s, *e;
    
        if (*p[i] == (char)0) {
            fprintf(stderr, "needs a ressource argument!\n");
            return -1;
        }

        if (*p[i] == '$') {
            s = p[i] + 1; /* register number or name */
            n = strtol(s, &e, 0);
            if (!strcasecmp(s, "pc")) {
                res->addr = (int*)&r_pc;
                return 0;
            } else if(!strcasecmp(s, "v0")) {
                res->addr = (int*)&r_gp[2];
                return 0;
            } else if(!strcasecmp(s, "v1")) {
                res->addr = (int*)&r_gp[3];
                return 0;
            } else if(!strcasecmp(s, "a0")) {
                res->addr = (int*)&r_gp[4];
                return 0;
            } else if(!strcasecmp(s, "a1")) {
                res->addr = (int*)&r_gp[5];
                return 0;
            } else if(!strcasecmp(s, "a2")) {
                res->addr = (int*)&r_gp[6];
                return 0;
            } else if(!strcasecmp(s, "a3")) {
                res->addr = (int*)&r_gp[7];
                return 0;
            } else if(!strcasecmp(s, "t0")) {
                res->addr = (int*)&r_gp[8];
                return 0;
            } else if(!strcasecmp(s, "t1")) {
                res->addr = (int*)&r_gp[9];
                return 0;
            } else if(!strcasecmp(s, "t2")) {
                res->addr = (int*)&r_gp[10];
                return 0;
            } else if(!strcasecmp(s, "t3")) {
                res->addr = (int*)&r_gp[11];
                return 0;
            } else if(!strcasecmp(s, "t4")) {
                res->addr = (int*)&r_gp[12];
                return 0;
            } else if(!strcasecmp(s, "t5")) {
                res->addr = (int*)&r_gp[13];
                return 0;
            } else if(!strcasecmp(s, "t6")) {
                res->addr = (int*)&r_gp[14];
                return 0;
            } else if(!strcasecmp(s, "t7")) {
                res->addr = (int*)&r_gp[15];
                return 0;
            } else if(!strcasecmp(s, "s0")) {
                res->addr = (int*)&r_gp[16];
                return 0;
            } else if(!strcasecmp(s, "s1")) {
                res->addr = (int*)&r_gp[17];
                return 0;
            } else if(!strcasecmp(s, "s2")) {
                res->addr = (int*)&r_gp[18];
                return 0;
            } else if(!strcasecmp(s, "s3")) {
                res->addr = (int*)&r_gp[19];
                return 0;
            } else if(!strcasecmp(s, "s4")) {
                res->addr = (int*)&r_gp[20];
                return 0;
            } else if(!strcasecmp(s, "s5")) {
                res->addr = (int*)&r_gp[21];
                return 0;
            } else if(!strcasecmp(s, "s6")) {
                res->addr = (int*)&r_gp[22];
                return 0;
            } else if(!strcasecmp(s, "s7")) {
                res->addr = (int*)&r_gp[23];
                return 0;
            } else if(!strcasecmp(s, "t8")) {
                res->addr = (int*)&r_gp[24];
                return 0;
            } else if(!strcasecmp(s, "t9")) {
                res->addr = (int*)&r_gp[25];
                return 0;
            } else if(!strcasecmp(s, "k0")) {
                res->addr = (int*)&r_gp[26];
                return 0;
            } else if(!strcasecmp(s, "k1")) {
                res->addr = (int*)&r_gp[27];
                return 0;
            } else if(!strcasecmp(s, "gp")) {
                res->addr = (int*)&r_gp[28];
                return 0;
            } else if(!strcasecmp(s, "sp")) {
                res->addr = (int*)&r_gp[29];
                return 0;
            } else if(!strcasecmp(s, "fp")) {
                res->addr = (int*)&r_gp[30];
                return 0;
            } else if(!strcasecmp(s, "ra")) {
                res->addr = (int*)&r_gp[31];
                return 0;
            }else { /* finally */
                fprintf(stderr, "illegal register name %s\n", s);
                return -1;
            }
        } else
            fprintf(stderr, "Mips R300 No such ressource\n"); // todo fprintf cerr
        return -1;
}
int Mips32Iss::local_Resource(char **args)
{
    size_t  i = 1;

    static char *MIPSRessources[] = { // todo MIPS?
        (char *)"p/t\t[int]\t$pc\t\t: Prints PC value",
        (char *)"p/t\t[int]\t$sp\t\t: Prints SP value",
    };

    if (*args[i] != (char)0) {
        fprintf(stderr, "Ressources MIPS :  junk at end of command!\n"); // todo
        return -1;
    }
    for (i = 0; i < sizeof(MIPSRessources)/sizeof(*MIPSRessources); i++)
        fprintf(stdout,"%s\n",MIPSRessources[i]);
    return 0;
}
#endif

Iss2::debug_register_t Mips32Iss::debugGetRegisterValue(unsigned int reg) const
{
    switch (reg)
        {
        case 0:
            return 0;
        case 1 ... 31:
            return r_gp[reg];
        case 32:
            return r_status.whole;
        case 33:
            return r_lo;
        case 34:
            return r_hi;
        case 35:
            return r_bar;
        case 36:
            return r_cause.whole;
        case 37:
            return r_pc;
        default:
            return 0;
        }
}

void Mips32Iss::debugSetRegisterValue(unsigned int reg, debug_register_t value)
{
    switch (reg)
        {
        case 1 ... 31:
            r_gp[reg] = value;
            break;
        case 32:
            r_status.whole = value;
            break;
        case 33:
            r_lo = value;
            break;
        case 34:
            r_hi = value;
            break;
        case 35:
            r_bar = value;
            break;
        case 36:
            r_cause.whole = value;
            break;
        case 37:
            r_pc = value;
            r_npc = value+4;
            break;
        default:
            break;
        }
}

namespace {
static size_t lines_to_s( size_t lines )
{
    return clamp<size_t>(0, uint32_log2(lines)-6, 7);
}
static size_t line_size_to_l( size_t line_size )
{
    if ( line_size == 0 )
        return 0;
    return clamp<size_t>(1, uint32_log2(line_size/4)+1, 7);
}
}

void Mips32Iss::setICacheInfo( size_t line_size, size_t assoc, size_t n_lines )
{
    r_config1.ia = assoc-1;
    r_config1.is = lines_to_s(n_lines);
    r_config1.il = line_size_to_l(line_size);
}

void Mips32Iss::setDCacheInfo( size_t line_size, size_t assoc, size_t n_lines )
{
    r_config1.da = assoc-1;
    r_config1.ds = lines_to_s(n_lines);
    r_config1.dl = line_size_to_l(line_size);
}

Mips32Iss::addr_t Mips32Iss::exceptOffsetAddr( enum ExceptCause cause ) const
{
    if ( r_cause.iv ) {
        if ( r_status.bev || !r_intctl.vs )
            return 0x200;
        else {
            int vn;

            if ( r_config3.veic )
                vn = r_cause.ip>>2;
            else {
                // TODO
                SOCLIB_WARNING("Handling exception offset address when iv and !bev is still to do !");
                vn = 0;
            }
            return 0x200 + vn * (r_intctl.vs<<5);
        }
    } else {
        return 0x180;
    }
}

Mips32Iss::addr_t Mips32Iss::exceptBaseAddr() const
{
    if ( r_status.bev )
        return 0xbfc00200;
    else
        return r_ebase & 0xfffff000;
}

}}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
