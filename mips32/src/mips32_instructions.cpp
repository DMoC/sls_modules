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
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 *
 * Maintainers: nipo
 *
 * $Id$
 */

#include "mips32.h"
#include "base_module.h"
#include "arithmetics.h"

#include <strings.h>

namespace soclib { namespace common {

void Mips32Iss::op_bcond()
{
    bool taken;

    taken = (int32_t)r_gp[m_ins.i.rs] < 0;
    taken ^= (bool)(m_ins.i.rt & 1);

    bool likely = m_ins.i.rt & 0x2;

    // and link ?
    if (m_ins.i.rt & 0x10)
        r_gp[31] = r_pc+8;

    if (taken) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    } else if ( likely ) {
        m_skip_next_instruction = true;
    }
}

void Mips32Iss::op_j()
{
    m_next_pc = (r_pc&0xf0000000) | (m_ins.j.imd * 4);
}

void Mips32Iss::op_jal()
{
    r_gp[31] = r_pc+8;
    m_next_pc = (r_pc&0xf0000000) | (m_ins.j.imd * 4);
}

void Mips32Iss::op_beq()
{
    if ( r_gp[m_ins.i.rs] == r_gp[m_ins.i.rt] ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    }
}

void Mips32Iss::op_bne()
{
    if ( r_gp[m_ins.i.rs] != r_gp[m_ins.i.rt] ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    }
}

void Mips32Iss::op_blez()
{
    if ( (int32_t)r_gp[m_ins.i.rs] <= 0 ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    }
}

void Mips32Iss::op_bgtz()
{
    if ( (int32_t)r_gp[m_ins.i.rs] > 0 ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    }
}

void Mips32Iss::op_beql()
{
    if ( r_gp[m_ins.i.rs] == r_gp[m_ins.i.rt] ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    } else {
        m_skip_next_instruction = true;
    }
}

void Mips32Iss::op_bnel()
{
    if ( r_gp[m_ins.i.rs] != r_gp[m_ins.i.rt] ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    } else {
        m_skip_next_instruction = true;
    }
}

void Mips32Iss::op_blezl()
{
    if ( (int32_t)r_gp[m_ins.i.rs] <= 0 ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    } else {
        m_skip_next_instruction = true;
    }
}

void Mips32Iss::op_bgtzl()
{
    if ( (int32_t)r_gp[m_ins.i.rs] > 0 ) {
        m_next_pc = sign_ext16(m_ins.i.imd)*4 + r_pc + 4;
    } else {
        m_skip_next_instruction = true;
    }
}

void Mips32Iss::op_addi()
{
    uint64_t tmp = (uint64_t)r_gp[m_ins.i.rs] + (uint64_t)sign_ext16(m_ins.i.imd);
    if ( overflow( r_gp[m_ins.i.rs], sign_ext16(m_ins.i.imd), 0 ) )
        m_exception = X_OV;
    else
        r_gp[m_ins.i.rt] = tmp;
}

void Mips32Iss::op_addiu()
{
    r_gp[m_ins.i.rt] = r_gp[m_ins.i.rs] + sign_ext16(m_ins.i.imd);
}

void Mips32Iss::op_slti()
{
    r_gp[m_ins.i.rt] = (bool)
        ((int32_t)r_gp[m_ins.i.rs] < sign_ext16(m_ins.i.imd));
}

void Mips32Iss::op_sltiu()
{
    r_gp[m_ins.i.rt] = (bool)
        ((uint32_t)r_gp[m_ins.i.rs] < (uint32_t)sign_ext16(m_ins.i.imd));
}

void Mips32Iss::op_andi()
{
    r_gp[m_ins.i.rt] = r_gp[m_ins.i.rs] & m_ins.i.imd;
}

void Mips32Iss::op_ori()
{
    r_gp[m_ins.i.rt] = r_gp[m_ins.i.rs] | m_ins.i.imd;
}

void Mips32Iss::op_xori()
{
    r_gp[m_ins.i.rt] = r_gp[m_ins.i.rs] ^ m_ins.i.imd;
}

void Mips32Iss::op_lui()
{
    r_gp[m_ins.i.rt] = m_ins.i.imd << 16;
}

void Mips32Iss::op_cop0()
{
    if (!isCopAccessible(0)) {
        m_exception = X_CPU;
        return;
    }

    enum {
        MF = 0,
        MT = 4,
        MFMC0 = 0xb,
        CO1 = 0x10,
    };

    enum {
        ERET = 0x18,
        WAIT = 0x20,
    };

    if ( m_ins.coproc.action & CO1 ) {
        uint32_t co = m_ins.ins & 0x3f;
        switch (co) {
        case ERET:
#ifdef SOCLIB_MODULE_DEBUG
            std::cout << name() << " ERET ";
#endif
            if ( r_status.erl ) {
                m_next_pc = r_error_epc;
                r_status.erl = 0;
#ifdef SOCLIB_MODULE_DEBUG
            std::cout << "erl";
#endif
            } else if ( r_status.exl ) {
                m_next_pc = r_epc;
                r_status.exl = 0;
#ifdef SOCLIB_MODULE_DEBUG
            std::cout << " exl";
#endif
            } else {
                std::cout << m_name << " calling ERET without exl nor erl, ignored" << std::endl;
            }
#ifdef SOCLIB_MODULE_DEBUG
            std::cout << " next_pc: " << m_next_pc << std::endl;
#endif
            m_skip_next_instruction = true;
            update_mode();
            break;
        case WAIT:
            m_sleeping = true;
            break;
        default: // Not handled, so raise an exception
            op_ill();
        }
    } else {
        switch (m_ins.coproc.action) {
        case MT:
            cp0Set( m_ins.coproc.rd, m_ins.coproc.sel, r_gp[m_ins.i.rt] );
            break;
        case MF:
            r_gp[m_ins.coproc.rt] = cp0Get( m_ins.coproc.rd, m_ins.coproc.sel );
            break;
        case MFMC0:
            r_gp[m_ins.coproc.rt] = r_status.whole;
            r_status.ie = m_ins.coproc.sc;
            break;
        default: // Not handled, so raise an exception
            op_ill();
        }
    }
}

void Mips32Iss::op_cop2()
{
    enum {
        MF = 0,
        MT = 4,
    };
    
    if (!isCopAccessible(2)) {
        m_exception = X_CPU;
        return;
    }

    switch (m_ins.coproc.action) {
    case MT:
        //do_mem_access(m_ins.coproc.rd*4, 4, 0, 0, 0, r_gp[m_ins.i.rt], XTN_WRITE);
        do_mem_access(r_gp[m_ins.coproc.rd]*4, 4, 0, 0, 0, r_gp[m_ins.i.rt], XTN_WRITE);
        break;
    case MF:
        //do_mem_access(m_ins.coproc.rd*4, 4, 0, m_ins.i.rt, 0, 0, XTN_READ);
        do_mem_access(r_gp[m_ins.coproc.rd]*4, 4, 0, m_ins.i.rt, 0, 0, XTN_READ);
        break;
    default: // Not handled, so raise an exception
        op_ill();
    }
}

void Mips32Iss::op_ill()
{
    m_exception = X_RI;
}

#define CACHE_OP(what, cache) (((what)<<2)+(cache))
enum {
    ICACHE,
    DCACHE,
    TCACHE,
    SCACHE,
};

enum {
    INDEX_INVAL,
    LOAD_TAG,
    STORE_TAG,
    DEP,
    HIT_INVAL,
    FILL,
    HIT_WB,
    FETCH_AND_LOCK,
};

void Mips32Iss::op_cache()
{
    uint32_t address =  (r_gp[m_ins.i.rs] + sign_ext16(m_ins.i.imd))&~3;

    switch (m_ins.i.rt) {
    case CACHE_OP(HIT_INVAL,DCACHE):
        do_mem_access(4*XTN_DCACHE_INVAL, 4, false, 0, 0, address, XTN_WRITE);
        break;
    default:
        std::cout << name() << " Unsupported cache operation "
                  << std::hex << m_ins.i.rt
                  << " @" << address << std::endl;
        break;
    }
}

}}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
