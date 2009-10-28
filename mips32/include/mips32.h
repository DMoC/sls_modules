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
 * Based on mips code
 *         Nicolas Pouillon <nipo@ssji.net>, 2007
 *         Alain Greiner <alain.greiner@lip6.fr>, 2007
 *
 * Maintainers: nipo
 *
 * $Id$
 *
 * History:
 * - 2008-07-10
 *   Nicolas Pouillon: Forked mips r3000 to begin mips32
 */

#ifndef _SOCLIB_MIPS32_ISS_H_
#define _SOCLIB_MIPS32_ISS_H_

#include <cassert>

#include "iss2.h"
#include "soclib_endian.h"
#include "register.h"

namespace soclib { namespace common {

class Mips32Iss
    : public Iss2
{
public:
    static const int n_irq = 6;

private:
    enum MipsDataAccessType {
        MDAT_LB,
        MDAT_LBU,
        MDAT_LH,
        MDAT_LHU,
        MDAT_LW,
        MDAT_LL,
        MDAT_SC,
        MDAT_INVAL,
        MDAT_SB,
        MDAT_SH,
        MDAT_SW,
        MDAT_LWL,
        MDAT_LWR,
        MDAT_SWL,
        MDAT_SWR,
    };

    enum Addresses {
        RESET_ADDRESS   = 0xbfc00000
    };

    enum ExceptCause {
        X_INT,      // Interrupt
        X_MOD,      // TLB Modification
        X_TLBL,     // TLB Load error
        X_TLBS,     // TLB Store error
        X_ADEL,     // Address error (load or fetch)
        X_ADES,     // Address error (store)
        X_IBE,      // Ins bus error
        X_DBE,      // Data bus error (load/store)
        X_SYS,      // Syscall
        X_BP,       // Break point
        X_RI,       // Reserved
        X_CPU,      // Coproc unusable
        X_OV,       // Overflow
        X_TR,       // Trap
        X_reserved,     // Reserved
        X_FPE,      // Floating point
        NO_EXCEPTION,
    };

    // member variables (internal registers)

	data_t 	r_pc;			// Program Counter
	data_t 	r_npc;			// Next Program Counter
    data_t    r_gp[32];       // General Registers
    data_t    r_hi;           // Multiply result (MSB bits)
    data_t    r_lo;           // Multiply result (LSB bits)

    struct DataRequest m_dreq;
    int r_mem_do_sign_extend;
    int r_mem_byte_le;
    int r_mem_byte_count;
    int r_mem_offset_byte_in_reg;
    uint32_t r_mem_dest;

	data_t	m_rdata;
	bool		m_ibe;
	bool		m_dbe;

    bool m_skip_next_instruction;

    // Instruction latency simulation
    uint32_t m_ins_delay;


    typedef union {
        struct {
            union {
                PACKED_BITFIELD(
                    uint32_t op:6,
                    uint32_t imd:26
                    ) j;
                PACKED_BITFIELD(
                    uint32_t op:6,
                    uint32_t rs:5,
                    uint32_t rt:5,
                    uint32_t imd:16
                    ) i;
                PACKED_BITFIELD(
                    uint32_t op:6,
                    uint32_t rs:5,
                    uint32_t rt:5,
                    uint32_t rd:5,
                    uint32_t sh:5,
                    uint32_t func:6
                    ) r;
                PACKED_BITFIELD(
                    uint32_t op:6,
                    uint32_t action:5,
                    uint32_t rt:5,
                    uint32_t rd:5,
                    uint32_t zero:5,
                    uint32_t sc:1,
                    uint32_t zero2:2,
                    uint32_t sel:3
                    ) coproc;
            } __attribute__((packed));
        } __attribute__((packed));
        uint32_t ins;
    } ins_t;

    enum KSUMode {
        MIPS32_KSU_KERNEL,
        MIPS32_KSU_SUPERVISOR,
        MIPS32_KSU_USER,
        MIPS32_KSU_RESERVED,
    };

    enum Mips32Mode {
        MIPS32_KERNEL,
        MIPS32_SUPERVISOR,
        MIPS32_USER,
        MIPS32_DEBUG,
    };

    enum Iss2::ExecMode r_bus_mode;
    enum Mips32Mode r_cpu_mode;

    typedef REG32_BITFIELD(
        uint32_t cu3:1,
        uint32_t cu2:1,
        uint32_t cu1:1,
        uint32_t cu0:1,
        uint32_t rp:1,
        uint32_t fr:1,
        uint32_t re:1,
        uint32_t mx:1,
        uint32_t px:1,
        uint32_t bev:1,
        uint32_t ts:1,
        uint32_t sr:1,
        uint32_t nmi:1,
        uint32_t zero:1,
        uint32_t impl:2,
        uint32_t im:8,
        uint32_t kx:1,
        uint32_t sx:1,
        uint32_t ux:1,
        uint32_t ksu:2,
        uint32_t erl:1,
        uint32_t exl:1,
        uint32_t ie:1,
        ) status_t;

    typedef REG32_BITFIELD(
        uint32_t bd:1,
        uint32_t ti:1,
        uint32_t ce:2,
        uint32_t dc:1,
        uint32_t pci:1,
        uint32_t zero:2,
        uint32_t iv:1,
        uint32_t wp:1,
        uint32_t zero2:6,
        uint32_t ip:8,
        uint32_t zero3:1,
        uint32_t xcode:5,
        uint32_t zero4:2,
        ) cause_t;

    typedef REG32_BITFIELD(
        uint32_t ipti:3,
        uint32_t ippci:3,
        uint32_t zero:16,
        uint32_t vs:5,
        uint32_t zero2:5
        ) intctl_t;

    typedef REG32_BITFIELD(
        uint32_t m:1,
        uint32_t mmu_size:6,
        uint32_t is:3,
        uint32_t il:3,
        uint32_t ia:3,
        uint32_t ds:3,
        uint32_t dl:3,
        uint32_t da:3,
        uint32_t c2:1,
        uint32_t md:1,
        uint32_t pc:1,
        uint32_t wr:1,
        uint32_t ca:1,
        uint32_t ep:1,
        uint32_t fp:1,
        ) config1_t;

    typedef REG32_BITFIELD(
        uint32_t m:1,
        uint32_t k23:3,
        uint32_t ku:3,
        uint32_t impl:9,
        uint32_t be:1,
        uint32_t at:2,
        uint32_t ar:3,
        uint32_t mt:3,
        uint32_t zero:3,
        uint32_t vi:1,
        uint32_t k0:3,
        ) config_t;

    typedef REG32_BITFIELD(
        uint32_t m:1,
        uint32_t tu:3,
        uint32_t ts:4,
        uint32_t tl:4,
        uint32_t ta:4,
        uint32_t su:4,
        uint32_t ss:4,
        uint32_t sl:4,
        uint32_t sa:4,
        ) config2_t;

    typedef REG32_BITFIELD(
        uint32_t M:1,
        uint32_t reserved0:17,
        uint32_t ulri:1,
        uint32_t reserved12:1,
        uint32_t dsp2p:1,
        uint32_t dspp:1,
        uint32_t reserved1:2,
        uint32_t lpa:1,
        uint32_t veic:1,
        uint32_t vint:1,
        uint32_t sp:1,
        uint32_t reserved2:1,
        uint32_t mt:1,
        uint32_t sm:1,
        uint32_t tl:1
        ) config3_t;

    status_t r_status;
    cause_t r_cause;
    addr_t r_ebase;
    addr_t r_bar;
    addr_t r_epc;
    addr_t r_error_epc;
    uint32_t r_count;
    uint32_t r_compare;

    bool m_sleeping;
    
    // member variables used for communication between
    // member functions (they are not registers)

    ins_t       m_ins;
    enum ExceptCause    m_exception;
    addr_t    m_next_pc;
    uint32_t    m_exec_cycles;
    bool m_hazard;

    config_t r_config;
    config1_t r_config1;
    config2_t r_config2;
    config3_t r_config3;
    intctl_t r_intctl;
    uint32_t r_hwrena;
    uint32_t r_tls_base;

    const bool m_little_endian;

    bool m_ireq_ok;
    bool m_dreq_ok;

public:
    Mips32Iss(const std::string &name, uint32_t ident, bool default_little_endian);

    void dump() const;

    uint32_t executeNCycles(
        uint32_t ncycle,
        const struct InstructionResponse &irsp,
        const struct DataResponse &drsp,
        uint32_t irq_bit_field );

	inline void getRequests( struct InstructionRequest &ireq,
                             struct DataRequest &dreq ) const
	{
        ireq.valid = !m_sleeping;
		ireq.addr = r_pc;
        ireq.mode = r_bus_mode;
        dreq = m_dreq;
	}

	inline void setWriteBerr()
	{
		m_dbe = true;
	}

    void reset();

    // processor internal registers access API, used by
    // debugger. Mips32 order is 32 general-purpose; sr; lo; hi; bad; cause; pc;

    // CDB method's
#ifdef CDB_COMPONENT_IF_H
    const char* local_GetModel();
    int local_PrintResource(modelResource *res, char **p);
    int local_TestResource(modelResource *res, char **p);
    int local_Resource(char** args);
#endif

    inline unsigned int debugGetRegisterCount() const
    {
        return 32 + 6;
    }

    virtual debug_register_t debugGetRegisterValue(unsigned int reg) const;

    inline size_t debugGetRegisterSize(unsigned int reg) const
    {
        return 32;
    }

    virtual void debugSetRegisterValue(unsigned int reg, debug_register_t value);

    static const unsigned int s_sp_register_no = 29;
    static const unsigned int s_fp_register_no = 30;
    static const unsigned int s_pc_register_no = 37;

    int debugCpuCauseToSignal( uint32_t cause ) const;

    void setICacheInfo( size_t line_size, size_t assoc, size_t n_lines );
    void setDCacheInfo( size_t line_size, size_t assoc, size_t n_lines );

private:
    void run();

    void _setData(const struct DataResponse &rsp);

    inline void setInsDelay( uint32_t delay )
    {
        assert( delay > 0 );
        m_ins_delay = delay-1;
    }

    addr_t exceptOffsetAddr( enum ExceptCause cause ) const;
    addr_t exceptBaseAddr() const;

    inline bool isPriviliged() const
    {
        return r_cpu_mode != MIPS32_USER;
    }

    inline bool isHighPC() const
    {
        return (addr_t)r_pc & (addr_t)0x80000000;
    }

    inline bool isPrivDataAddr( addr_t addr ) const
    {
        return addr & (addr_t)0x80000000;
    }

    typedef void (Mips32Iss::*func_t)();

    static func_t const opcod_table[64];
    static func_t const special_table[64];


    void do_mem_access( addr_t address,
                        int byte_count,
                        int sign_extend,
                        int dest_reg,
                        int dest_byte_in_reg,
                        data_t wdata,
                        enum DataOperationType operation );

    void op_special();
    void op_special2();
    void op_special3();
    void op_bcond();
    void op_j();
    void op_jal();
    void op_beq();
    void op_bne();
    void op_blez();
    void op_bgtz();
    void op_addi();
    void op_addiu();
    void op_slti();
    void op_sltiu();
    void op_andi();
    void op_ori();
    void op_xori();
    void op_lui();
    void op_cop0();
    void op_cop2();
    void op_beql();
    void op_bnel();
    void op_blezl();
    void op_bgtzl();
    void op_ill();
    void op_lb();
    void op_lh();
    void op_ll();
    void op_lw();
    void op_lwl();
    void op_lwr();
    void op_lbu();
    void op_lhu();
    void op_sb();
    void op_sh();
    void op_sw();
    void op_swl();
    void op_swr();
    void op_sc();
    void op_cache();

    void special_sll();
    void special_srl();
    void special_sra();
    void special_sllv();
    void special_srlv();
    void special_srav();
    void special_jr();
    void special_jalr();
    void special_sysc();
    void special_brek();
    void special_sync();
    void special_mfhi();
    void special_movn();
    void special_movz();
    void special_mthi();
    void special_mflo();
    void special_mtlo();
    void special_mult();
    void special_multu();
    void special_div();
    void special_divu();
    void special_add();
    void special_addu();
    void special_sub();
    void special_subu();
    void special_and();
    void special_or();
    void special_xor();
    void special_nor();
    void special_slt();
    void special_sltu();
    void special_ill();

    void special_tlt();
    void special_tltu();
    void special_tge();
    void special_tgeu();
    void special_teq();
    void special_tne();

    typedef enum {
        USE_NONE = 0,
        USE_T    = 1,
        USE_S    = 2,
        USE_ST   = 3,
        USE_SPECIAL = 4,
    } use_t;

    use_t curInstructionUsesRegs();

    static const char* name_table[64];
    static use_t const use_table[64];
    static use_t const use_special_table[64];

    bool isCopAccessible(int) const;
    uint32_t cp0Get( uint32_t reg, uint32_t sel ) const;
    void cp0Set( uint32_t reg, uint32_t sel, uint32_t value );

    void update_mode();

    // Make sure users dont try to instanciate Mips32Iss class
    virtual void please_instanciate_Mips32ElIss_or_Mips32EbIss() = 0;
};

class Mips32ElIss
    : public Mips32Iss
{
public:
    static const Iss2::debugCpuEndianness s_endianness = Iss2::ISS_LITTLE_ENDIAN;

    Mips32ElIss(const std::string &name, uint32_t ident)
        : Mips32Iss(name, ident, true)
    {}

    void please_instanciate_Mips32ElIss_or_Mips32EbIss() {}
};

class Mips32EbIss
    : public Mips32Iss
{
public:
    static const Iss2::debugCpuEndianness s_endianness = Iss2::ISS_BIG_ENDIAN;

    Mips32EbIss(const std::string &name, uint32_t ident)
        : Mips32Iss(name, ident, false)
    {}

    void please_instanciate_Mips32ElIss_or_Mips32EbIss() {}
};

}}

#endif // _SOCLIB_MIPS32_ISS_H_

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
