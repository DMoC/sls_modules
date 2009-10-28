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
 * Copyright (c) TIMA
 *          Patrice Gerin < patrice.gerin@imag.fr >
 *
 * Maintainers: Patrice Gerin
 */
#ifndef _SOCLIB_CABA_VCI_IDCT_H
#define _SOCLIB_CABA_VCI_IDCT_H
	
#include <stdio.h>
#include <systemc.h>
#include <cassert>

#include "caba_base_module.h"
#include "generic_fifo.h"
#include "vci_target.h"
#include "mapping_table.h"

namespace soclib {
namespace caba {

using namespace sc_core;

template<typename    vci_param>
class VciIDCT
    : public soclib::caba::BaseModule
{
private:

  typedef enum{ VCI_IDLE, VCI_RSP, VCI_FIFO_WRITE_WAIT, VCI_FIFO_READ_WAIT } VCI_FSM_t;

// REGISTERS
  VCI_FSM_t       r_vci_fsm;
  sc_signal< bool     >    r_save_eop;
  sc_signal< uint8_t  >    r_save_srcid;
  sc_signal< uint32_t >    r_save_wdata; 
  sc_signal< uint32_t >    r_save_addr;  

  sc_signal< uint32_t >    r_rdata;  // DATA BUFFER
  sc_signal< uint32_t >    r_rerror;  // DATA BUFFER
// STRUCTURAL 
  soclib::caba::GenericFifo<int32_t> m_fifo_in;
  soclib::caba::GenericFifo<int32_t> m_fifo_out; 
  uint32_t m_status;
  int rd_times;
  int wr_times;
  int32_t  rr;
  int32_t* m_received_data;
  uint8_t* m_computed_data;
  int m_op;
  soclib::common::Segment m_segment;

// some helpful methods
  bool on_write(uint32_t addr, uint32_t data);
  bool on_read(uint32_t addr, uint32_t  * data);

  void transition();
  void genMoore();
   
public:
   sc_in<bool> 							 p_clk;
   sc_in<bool> 							 p_resetn;
   soclib::caba::VciTarget<vci_param>    p_t_vci;
   //   sc_in<bool> 						    p_start;
   //   soclib::caba::VciInitiator<vci_param>   p_i_vci;

///////////////////////////////////////////////////
//	constructor
///////////////////////////////////////////////////

protected:
SC_HAS_PROCESS(VciIDCT);

public: 

VciIDCT(sc_module_name insname,
		const soclib::common::MappingTable &mt,
		const soclib::common::IntTab &t_index
	);
        

///////////////////////////////////////////////////////
//      destructor()
//////////////////////////////////////////////////////

~VciIDCT() {}


}; // end class VciIDCT

}}

#endif /* _SOCLIB_CABA_VCI_IDCT_H */


