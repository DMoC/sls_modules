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
#include "../include/vci_idct.h"
#include "../include/idct_reg.h"
#include "../include/idct.h"
#include <iostream>
using namespace std;

namespace soclib { 
namespace caba {

#define tmpl(x)  template<typename vci_param> x VciIDCT<vci_param>

tmpl(/**/)::VciIDCT(sc_module_name insname,
		const soclib::common::MappingTable &mt,
		const soclib::common::IntTab &t_index
)
         :soclib::caba::BaseModule(insname),
         m_fifo_in("fifo_in",100),
         m_fifo_out("fifo_out",100),
		 m_segment(mt.getSegment(t_index)),
         p_clk("clk"),
		 p_resetn("resetn"),
		 p_t_vci("t_vci")
	 	 // and get the segment from the mapping table and our index
	     // m_segment(mt.getSegment(targetIndex))
{
	
	SC_METHOD (transition);
	sensitive << p_clk.pos();
	dont_initialize();

	SC_METHOD (genMoore);
	sensitive << p_clk.neg();
	dont_initialize();
	
	m_status=false;
	m_op = IDCT_READ; // IDCT has 3 main states : READ (wait for 64 data), COMPUTE , WRITE
	rd_times=0; // how many words have been read
	wr_times=0; // how many words have been written
	m_received_data = (int32_t *) malloc (64 * sizeof (int32_t)); // A BUFFER that contains the received data
	m_computed_data = (uint8_t *) malloc (64 * sizeof (uint8_t)); // A BUFFER that will cotain the computed result	
	
} // end constructor

tmpl(void)::transition()
{		
	uint32_t address = 0xDEADBEEF; // dummy values to help debug
	uint32_t wdata   = 0xDEADBEEF; 
	uint32_t rdata   = 0xDEADBEEF;
	bool nerror      = false;

	if (!p_resetn) {
		r_vci_fsm = VCI_IDLE;
		m_status=false;
		m_op = IDCT_READ; // IDCT has 3 main states : READ (wait for 64 data), COMPUTE , WRITE
		rd_times=0; // how many words have been read
		wr_times=0; // how many words have been written
		return;
	}

	//////////////////////////////////////////////////////////////////
	//   This FSM's controls the target interface.
	//////////////////////////////////////////////////////////////////

	switch(r_vci_fsm) {
		case VCI_IDLE :
			if (p_t_vci.cmdval == true){ // there is a request on the interface! first, save the request
				r_save_eop = p_t_vci.eop;
				r_save_srcid = p_t_vci.srcid.read();
				r_save_wdata = p_t_vci.wdata.read();
				r_save_addr = p_t_vci.address.read();

				// There is a request on the interface, save it and go to T_VCI_RSP state
				switch((int)p_t_vci.cmd.read())
				{
					case vci_param::CMD_READ:
						{
							address = (uint32_t)(p_t_vci.address.read() - m_segment.baseAddress()) >> 2; //mettre un uint32_t devant sinon adresse hasardeuse...
							nerror = on_read(address,&rdata);
							r_rdata = rdata;
							//cout << "reading a data : 0x" << std::hex << rdata << endl;
							// Si la fifo est vide, nous ne pouvons pas répondre, il faut donc attendre une donnée dans la FIFO
							// pour cela, on mémorise l'adresse (la requête a été acquitée et ne sera donc pas présente au prochain
							// cycle).
							if (nerror) r_vci_fsm = VCI_RSP;
							else r_vci_fsm = VCI_FIFO_READ_WAIT;
							break;
						}
					case vci_param::CMD_WRITE:
						address = (uint32_t)(p_t_vci.address.read() - m_segment.baseAddress()) >> 2;
						wdata = (uint32_t)p_t_vci.wdata.read();
						//cout << "writing a data : 0x" << std::hex << wdata << endl;
						nerror = on_write(address,wdata);
						// Si la fifo est pleine, nous ne pouvons pas répondre, il faut donc attendre une place vide dans la fifo,
						// pour cela, on mémorise l'adresse (la requête a été acquitée et ne sera donc pas présente au prochain
						// cycle).
						if (nerror) r_vci_fsm = VCI_RSP;
						else r_vci_fsm = VCI_FIFO_WRITE_WAIT;
						break;

					default:
						assert(false); // command that does not exist
						r_rerror=1;
						break;
				}
			}
			// In GenMoore we ACK the request
			break;

		case VCI_FIFO_READ_WAIT :
			assert(false);
			address = (uint32_t)(r_save_addr.read() - m_segment.baseAddress()) >> 2;
			nerror = on_read(address,&rdata);
			r_rdata = rdata;
			if (nerror) r_vci_fsm = VCI_RSP;
			else r_vci_fsm = VCI_FIFO_READ_WAIT;
			break;

		case VCI_FIFO_WRITE_WAIT :
			assert(false);
			address = (uint32_t)(r_save_addr.read() - m_segment.baseAddress()) >> 2;
			wdata = r_save_wdata.read();
			nerror = on_write(address,wdata);
			if (nerror) r_vci_fsm = VCI_RSP;
			else r_vci_fsm = VCI_FIFO_WRITE_WAIT;
			break;

		case VCI_RSP :
			if(p_t_vci.rspack == true){
				// The response was acknowledged, go to VCI_IDLE state
				r_vci_fsm = VCI_IDLE;
			}
			// In GenMoore we set RSPVAL
			break;

		default :
			assert(false);
			break;
	}

	switch (m_op) {

		case IDCT_READ: // READ 64 words from the fifo
			if(m_fifo_in.rok()){
				m_received_data[rd_times]=m_fifo_in.read();
				m_fifo_in.simple_get();
				rd_times++;
				if(rd_times==64){
					m_op=IDCT_COMPUTE; // 64 data have been received, begin the computation of the IDCT
					rd_times=0;					
				}
			}
			break;

		case IDCT_COMPUTE: 
			IDCT(m_received_data, m_computed_data); // call idct, note that this is done here in 1 cycle, this is
													// not physically possible.
			m_op=IDCT_WRITE;
			break;

		case IDCT_WRITE:
			if(m_fifo_out.wok()){
				m_status=false;
				m_fifo_out.simple_put(m_computed_data[wr_times]);	
				wr_times++;			
				if(wr_times==64){
					m_op=IDCT_READ;
					wr_times=0;					
				}	
			}
			break; 		
		default :
			assert(false);
			break;
	}     	
      	
   if(m_fifo_out.rok() && m_op!=IDCT_WRITE){
    	m_status=true;   // the computed IDCT is in the fifo waiting to be read.	
   }

}

tmpl(void)::genMoore()
{
  switch(r_vci_fsm) 
  {
    case VCI_IDLE :
      p_t_vci.cmdack = true;
      p_t_vci.rspval = false;
      p_t_vci.rdata  = 0;
      p_t_vci.reop = false;
      break;

    case VCI_FIFO_WRITE_WAIT :
    case VCI_FIFO_READ_WAIT :
      p_t_vci.cmdack = false;
      p_t_vci.rspval = false;
      p_t_vci.rdata  = 0;
      p_t_vci.reop = false;
      break;

    case VCI_RSP :
      p_t_vci.cmdack = false;
      p_t_vci.rspval = true;
      p_t_vci.rerror = r_rerror.read();
      p_t_vci.rdata  = r_rdata.read();
      p_t_vci.reop = r_save_eop.read();
      p_t_vci.rsrcid = r_save_srcid.read();
      break;

    default :
	  assert(false);
      break;
  }

}

tmpl(bool)::on_read(uint32_t addr, uint32_t  * data)
{	
	bool nerror = false; // 1 si OK, 0 si erreur

	switch (addr) {
		case STATUS:                     // Lecture du registre status
			*data = m_status;
			if(m_status) m_status=false; // pas encore compris à quoi ça sert!
			nerror = true;
			break;
		case IN_DATA:                    // On ne peut pas lire la queue de la fifo
			cout << "error in idct READ_IN_DATA returning false" << endl;
			nerror = false;
			break;
		case OUT_DATA:                   // On lis la tête de la fifo sortante
			if(m_fifo_out.rok()){
				*data=m_fifo_out.read();  // lecture de l'élément
				m_fifo_out.simple_get(); // retrait de l'élément
				nerror = true;		
			}else{
				nerror = false;
			}
			break;
		default :
			assert(false);
			break;
	};
	return nerror;
}

tmpl(bool)::on_write(uint32_t addr, uint32_t data)
{
	bool nerror = false; // 1 si OK, 0 si erreur
	
	switch (addr) {
		case STATUS:
			cout << "error in idct STATUS returning false" << endl;
			nerror = false;
			assert(false);
			break;
		case IN_DATA:
			if(m_fifo_in.wok()){
				m_fifo_in.simple_put(data);
				nerror = true;
			}else{
				nerror = false;
			}
			break;
		case OUT_DATA:
			cout << "error in idct WRITE_OUTDATA returning false" << endl;
			nerror = false;
			assert(false);
			break;
		default : 
			assert(false);
			break;
	};
	return nerror;
}    
}}
