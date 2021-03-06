/*
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
 * 		Guironnet de Massas Pierre 2009 
 *
 * Maintainers: Pierre <pierre.guironnet-de-massas@imag.fr>
 */
#ifndef IDCT_REGS_H
#define IDCT_REGS_H

#include "soclib_io.h"

enum IDCTRegisters {
    STATUS,
    IN_DATA,
    OUT_DATA,
};

enum IDCTOp {
	IDCT_READ,
	IDCT_COMPUTE,
	IDCT_WRITE,
};

#endif /* IDCT_REGS_H */
