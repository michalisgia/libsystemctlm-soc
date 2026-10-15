/*
 * TLM-2.0 to PCIe TLP bridge.
 *
 * Copyright (c) 2022 AMD Inc.
 *
 * Written by Francisco Iglesias <francisco.iglesias@amd.com>.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * [1] PCI Express Base Specification Revision 5.0 Version 1.0
 *
 */

#ifndef TLM2TLP_BRIDGE_H__
#define TLM2TLP_BRIDGE_H__

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <list>
#include <sstream>

#include "tlm-bridges/pci.h"
#include "tlm-extensions/genattr.h"
#include "tlm-extensions/atsattr.h"
#include "utils/bitops.h"

#define GEN_TLP_HDR_FIELD_FUNCS(idx, fname, shift, width) 		\
uint32_t get_ ## fname()						\
{									\
	return (m_hdr[idx] >> shift) & bitops_mask64(0, width);		\
}									\
uint32_t set_ ## fname(uint32_t v) {					\
	uint32_t mask = static_cast<uint32_t>(bitops_mask64(0, width));	\
	assert((v & ~mask) == 0);					\
	return (v & mask) << shift;					\
}

#define GEN_FIELD_FUNCS(fname, shift, width)				\
uint32_t get_ ## fname(uint32_t r)					\
{									\
	return (r >> shift) & bitops_mask64(0, width);			\
} 									\
uint32_t set_ ## fname(uint32_t v) {					\
	uint32_t mask = static_cast<uint32_t>(bitops_mask64(0, width));	\
	assert((v & ~mask) == 0);					\
	return (v & mask) << shift;					\
}

#define TLM2TLP_BRIDGE_MSG "tlm2tlp-bridge"

using namespace PCI::TLP;

class tlm2tlp_bridge : public sc_core::sc_module
{
public:
	//
	// TLM side
	//
	tlm_utils::simple_target_socket<tlm2tlp_bridge> cfg_tgt_socket;
	tlm_utils::simple_target_socket<tlm2tlp_bridge> mem_tgt_socket;
	tlm_utils::simple_target_socket<tlm2tlp_bridge> io_tgt_socket;
	tlm_utils::simple_target_socket<tlm2tlp_bridge> msg_tgt_socket;

	tlm_utils::simple_initiator_socket<tlm2tlp_bridge> dma_init_socket;

	//
	// TLP side
	//
	tlm_utils::simple_initiator_socket<tlm2tlp_bridge> init_socket;
	tlm_utils::simple_target_socket<tlm2tlp_bridge> tgt_socket;

	class TLPHdr_Base
	{
	public:
		//
		// For bits [31:0]
		//
		GEN_TLP_HDR_FIELD_FUNCS(0, fmt, 29, 3);
		GEN_TLP_HDR_FIELD_FUNCS(0, type, 24, 5);
		GEN_TLP_HDR_FIELD_FUNCS(0, t9, 23, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, tc, 20, 3);
		GEN_TLP_HDR_FIELD_FUNCS(0, t8, 19, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, attr2, 18, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, ln, 17, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, th, 16, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, td, 15, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, ep, 14, 1);
		GEN_TLP_HDR_FIELD_FUNCS(0, attr_1_0, 12, 2);
		GEN_TLP_HDR_FIELD_FUNCS(0, at, 10, 2);
		GEN_TLP_HDR_FIELD_FUNCS(0, length, 0, 10);

		GEN_TLP_HDR_FIELD_FUNCS(0, fmt_data_bit, 30, 1);

		// Used when creating from raw TLP data
		TLPHdr_Base() :
			m_gp(nullptr),
			m_done(false)
		{}

		TLPHdr_Base(tlm::tlm_generic_payload *gp) :
			m_gp(gp),
			m_done(false)
		{}

		virtual ~TLPHdr_Base() {}

		tlm::tlm_generic_payload *GetTLPGP() { return &m_tlp; }

		//
		// The stored orginal gp for setting response
		//
		tlm::tlm_generic_payload *GetStoredGP() { return m_gp; }

		void SetDone(bool val = true) { m_done = val; }
		bool Done() { return m_done; }

		bool ExpectsData()
		{
			switch (get_type()) {
			case Type_CfgRdWr_type0:
			case Type_MRdWr:
				return get_fmt_data_bit() == 0;
			default:
				break;
			}
			return false;
		}

		virtual uint32_t GetTxID() { return 0; }

		uint32_t GetTLPData(unsigned int DW_pos)
		{
			unsigned int data_offset =
				(get_fmt() == FMT_3DW_WithData) ? 3 : 4;

			data_offset += DW_pos;

			assert(data_offset < m_hdr.size());

			return m_hdr[data_offset];
		}


		//
		// Convert byte position to DW position
		//
		unsigned int dw_pos(unsigned int b_pos) { return b_pos / 4; }

		void CopyData(TLPHdr_Base *tlp)
		{
			unsigned int dlen = m_gp->get_data_length();
			uint8_t *d = m_gp->get_data_ptr();
			unsigned int dw_len = tlp->get_length() ?
						tlp->get_length() :
						MAX_DW_LEN;
			//
			// The completion data starts at the DW aligned
			// address, skip the leading bytes of the first DW.
			//
			unsigned int addr_offset = m_gp->get_address() & 0x3;

			for (unsigned int i = 0; i < dlen; i++) {
				unsigned int lane = addr_offset + i;
				unsigned int DW_pos = dw_pos(lane);
				uint32_t v = (DW_pos < dw_len) ?
						tlp->GetTLPData(DW_pos) : 0;

				d[i] = (v >> (24 - (lane % 4) * 8)) & 0xFF;
			}
		}

		uint32_t byteswap(uint32_t val)
		{
#if __BYTE_ORDER == __LITTLE_ENDIAN
			val = (val & 0xFF) << 24 |
				(val >> 8 & 0xFF) << 16 |
				(val >> 16 & 0xFF) << 8 |
				(val >> 24 & 0xFF);
#endif
			return val;
		}

		//
		// Converts the TLP into wire byte order. This is done on
		// a copy so that m_hdr stays in host order, it is used
		// for matching the completion (GetTxID, ExpectsData) and
		// the receiver is free to modify the transmitted data.
		//
		void ByteSwap()
		{
			m_wire.resize(m_hdr.size());
			for (unsigned int i = 0; i < m_hdr.size(); i++) {
				m_wire[i] = byteswap(m_hdr[i]);
			}
			m_tlp.set_data_ptr(reinterpret_cast<uint8_t*>(m_wire.data()));
		}

		sc_event& DMADoneEvent() { return m_dma_done_event; }

	protected:
		std::vector<uint32_t> m_hdr;
		std::vector<uint32_t> m_wire;	// m_hdr in wire byte order
		tlm::tlm_generic_payload m_tlp;
		tlm::tlm_generic_payload *m_gp;  // stored gp
		bool m_done;
		sc_event m_dma_done_event;
	};

	class TLP_Conf: public TLPHdr_Base
	{
	public:
		//
		// For bits [63:32]
		//
		GEN_TLP_HDR_FIELD_FUNCS(1, requestorID, 16, 16);
		GEN_TLP_HDR_FIELD_FUNCS(1, tag, 8, 8);
		GEN_TLP_HDR_FIELD_FUNCS(1, lastDWBE, 4, 4);
		GEN_TLP_HDR_FIELD_FUNCS(1, firstDWBE, 0, 4);

		//
		// For bits [95:64]
		//
		GEN_TLP_HDR_FIELD_FUNCS(2, busNumber, 24, 8);
		GEN_TLP_HDR_FIELD_FUNCS(2, deviceNumber, 19, 5);
		GEN_TLP_HDR_FIELD_FUNCS(2, funcNumber, 16, 3);
		GEN_TLP_HDR_FIELD_FUNCS(2, rsvd1, 12, 4);
		GEN_TLP_HDR_FIELD_FUNCS(2, extRegNum, 8, 4);
		GEN_TLP_HDR_FIELD_FUNCS(2, RegNum, 2, 6);
		GEN_TLP_HDR_FIELD_FUNCS(2, rsvd0, 0, 2);

		//
		// ECAM Address fields
		//
		GEN_FIELD_FUNCS(ECAMAddr_bus, 20, 8);
		GEN_FIELD_FUNCS(ECAMAddr_dev, 15, 5);
		GEN_FIELD_FUNCS(ECAMAddr_fn, 12, 3);
		GEN_FIELD_FUNCS(ECAMAddr_extRegNum, 8, 4);
		GEN_FIELD_FUNCS(ECAMAddr_RegNum, 2, 6);
		GEN_FIELD_FUNCS(ECAMAddr_BE, 0, 2);

		TLP_Conf(tlm::tlm_generic_payload *gp) :
			TLPHdr_Base(gp)
		{
			uint32_t addr = static_cast<uint32_t>(gp->get_address());
			uint32_t fmt = gp->is_write() ?
					FMT_3DW_WithData : FMT_3DW_NoData;
			uint32_t firstDWBE = 0;
			unsigned char *be = gp->get_byte_enable_ptr();
			int be_len = gp->get_byte_enable_length();
			genattr_extension *genattr;
			uint32_t masterID = 0;
			uint32_t txID = 0;
			uint32_t dlen;

			// Bits [31:0]
			m_hdr.push_back(set_fmt(fmt) |
					set_type(Type_CfgRdWr_type0) |

					set_t9(0) |
					set_tc(0) |
					set_t8(0) |
					set_attr2(0) |
					set_ln(0) |
					set_th(0) |

					set_td(0) |
					set_ep(0) |
					set_attr_1_0(0) |
					set_at(0) |

					// Len is always 1 ([1] 2.2.7)
					set_length(1));

			//
			// Bits [63:32]
			//
			gp->get_extension(genattr);
			if (genattr) {
				masterID = genattr->get_master_id();
				txID = genattr->get_transaction_id();
			}
			//
			// max dlen is 1 DW
			//
			dlen = (gp->get_data_length() < 4) ? gp->get_data_length() : 4;
			if (be && be_len) {
				unsigned int start_be_bit = get_ECAMAddr_BE(addr);

				for (unsigned int i = 0; i < dlen; i++) {
					uint8_t b = be[i % be_len];
					if (b == TLM_BYTE_ENABLED) {
						 firstDWBE |= 1 << (start_be_bit + i);
					}
				}
			} else {
				unsigned int start_be_bit = get_ECAMAddr_BE(addr);

				for (unsigned int i = 0; i < dlen; i++) {
					 firstDWBE |= 1 << (start_be_bit + i);
				}
			}
			m_hdr.push_back(
				set_requestorID(masterID) |
				set_tag(txID) |
				set_lastDWBE(0) |
				set_firstDWBE(firstDWBE));

			//
			// Bits [95:64]
			//
			m_hdr.push_back(set_busNumber(get_ECAMAddr_bus(addr)) |
					set_deviceNumber(get_ECAMAddr_dev(addr)) |
					set_funcNumber(get_ECAMAddr_fn(addr)) |
					set_rsvd1(0) |
					set_extRegNum(get_ECAMAddr_extRegNum(addr)) |
					set_RegNum(get_ECAMAddr_RegNum(addr)) |
					set_rsvd0(0));
			//
			// Data if it is a write configuration packet.
			// Data is later going to be run through byteswap.
			//
			if (gp->is_write()) {
				uint8_t *d = gp->get_data_ptr();
				uint32_t val = 0;
				unsigned int start_idx = get_ECAMAddr_BE(addr);

				for (unsigned int i = 0; i < dlen; i++) {
					uint32_t shift = (3 - start_idx - i) * 8;
					val |= d[i] << shift;
				}
				m_hdr.push_back(val);
			}

			//
			// Setup m_tlp
			//
			// command should perhaps always be TLM_WRITE_COMMAND???
			//
			// (Check TLM spec if read means data can be manipulated)
			//
			m_tlp.set_command(gp->get_command());
			m_tlp.set_address(0); // unused

			m_tlp.set_data_ptr(reinterpret_cast<uint8_t*>(m_hdr.data()));
			m_tlp.set_data_length(m_hdr.size() * 4);
			m_tlp.set_byte_enable_ptr(nullptr);
			m_tlp.set_byte_enable_length(0);
			m_tlp.set_streaming_width(m_hdr.size() * 4);
			m_tlp.set_dmi_allowed(false);
			m_tlp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		}

		//
		// Section 2.2.7 [1]
		//
		uint32_t GetTxID()
		{
			return get_requestorID() << 16 | get_t9() << 9 |
				get_t8() << 8 | get_tag();
		}

	};

	class TLP_Mem: public TLPHdr_Base
	{
	public:
		//
		// For bits [63:32]
		//
		GEN_TLP_HDR_FIELD_FUNCS(1, requestorID, 16, 16);
		GEN_TLP_HDR_FIELD_FUNCS(1, tag, 8, 8);
		GEN_TLP_HDR_FIELD_FUNCS(1, lastDWBE, 4, 4);
		GEN_TLP_HDR_FIELD_FUNCS(1, firstDWBE, 0, 4);

		//
		// For bits [95:64] on the 4DW header
		//
		GEN_TLP_HDR_FIELD_FUNCS(2, 4DWHdr_addr_63_32, 0, 32);

		//
		// For bits [95:64] on the 3DW header
		//
		GEN_TLP_HDR_FIELD_FUNCS(2, 3DWHdr_addr_31_2, 2, 30);
		GEN_TLP_HDR_FIELD_FUNCS(2, 3DWHdr_ph, 0, 2);

		//
		// For bits [127:96] on the 4DW header
		//
		GEN_TLP_HDR_FIELD_FUNCS(3, 4DWHdr_addr_31_2, 2, 30);
		GEN_TLP_HDR_FIELD_FUNCS(3, 4DWHdr_ph, 0, 2);

		GEN_FIELD_FUNCS(addr_32_2, 2, 30); // Get addr_32_2 from u32

		//
		// Returns true if byte lane 'lane' (counted from the DW
		// aligned start address) is written / read by the gp.
		//
		static bool LaneEnabled(tlm::tlm_generic_payload *gp,
					unsigned int lane)
		{
			unsigned int b_off = gp->get_address() & 0x3;
			unsigned char *be = gp->get_byte_enable_ptr();
			unsigned int be_len = gp->get_byte_enable_length();

			if (lane < b_off ||
			    lane >= b_off + gp->get_data_length()) {
				return false;
			}
			if (be && be_len) {
				return be[(lane - b_off) % be_len] ==
					TLM_BYTE_ENABLED;
			}
			return true;
		}

		//
		// Generates 3DW header if address does not use the upper 32
		// bits, otherwise it generates a 4 DW header
		//
		TLP_Mem(tlm::tlm_generic_payload *gp) :
			TLPHdr_Base(gp),
			m_tlp_data(nullptr),
			m_tlp_dw_len(0)
		{
			uint64_t addr = gp->get_address();
			uint32_t fmt = gp->is_write() ?
					FMT_3DW_WithData : FMT_3DW_NoData;
			uint32_t firstDWBE = 0;
			uint32_t lastDWBE = 0;
			genattr_extension *genattr;
			uint32_t masterID = 0;
			uint32_t txID = 0;
			//
			// Number of DWs touched, taking the byte offset of an
			// unaligned address into account.
			//
			unsigned int b_off = addr & 0x3;
			uint32_t dlen = (b_off + gp->get_data_length() + 3) / 4;
			bool gen4DWhdr = (addr >> 32) ?
                                         true : false; // Any upper bit set

			if (gen4DWhdr) {
				fmt = gp->is_write() ?
					FMT_4DW_WithData : FMT_4DW_NoData;
			} else {
				fmt = gp->is_write() ?
					FMT_3DW_WithData : FMT_3DW_NoData;
			}

			// Bits [31:0]
			m_hdr.push_back(set_fmt(fmt) |
					set_type(Type_MRdWr) |

					set_t9(0) |
					set_tc(0) |
					set_t8(0) |
					set_attr2(0) |
					set_ln(0) |
					set_th(0) |

					set_td(0) |
					set_ep(0) |
					set_attr_1_0(0) |
					set_at(0) |

					set_length(dlen == MAX_DW_LEN ? 0 : dlen));

			//
			// Bits [63:32]
			//
			gp->get_extension(genattr);
			if (genattr) {
				masterID = genattr->get_master_id();
				txID = genattr->get_transaction_id();
			}

			//
			// FirstDWBE is for the first 4 byte lanes
			//
			if (dlen >= 1) {
				for (unsigned int i = 0; i < 4; i++) {
					if (LaneEnabled(gp, i)) {
						firstDWBE |= 1 << i;
					}
				}
			}

			//
			// LastDWBE is for the last 4 byte lanes
			//
			if (dlen > 1) {
				for (unsigned int i = 0; i < 4; i++) {
					if (LaneEnabled(gp, (dlen - 1) * 4 + i)) {
						lastDWBE |= 1 << i;
					}
				}
			}

			m_hdr.push_back(
				set_requestorID(masterID) |
				set_tag(txID) |
				set_lastDWBE(lastDWBE) |
				set_firstDWBE(firstDWBE));

			//
			// Bits [127:96] on the 4DW header
			// Bits [95:64] on the 3DW header
			//
			if (gen4DWhdr) {
				//
				// Upper 32 bits of the addr
				//
				m_hdr.push_back(addr >> 32);
			}
			addr = get_addr_32_2(addr & 0xFFFFFFFF);
			m_hdr.push_back(set_4DWHdr_addr_31_2(addr) |
					set_4DWHdr_ph(0));

			//
			// Data if it is a write packet
			//
			if (gp->is_write()) {
				uint8_t *d = gp->get_data_ptr();
				unsigned int len = gp->get_data_length();

				//
				// Place each byte on its byte lane, lanes
				// before the start address offset and after
				// the end are left as zero (disabled by BE).
				//
				for (unsigned int dw = 0; dw < dlen; dw++) {
					uint32_t val = 0;

					for (unsigned int i = 0; i < 4; i++) {
						unsigned int lane = dw * 4 + i;

						if (lane >= b_off &&
						    lane < b_off + len) {
							val |= d[lane - b_off] <<
								(24 - i * 8);
						}
					}
					m_hdr.push_back(val);
				}

				//
				// Memory Writes are posted, 2.1.1.1 [1]
				//
				m_done = true;
				gp->set_response_status(tlm::TLM_OK_RESPONSE);
			}

			//
			// Setup m_tlp
			//
			// command should perhaps always be TLM_WRITE_COMMAND???
			//
			// (Check TLM spec if read means data can be manipulated)
			//
			m_tlp.set_command(gp->get_command());
			m_tlp.set_address(0); // unused

			m_tlp.set_data_ptr(reinterpret_cast<uint8_t*>(m_hdr.data()));
			m_tlp.set_data_length(m_hdr.size() * 4);
			m_tlp.set_byte_enable_ptr(nullptr);
			m_tlp.set_byte_enable_length(0);
			m_tlp.set_streaming_width(m_hdr.size() * 4);
			m_tlp.set_dmi_allowed(false);
			m_tlp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		}

		TLP_Mem(uint8_t *hdr,  unsigned int b_len) :
			TLPHdr_Base(nullptr),
			m_tlp_data(nullptr),
			m_tlp_dw_len(0)
		{
			unsigned int i = 0;
			uint32_t fmt;

			assert(b_len >= TLPHdr_3DW_Sz);

			for (; i < TLPHdr_3DW_Sz; i+=4) {
				uint32_t val = hdr[i] << 24 |
						hdr[i+1] << 16 |
						hdr[i+2] << 8 |
						hdr[i+3];

				m_hdr.push_back(val);
			}

			fmt = get_fmt();
			if (fmt == FMT_4DW_NoData || fmt == FMT_4DW_WithData) {
				uint32_t val;

				assert(b_len >= TLPHdr_4DW_Sz);

				val = hdr[i] << 24 |
					hdr[i+1] << 16 |
					hdr[i+2] << 8 |
					hdr[i+3];

				m_hdr.push_back(val);

				i+=4;
			}

			m_tlp_dw_len = get_length() ? get_length() : MAX_DW_LEN;

			if (fmt == FMT_4DW_WithData ||
				fmt == FMT_3DW_WithData) {
				uint32_t data_u32_idx = i / 4;
				uint32_t *data_u32;
				uint8_t *data;

				assert(b_len >= (i + m_tlp_dw_len * 4));

				//
				// Copy the data
				//
				data = &hdr[i];
				for (uint32_t j = 0; j < (m_tlp_dw_len * 4); j+=4) {
					uint32_t *val = reinterpret_cast<uint32_t*>(&data[j]);
					m_hdr.push_back(val[0]);
				}

				//
				// Store the data location
				//
				data_u32 = &(m_hdr.data()[data_u32_idx]);
			        m_tlp_data = reinterpret_cast<uint8_t*>(data_u32);
			}
		}

		uint32_t GetTLPDWLength() { return m_tlp_dw_len; }
		uint8_t *GetTLPData() { return m_tlp_data; }

		//
		// Section 2.2.7 [1]
		//
		uint32_t GetTxID()
		{
			return get_requestorID() << 16 | get_t9() << 9 |
				get_t8() << 8 | get_tag();
		}

		bool IsMemRd()
		{
			//
			// FMT_4DW_NoData / FMT_3DW_NoData is read
			//
			switch(get_fmt()) {
				case FMT_4DW_NoData:
				case FMT_3DW_NoData:
					return true;
				default:
					break;
			}
			return false;
		}

		uint64_t GetAddress()
		{
			uint32_t fmt = get_fmt();
			bool is_4dw_hdr = (fmt == FMT_4DW_WithData) ||
						(fmt == FMT_4DW_NoData);
			uint64_t addr;

			if (is_4dw_hdr) {
				addr = get_4DWHdr_addr_63_32();
				addr <<= 32;
				addr |= get_4DWHdr_addr_31_2() << 2;
			} else {
				addr = get_3DWHdr_addr_31_2() << 2;
			}

			return addr;
		}

		uint8_t *m_tlp_data;
		uint32_t m_tlp_dw_len;
	};

	class TLP_Cpl: public TLPHdr_Base
	{
	public:
		//
		// For bits [63:32]
		//
		GEN_TLP_HDR_FIELD_FUNCS(1, completerID, 16, 16);
		GEN_TLP_HDR_FIELD_FUNCS(1, cplStatus, 13, 3);
		GEN_TLP_HDR_FIELD_FUNCS(1, bcm, 12, 1);
		GEN_TLP_HDR_FIELD_FUNCS(1, byteCount, 0, 12);

		//
		// For bits [95:64]
		//
		GEN_TLP_HDR_FIELD_FUNCS(2, requestorID, 16, 16);
		GEN_TLP_HDR_FIELD_FUNCS(2, tag, 8, 8);
		GEN_TLP_HDR_FIELD_FUNCS(2, rsvd0, 7, 1);
		GEN_TLP_HDR_FIELD_FUNCS(2, lowerAddress, 0, 7);

		TLP_Cpl(tlm::tlm_generic_payload *gp) :
			TLPHdr_Base(gp)
		{
			//
			// A failed read (e.g. an IOMMU fault in the host) is
			// completed with a Cpl (no data) and UR status.
			//
			bool ok = gp->get_response_status() ==
					tlm::TLM_OK_RESPONSE;
			uint32_t fmt = (gp->is_read() && ok) ?
					FMT_3DW_WithData : FMT_3DW_NoData;
			unsigned char *be = gp->get_byte_enable_ptr();
			int be_len = gp->get_byte_enable_length();
			genattr_extension *genattr;
			uint32_t masterID = 0;
			uint32_t txID = 0;
			uint32_t lowerAddr = 0;
			uint32_t byteCount;
			uint32_t cpl_status;
			uint32_t dlen;
			unsigned int first_en = 0;
			unsigned int last_en = 0;
			bool any_en = false;

			//
			// Bits [31:0]
			//
			dlen = gp->get_data_length()/4;
			if (gp->get_data_length() % 4) {
				//
				// 1 more DW is needed if len is not a multiple
				// of 4.
				//
				dlen++;
			}
			if (fmt == FMT_3DW_NoData) {
				dlen = 0;
			}

			m_hdr.push_back(set_fmt(fmt) |
					set_type(Type_Cpl) |

					set_t9(0) |
					set_tc(0) |
					set_t8(0) |
					set_attr2(0) |
					set_ln(0) |
					set_th(0) |

					set_td(0) |
					set_ep(0) |
					set_attr_1_0(0) |
					set_at(0) |

					set_length(dlen == MAX_DW_LEN ? 0 : dlen));

			//
			// The byte enables of the gp carry the First/Last
			// DW BE of the request, find the first and last
			// requested byte ([1] 2.3.1.1).
			//
			for (unsigned int i = 0; i < gp->get_data_length(); i++) {
				if (!be || !be_len ||
				    be[i % be_len] == TLM_BYTE_ENABLED) {
					if (!any_en) {
						first_en = i;
					}
					last_en = i;
					any_en = true;
				}
			}

			//
			// For bits [63:32]
			//
			cpl_status = (gp->get_response_status() ==
					tlm::TLM_OK_RESPONSE) ? Cpl_SC : Cpl_UR;
			//
			// Completer ID not supported (needs to be
			// provided with the gp).
			//
			// ByteCount = remaining bytes in the request
			// (upper layer is responsible of creating
			// correctly sized Cpl responses). A zero length
			// read is completed with a byte count of 1.
			//
			byteCount = any_en ? last_en - first_en + 1 : 1;
			m_hdr.push_back(set_completerID(0) |
					set_cplStatus(cpl_status) |
					set_bcm(0) |
					set_byteCount(byteCount == SZ_4K ?
							0 : byteCount));

			//
			// For bits [95:64]
			//
			gp->get_extension(genattr);
			if (genattr) {
				masterID = genattr->get_master_id();
				txID = genattr->get_transaction_id();
			}

			lowerAddr = (gp->get_address() + first_en) & 0x7F;

			m_hdr.push_back(set_requestorID(masterID) |
					set_tag(txID) |
					set_lowerAddress(lowerAddr));

			//
			// Data if it is a CplD
			//
			if (dlen) {
				uint8_t *d = gp->get_data_ptr();
				unsigned int len = gp->get_data_length();
				uint32_t pos = 0;

				while (pos < len) {
					uint32_t val = 0;

					if (pos < len) {
						val |= d[pos] << 24;
					}
					if (pos + 1  < len) {
						val |= d[pos + 1] << 16;
					}
					if (pos + 2  < len) {
						val |= d[pos + 2] <<  8;
					}
					if (pos + 3  < len) {
						val |= d[pos + 3];
					}
					pos += 4;

					m_hdr.push_back(val);
				}
			}
			m_done = true;

			//
			// Setup m_tlp
			//
			// command should perhaps always be TLM_WRITE_COMMAND???
			//
			// (Check TLM spec if read means data can be manipulated)
			//
			m_tlp.set_command(gp->get_command());
			m_tlp.set_address(0); // unused

			m_tlp.set_data_ptr(reinterpret_cast<uint8_t*>(m_hdr.data()));
			m_tlp.set_data_length(m_hdr.size() * 4);
			m_tlp.set_byte_enable_ptr(nullptr);
			m_tlp.set_byte_enable_length(0);
			m_tlp.set_streaming_width(m_hdr.size() * 4);
			m_tlp.set_dmi_allowed(false);
			m_tlp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		}

		TLP_Cpl(uint8_t *hdr, unsigned int len)
		{
			unsigned int i = 0;

			assert(len >= TLPHdr_Cpl_Sz);

			for (; i < TLPHdr_Cpl_Sz; i+=4) {
				uint32_t val = hdr[i] << 24 |
					hdr[i+1] << 16 |
					hdr[i+2] << 8 |
					hdr[i+3];

				m_hdr.push_back(val);
			}

			if (get_fmt() == FMT_3DW_WithData) {
				unsigned int dw_len = get_length() ?
							get_length() :
							MAX_DW_LEN;

				for (unsigned int j = 0;
				     j < dw_len && (i + 4) <= len;
				     j++, i += 4) {
					uint32_t val = hdr[i] << 24 |
						hdr[i+1] << 16 |
						hdr[i+2] << 8 |
						hdr[i+3];
					m_hdr.push_back(val);
				}
			}
		}

		uint32_t GetTxID()
		{
			return get_requestorID() << 16 | get_t9() << 9 |
				get_t8() << 8 | get_tag();
		}

		bool IsCplD()
		{
			return get_fmt() == FMT_3DW_WithData;
		}
	};

	SC_HAS_PROCESS(tlm2tlp_bridge);

	tlm2tlp_bridge(sc_core::sc_module_name name):
		sc_module(name),

		//
		// TLM side
		//
		cfg_tgt_socket("cfg-tgt-socket"),
		mem_tgt_socket("mem-tgt-socket"),
		io_tgt_socket("io-tgt-socket"),
		msg_tgt_socket("msg-tgt-socket"),
		dma_init_socket("dma_init_socket"),

		//
		// TLP side
		//
		init_socket("init_socket"),
		tgt_socket("tgt-socket"),

		m_rx_event("rx-event"),
		m_tlp_done_event("tlp-done-event"),

		m_dma_event("dma-event")
	{
		cfg_tgt_socket.register_b_transport(this,
				&tlm2tlp_bridge::cfg_b_transport);
		mem_tgt_socket.register_b_transport(this,
				&tlm2tlp_bridge::mem_b_transport);
		io_tgt_socket.register_b_transport(this,
				&tlm2tlp_bridge::io_b_transport);
		msg_tgt_socket.register_b_transport(this,
				&tlm2tlp_bridge::msg_b_transport);

		//
		// Setup TLP rx tgt_socket
		//
		tgt_socket.register_b_transport(this,
				&tlm2tlp_bridge::b_transport);

		SC_THREAD(dma_thread);
	}

	void transmitTLP(TLPHdr_Base *tlp, sc_time& delay)
	{
		//
		// Multiple oustanding TLPs not supported
		//
		while (!m_tlps.empty()) {
			wait(m_tlp_done_event);
		}

		m_tlps.push_back(tlp);

		tlp->ByteSwap();
		init_socket->b_transport(*tlp->GetTLPGP(), delay);

		//
		// Wait for the delay here (and allow the TLP to get processed)
		//
		wait(delay);
		delay = SC_ZERO_TIME;

		while (!tlp->Done()) {
			wait(m_rx_event);
		}

		m_tlps.remove(tlp);
		m_tlp_done_event.notify();
	}

	template<typename T>
	void transmit(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		T tlp(&trans);

		transmitTLP(reinterpret_cast<TLPHdr_Base*>(&tlp), delay);
	}

	void transmitCplD(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		TLP_Cpl tlp(&trans);

		tlp.ByteSwap();
		init_socket->b_transport(*tlp.GetTLPGP(), delay);

		//
		// Wait for the delay here (and allow the TLP to get processed)
		//
		wait(delay);
		delay = SC_ZERO_TIME;
	}

	void cfg_b_transport(tlm::tlm_generic_payload& trans,
			sc_time& delay)
	{
		transmit<TLP_Conf>(trans, delay);
	}

	void mem_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		transmit<TLP_Mem>(trans, delay);
	}

	void io_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		SC_REPORT_ERROR("tlm2tlp-bridge",
			"I/O transactions are currently not supported");
	}

	void msg_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		SC_REPORT_ERROR("tlm2tlp-bridge",
			"Message transactions are currently not supported");
	}

	TLPHdr_Base *LookupRequestTLP(std::list<TLPHdr_Base*>& l, uint32_t txID)
	{
		for (typename std::list<TLPHdr_Base*>::iterator it = l.begin();
				it != l.end(); it++) {

			TLPHdr_Base *t = (*it);

			if (t->GetTxID() == txID) {
				return t;
			}
		}
		return nullptr;
	}

	GEN_FIELD_FUNCS(type, 0, 5); // Get type from the first byte

	void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
	{
		unsigned int dlen = trans.get_data_length();
		uint8_t *d = trans.get_data_ptr();
		uint32_t d0 = (dlen > 0) ? d[0] : 0;

		switch (get_type(d0)) {
		case Type_Cpl:
		{
			TLP_Cpl cpl(trans.get_data_ptr(), trans.get_data_length());
			TLPHdr_Base *tlp = LookupRequestTLP(m_tlps, cpl.GetTxID());

			if (tlp) {
				bool resp_ok = (tlp->ExpectsData() && cpl.IsCplD()) ||
					(!tlp->ExpectsData() && !cpl.IsCplD());

				tlp->SetDone();

				if (resp_ok) {
					// Only on ok resp, no need to copy on error
					if (tlp->ExpectsData() && cpl.IsCplD()) {
						tlp->CopyData(&cpl);
					}

					tlp->GetStoredGP()->set_response_status(
							tlm::TLM_OK_RESPONSE);
				} else {
					//
					// Error occured
					//
					tlp->GetStoredGP()->set_response_status(
							tlm::TLM_GENERIC_ERROR_RESPONSE);
				}
			} else {
				SC_REPORT_ERROR("pcie_root_port",
						"Received an unexpected GP");
			}

			m_rx_event.notify();
			break;
		}
		case Type_MRdWr:
		{
			TLP_Mem *tlp = new TLP_Mem(trans.get_data_ptr(),
							trans.get_data_length());

			m_dma_tlps.push_back(tlp);
			m_dma_event.notify();

			wait(tlp->DMADoneEvent());

			break;
		}
		default:
			//
			// Messages (Type 10rrr, [1] 2.2.8). INTx is not
			// propagated (there is no INTx towards the host),
			// Assert/Deassert_INTx are dropped. libpcie sends
			// them for example when the host toggles the
			// Interrupt Disable bit in the command register.
			//
			if ((get_type(d0) & 0x18) == 0x10 && dlen >= 8) {
				uint8_t msg_code = d[7];

				if (msg_code < 0x20 || msg_code > 0x27) {
					SC_REPORT_WARNING("pcie_root_port",
						"Dropping unsupported message TLP");
				}
				break;
			}
			SC_REPORT_ERROR("pcie_root_port", "Received an unexpected GP");
			break;
		}

		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}

	genattr_extension *get_genattr(TLP_Mem *t)
	{
		genattr_extension *genattr = new genattr_extension();

		genattr->set_master_id(t->get_requestorID());
		genattr->set_transaction_id(t->get_tag());

		return genattr;
	}

	//
	// Address Type: requests with AT = Translated carry a physical address
	// and bypass the host IOMMU (marked with ATTR_PHYS_ADDR, QEMU's
	// remote-port memory slave then skips the IOMMU translation).
	//
	enum { AT_UNTRANSLATED = 0, AT_TRANSLATION_REQ = 1, AT_TRANSLATED = 2 };

	void set_atsattr(TLP_Mem *t, tlm::tlm_generic_payload &gp)
	{
		atsattr_extension *atsattr;

		if (t->get_at() != AT_TRANSLATED) {
			return;
		}
		atsattr = new atsattr_extension();
		atsattr->set_attributes(atsattr_extension::ATTR_PHYS_ADDR);
		gp.set_extension(atsattr);
	}

	//
	// Expands the First/Last DW BE of a memory request TLP into TLM
	// byte enables, returns false if all bytes are enabled.
	//
	bool FillByteEnables(TLP_Mem *t, uint8_t *be, unsigned int dlen)
	{
		uint32_t firstDWBE = t->get_firstDWBE();
		uint32_t lastDWBE = (dlen > 4) ? t->get_lastDWBE() : firstDWBE;
		bool partial = false;

		for (unsigned int i = 0; i < dlen; i++) {
			bool en = true;

			if (i < 4) {
				en = firstDWBE & (1 << i);
			} else if (i >= dlen - 4) {
				en = lastDWBE & (1 << (i % 4));
			}

			be[i] = en ? TLM_BYTE_ENABLED : TLM_BYTE_DISABLED;
			if (!en) {
				partial = true;
			}
		}
		return partial;
	}

	void ProcessReadDMA(TLP_Mem *t)
	{
		genattr_extension *genattr = get_genattr(t);
		unsigned int dlen = t->GetTLPDWLength() * 4;
		sc_time delay(SC_ZERO_TIME);
		tlm::tlm_generic_payload gp;
		uint8_t data[SZ_4K];
		uint8_t be[SZ_4K];

		gp.set_command(tlm::TLM_READ_COMMAND);
		gp.set_address(t->GetAddress());
		gp.set_data_ptr(data);
		gp.set_data_length(dlen);
		gp.set_streaming_width(dlen);
		gp.set_byte_enable_ptr(nullptr);
		gp.set_byte_enable_length(0);
		gp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

		gp.set_extension(genattr);
		set_atsattr(t, gp);

		dma_init_socket->b_transport(gp, delay);
		wait(delay);
		delay = SC_ZERO_TIME;
		if (gp.get_response_status() != tlm::TLM_OK_RESPONSE) {
			// Completed with UR by TLP_Cpl
			SC_REPORT_WARNING("pcie_root_port",
				"DMA read failed in the host (IOMMU fault?), "
				"completing with UR");
		}

		//
		// Whole DWs are read above, the requested bytes are
		// passed to the completion through the byte enables
		// (for the byte count and lower address fields).
		//
		if (FillByteEnables(t, be, dlen)) {
			gp.set_byte_enable_ptr(be);
			gp.set_byte_enable_length(dlen);
		}

		//
		// transmit the DMA read completion
		//
		transmitCplD(gp, delay);
	}

	void ProcessWriteDMA(TLP_Mem *t)
	{
		genattr_extension *genattr = get_genattr(t);
		unsigned int dlen = t->GetTLPDWLength() * 4;
	        sc_time delay(SC_ZERO_TIME);
		tlm::tlm_generic_payload gp;
		uint8_t be[SZ_4K];

		gp.set_command(tlm::TLM_WRITE_COMMAND);
		gp.set_address(t->GetAddress());
		gp.set_data_ptr(t->GetTLPData());
		gp.set_data_length(dlen);
		gp.set_streaming_width(dlen);
		gp.set_byte_enable_ptr(nullptr);
		gp.set_byte_enable_length(0);
		gp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

		//
		// Only write the bytes enabled by First/Last DW BE
		//
		if (FillByteEnables(t, be, dlen)) {
			gp.set_byte_enable_ptr(be);
			gp.set_byte_enable_length(dlen);
		}

		gp.set_extension(genattr);
		set_atsattr(t, gp);

		dma_init_socket->b_transport(gp, delay);
		wait(delay);
		delay = SC_ZERO_TIME;
		if (gp.get_response_status() != tlm::TLM_OK_RESPONSE) {
			SC_REPORT_WARNING("pcie_root_port",
				"DMA write failed in the host (IOMMU fault?), "
				"dropped");
		}
		//
		// Writes are posted
		//
	}

	void dma_thread()
	{
		while (true) {
			TLP_Mem *tlp;

			if (m_dma_tlps.empty()) {
				wait(m_dma_event);
			}

			tlp = m_dma_tlps.front();
			m_dma_tlps.remove(tlp);

			if (tlp->IsMemRd()) {
				ProcessReadDMA(tlp);
			} else {
				ProcessWriteDMA(tlp);
			}

			tlp->DMADoneEvent().notify();
			delete tlp;
		}
	}

	std::list<TLPHdr_Base*> m_tlps;
	std::list<TLP_Mem*> m_dma_tlps;

	sc_event m_rx_event;
	sc_event m_tlp_done_event;
	sc_event m_dma_event;
};

#undef GEN_FIELD_FUNCS
#undef GEN_TLP_HDR_FIELD_FUNCS
#endif
