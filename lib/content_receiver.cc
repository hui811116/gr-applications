/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <applications/content_receiver.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace applications {

    class content_receiver_impl : public content_receiver
    {
    public:
    	content_receiver_impl() : block("content_receiver",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_in_port(pmt::mp("pkt_in")),
    		d_cnt_port(pmt::mp("content_out")),
    		d_msg_port(pmt::mp("msg_out"))
    	{
    		message_port_register_out(d_cnt_port);
    		message_port_register_out(d_msg_port);
    		message_port_register_in(d_in_port);
    		set_msg_handler(d_in_port,boost::bind(&content_receiver_impl::msg_in,this,_1));
    		d_buf = new char[512*512];
    		d_buf_size = 512 *512;
    		reset_counters();
    	}

    	~content_receiver_impl()
    	{
    		delete [] d_buf;
    	}
    	
    	void msg_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    		size_t io(0);
    		const uint8_t* uvec = pmt::u8vector_elements(v,io);
    		// read headers
    		uint16_t rx_pkt_cnt=0x00, rx_total_pkt=0x00;
    		rx_total_pkt = uvec[0];
    		rx_total_pkt |= (uvec[1] << 8);
    		rx_pkt_cnt = uvec[2];
    		rx_pkt_cnt |= (uvec[3] << 8);
    		std::cout<<"decoded header: total_packets="<<rx_total_pkt<<" ,packet_cnt="<<rx_pkt_cnt<<std::endl;
    	}


	private:
		void reset_counters()
		{
			d_total_pkt = 0;
			d_pkt_cnt = 0;
			d_bytes_cnt =0;
		}
		void _mem_check_and_resize(int size)
		{
			if(size>d_buf_size){
				delete [] d_buf;
				d_buf = new char[size];
				d_buf_size = size;
			}
		}
		char* d_buf;
		int d_buf_size;
		int d_total_pkt;
		int d_pkt_cnt;
		int d_bytes_cnt;
		const pmt::pmt_t d_in_port;
		const pmt::pmt_t d_cnt_port; // for received content
		const pmt::pmt_t d_msg_port; // for acknowledgement
    };

    content_receiver::sptr
    content_receiver::make()
    {
    	return gnuradio::get_initial_sptr(
    		new content_receiver_impl());
    }

  } /* namespace applications */
} /* namespace gr */

