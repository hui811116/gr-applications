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
#include <cstring>

namespace gr {
  namespace applications {
    #define d_debug 0
    #define dout d_debug && std::cout
  	#define CONTENT_HEADER_BYTES 4
  	#define RX_ACK_BYTES 4
    class content_receiver_impl : public content_receiver
    {
    public:
    	content_receiver_impl(bool returnAck) : block("content_receiver",
    			gr::io_signature::make(0,0,0),
    			gr::io_signature::make(0,0,0)),
    			d_pkt_in(pmt::mp("pkt_in")),
    			d_ack_out(pmt::mp("ack_out")),
    			d_cnt_out(pmt::mp("content_out"))
    	{
    		message_port_register_in(d_pkt_in);
    		message_port_register_out(d_ack_out);
    		message_port_register_out(d_cnt_out);
    		set_msg_handler(d_pkt_in,boost::bind(&content_receiver_impl::msg_in,this,_1));
    		d_buf = new char[1024*1024];
    		d_buf_size = 1024 *1024;
    		d_ack = returnAck;
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
            if(io<CONTENT_HEADER_BYTES)
                return;
    		// read headers
    		uint16_t rx_pkt_cnt = 0x0000, rx_total_pkt = 0x0000;
    		rx_total_pkt = uvec[0];
    		rx_total_pkt|= (uvec[1]<<8);
    		rx_pkt_cnt = uvec[2];
    		rx_pkt_cnt |= (uvec[3]<<8);
    		if(rx_pkt_cnt == 0){
                dout<<"resetting content receiver"<<std::endl;
    			mem_check_and_resize(io * rx_total_pkt);
    			reset_counters();
    		}
    		if(d_ack){
                dout<<"Content Receiver, ACK MODE, received index="<<rx_pkt_cnt<<std::endl;
                for(int i=0;i<4;i++)
                    d_ack_msg[i] = uvec[i%2+2];
                pmt::pmt_t ackMsg = pmt::make_blob(d_ack_msg,RX_ACK_BYTES);
                message_port_pub(d_ack_out,pmt::cons(pmt::PMT_NIL,ackMsg));
                if(d_rx_bytes + (io-CONTENT_HEADER_BYTES) > d_buf_size){
                    throw std::runtime_error("ERROR: Content Receiver Memory overflow");
                }
                if(rx_pkt_cnt == d_ack_cnt){
                    dout<<"Content Receiver, Write bytes-- ack_cnt="<<d_ack_cnt<<", bytes_write="<<(io-CONTENT_HEADER_BYTES)<<std::endl;
                    d_ack_cnt++;
                    memcpy(&d_buf[d_rx_bytes],uvec+CONTENT_HEADER_BYTES,io-CONTENT_HEADER_BYTES);
                    d_rx_bytes += (io-CONTENT_HEADER_BYTES);    
                }
    		}else{
                if(d_rx_bytes + (io-CONTENT_HEADER_BYTES) > d_buf_size){
                    throw std::runtime_error("ERROR: Content Receiver Memory overflow");
                }
                memcpy(&d_buf[d_rx_bytes],uvec+CONTENT_HEADER_BYTES,io-CONTENT_HEADER_BYTES);
                d_rx_bytes += (io-CONTENT_HEADER_BYTES);
            }
            
            
    		if(rx_pkt_cnt == rx_total_pkt-1){
    			// potentially received all
                dout<<"received bytes count="<<d_rx_bytes<<std::endl;
    			pmt::pmt_t contentOut = pmt::make_blob(d_buf, d_rx_bytes);
    			message_port_pub(d_cnt_out,pmt::cons(pmt::PMT_NIL, contentOut));
    		}
    	}
    private:
    	void mem_check_and_resize(int size){
    		if(size>d_buf_size){
                dout<<"CONTENT_RECEIVER: resizing buffer, from:"<<d_buf_size<<" to:"<<size<<std::endl;
    			delete [] d_buf;
    			d_buf = new char[size];
    			d_buf_size = size;
    		}
    	}
    	void reset_counters()
    	{
            gr::thread::scoped_lock guard(d_mutex);
    		d_rx_bytes =0;
            d_ack_cnt =0;
    	}
        gr::thread::mutex d_mutex;
    	const pmt::pmt_t d_pkt_in;
    	const pmt::pmt_t d_ack_out;
    	const pmt::pmt_t d_cnt_out;
    	char* d_buf;
    	char d_ack_msg[4];
    	int d_buf_size;
    	int d_rx_bytes;
        int d_ack_cnt;
    	bool d_ack;
    };

    content_receiver::sptr
    content_receiver::make(bool returnAck)
    {
    	return gnuradio::get_initial_sptr(new content_receiver_impl(returnAck));
    }

  } /* namespace applications */
} /* namespace gr */

