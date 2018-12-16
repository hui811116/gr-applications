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
#include <applications/content_sender.h>
#include <cstring>
#include <cmath>

namespace gr {
  namespace applications {
  	#define SENDER_HDR_SIZE 4
    class content_sender_impl : public content_sender
    {
    public:
    	content_sender_impl(int bytesPerPacket, bool useAck) : block("content_sender",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_src_port(pmt::mp("src_in")),
    		d_out_port(pmt::mp("msg_out")),
    		d_in_port(pmt::mp("msg_in"))
    	{
    		message_port_register_out(d_out_port);
    		message_port_register_in(d_src_port);
    		message_port_register_in(d_in_port);
    		set_msg_handler(d_src_port,boost::bind(&content_sender_impl::src_in,this,_1));
    		set_msg_handler(d_in_port,boost::bind(&content_sender_impl::msg_in,this,_1));

    		d_mem = new uint8_t[1024*1024+SENDER_HDR_SIZE]; // default content size
    		d_mem_size = 1024*1024;
    		d_busy = false;
            d_useAck = useAck;
            d_bytes_per_packet = (bytesPerPacket>0)? bytesPerPacket : 1024;
    	}
    	~content_sender_impl()
    	{

    	}
    	void src_in(pmt::pmt_t src)
    	{
    		if(d_busy){
    			return;
    		}
    		pmt::pmt_t k = pmt::car(src);
    		pmt::pmt_t v = pmt::cdr(src);
    		size_t io(0);
    		const uint8_t* uvec = pmt::u8vector_elements(v,io);
    		d_total_bytes = io;
    		d_total_packets = (uint16_t) std::ceil(io/(double)d_bytes_per_packet);
    		_mem_check();
    		memcpy(d_mem+SENDER_HDR_SIZE, uvec, io); // save 4 bytes space for header
    		_init_sender();
    		// set busy
    		d_busy = true;
    		d_init_session.notify_one();
    	}
    	void msg_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    		//FIXME
    		d_get_ack.notify_one();
    	}
    	bool start()
    	{
    		d_finished = false;
    		d_thread = boost::shared_ptr<gr::thread::thread>(
    			new gr::thread::thread(boost::bind(&content_sender_impl::run,this)));
    		return block::start();
    	}
    	bool stop()
    	{
    		d_finished = true;
    		d_init_session.notify_one();
    		d_get_ack.notify_one();
    		d_thread->interrupt();
    		d_thread->join();
    		return block::stop();
    	}
    private:
    	void _init_sender(){
    		d_send_cnt =0;
    		d_ack_cnt = 0;
    	}
    	void _mem_check(){
    		if(d_mem_size < d_total_bytes){
    			delete [] d_mem;
    			d_mem = new uint8_t[d_total_bytes+SENDER_HDR_SIZE];
    			d_mem_size = d_total_bytes;
    		}
    	}
    	void _gen_header(){
    		// header used for reconstruction
    		// design: 4 bytes
    		// details: 16 bits for number of packets, 16 bits for count
    		uint8_t* tbytes = (uint8_t*) & d_total_packets;
    		d_mem[d_send_cnt*d_bytes_per_packet] = tbytes[0];
    		d_mem[d_send_cnt*d_bytes_per_packet+1] = tbytes[1];
    		uint8_t* u8vec = (uint8_t*) & d_send_cnt;
    		d_mem[d_send_cnt*d_bytes_per_packet+2] = u8vec[0];
    		d_mem[d_send_cnt*d_bytes_per_packet+3] = u8vec[1];
    	}
    	void gen_pkt(){
    		_gen_header();
    		int tx_bytes = ( (d_send_cnt+1) == d_total_packets)? 
    					   d_total_bytes-d_send_cnt*d_bytes_per_packet : d_bytes_per_packet;
    		d_pkt = pmt::make_blob(d_mem+d_send_cnt*d_bytes_per_packet,tx_bytes+SENDER_HDR_SIZE);
    	}
    	void run()
    	{
    		if(!d_useAck){
    			while(!d_finished){
    				gr::thread::scoped_lock lock(d_mutex);
    				d_init_session.wait(lock);
    				lock.unlock();
    				while(!d_finished && (d_send_cnt<d_total_packets)){
    					gen_pkt();   		
    					message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,d_pkt));
    					d_send_cnt++;
    				}
    				if(d_send_cnt == d_total_packets){
    					d_busy = false;
    				}
    			}
    		}else{
    			while(!d_finished){
    				gr::thread::scoped_lock lock(d_mutex);
    				d_init_session.wait(lock);
    				lock.unlock();
    				while(!d_finished && (d_send_cnt<d_total_packets)){
    					gen_pkt();
    					message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,d_pkt));
    					d_get_ack.timed_wait(lock,boost::posix_time::milliseconds(1000));
    					lock.unlock();
    					if(d_finished){
    						return;
    					}else if(d_ack_cnt == d_send_cnt+1){
    						d_send_cnt++;
    					}else{
    						// failed;
    					}
    				}
    				if(d_send_cnt == d_total_packets){
    					d_busy = false;
    				}
    			}
    		}
    	}
    	const pmt::pmt_t d_src_port;
    	const pmt::pmt_t d_out_port;
    	const pmt::pmt_t d_in_port;
    	boost::shared_ptr<gr::thread::thread> d_thread;
    	gr::thread::mutex d_mutex;
    	gr::thread::condition_variable d_init_session;
    	gr::thread::condition_variable d_get_ack;
    	uint8_t* d_mem;
    	int d_mem_size;
    	bool d_busy;
    	bool d_finished;
    	// system parameters
    	int d_bytes_per_packet;
    	bool d_useAck;
    	// sender counters
    	int d_total_bytes;
    	uint16_t d_send_cnt;
    	uint16_t d_total_packets;
    	int d_ack_cnt;
    	pmt::pmt_t d_pkt;
    };

    content_sender::sptr
    content_sender::make(int bytesPerPacket,bool useAck)
    {
    	return gnuradio::get_initial_sptr( 
            new content_sender_impl(bytesPerPacket,useAck));
    }
  } /* namespace applications */
} /* namespace gr */

