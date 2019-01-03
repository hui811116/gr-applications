/* -*- c++ -*- */
/* 
 * Copyright 2019 <+YOU OR YOUR COMPANY+>.
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
#include <applications/transmit_ctrl_sender.h>
#include <gnuradio/block.h>
#include <utility>
#include <map>
#include <atomic>
#include <thread>

namespace gr {
  namespace applications {
    #define d_debug 0
    #define dout d_debug && std::cout
    #define HDRBYTES 4
    class transmit_ctrl_sender_impl : public transmit_ctrl_sender
    {
    public:
    	transmit_ctrl_sender_impl(int timeout_ms, int windowsize, int bytes_per_pkt) : block("transmit_ctrl",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_item_port(pmt::mp("cnt_in")),
    		d_ack_port(pmt::mp("ack_in")),
    		d_pkt_port(pmt::mp("pkt_out")),
    		d_max_cap(1024*1024),
            d_windowsize(windowsize),
            d_timeout_ms(timeout_ms)
    	{
    		message_port_register_in(d_item_port);
    		message_port_register_in(d_ack_port);
    		message_port_register_out(d_pkt_port);
    		set_msg_handler(d_item_port,boost::bind(&transmit_ctrl_sender_impl::content_in,this,_1));
    		set_msg_handler(d_ack_port,boost::bind(&transmit_ctrl_sender_impl::ack_in,this,_1));
    		d_busy = false;
            d_bytes_per_pkt = bytes_per_pkt; // for debugging
            d_total_pkts = 0;
            d_arq_head.store(0);
    	}
    	~transmit_ctrl_sender_impl()
    	{

    	}
    	void ack_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    		size_t io(0);
    		const uint8_t * uvec = pmt::u8vector_elements(v,io);
    		if(io<4 ||  !(uvec[0]==uvec[2] && uvec[1]==uvec[3]) ){
    			return;
    		}
    		uint16_t u16hdr = uvec[0];
            u16hdr |= (uvec[1]<<8);
            // remove from pend, add to ack
            if(dequeue_pend(u16hdr)){
                // inform main thread to send new packet
                d_fctrl.notify_one();
            }
    		if(u16hdr == d_arq_head.load()){
                d_arq_head++; // atomic addition
    		}
    	}
    	void content_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    		size_t io(0);
    		const uint8_t* uvec = pmt::u8vector_elements(v,io);
    		if(io>d_max_cap)
    			return;
    		if(d_busy)
    			return;
    		memcpy(d_buffer, uvec, sizeof(char)* io);
            d_total_pkts = (uint16_t) std::ceil(io / (float)d_bytes_per_pkt);
            d_last_bytes = io - d_bytes_per_pkt* (int)std::floor(io/d_bytes_per_pkt);
            uint8_t* u8vec = (uint8_t*) & d_total_pkts;
            d_pkt_buf[0] = u8vec[0];
            d_pkt_buf[1] = u8vec[1];
            d_arq_head.store(0);
            d_fctrl.notify_one();            
    	}
    	bool start()
    	{
    		d_finished = false;
    		d_sender_ctrl = boost::shared_ptr<gr::thread::thread>(
    			new gr::thread::thread(boost::bind(&transmit_ctrl_sender_impl::run,this)));
    		return block::start();
    	}
    	bool stop()
    	{
    		d_finished = true;
    		d_sender_ctrl -> interrupt();
            d_fctrl.notify_all();
    		d_sender_ctrl -> join();
    		return block::stop();
    	}
        
        void enqueue_timeout(uint16_t id)
        {
            std::map<uint16_t,bool>::iterator it = d_timeout_map.find(id);
            if(it!=d_timeout_map.end())
                return;
            d_timeout_map.insert(std::pair<uint16_t,bool>(id,false));
        }
        void enqueue_pend(uint16_t id)
        {
            std::map<uint16_t,bool>::iterator it = d_pend_map.find(id);
            if(it!=d_pend_map.end())
                return;
            d_pend_map.insert(std::pair<uint16_t,bool>(id,false));
        }
        bool dequeue_pend(uint16_t id)
        {
            std::map<uint16_t,bool>::iterator it = d_pend_map.find(id);
            if(it==d_pend_map.end())
                return false;
            d_pend_map.erase(it);
            return true;
        }
    private:
        void thread_timer(uint16_t pkt_id)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(d_timeout_ms));
            //dout<<"thread timer timeout...sendid="<<pkt_id<<std::endl;
            // try lock?
            while(!d_finished){
                if(d_mutex.try_lock()){
                    if(dequeue_pend(pkt_id)){
                        // timeout
                        enqueue_timeout(pkt_id);
                        d_fctrl.notify_one();
                    }
                    d_mutex.unlock();
                    break;
                }
            }
        }
        pmt::pmt_t genPkt(uint16_t id)
        {
            uint8_t* u8vec = (uint8_t*) & id;
            d_pkt_buf[2] = u8vec[0];
            d_pkt_buf[3] = u8vec[1];
            int sizeCpy = (id+1 ==d_total_pkts)? d_last_bytes : d_bytes_per_pkt;
            memcpy(&d_pkt_buf[4],&d_buffer[id*d_bytes_per_pkt],sizeCpy);
            return pmt::make_blob(d_pkt_buf,sizeCpy+HDRBYTES);
        }
    	void run()
    	{
            std::map<uint16_t,bool>::iterator it;
            pmt::pmt_t pkt;
            uint16_t send_id;
    		// thread operation
    		while(!d_finished)
    		{
                gr::thread::scoped_lock lock(d_mutex);
    			if(!d_busy){
    				d_fctrl.wait(lock);
    				lock.unlock();
    				if(d_total_pkts>d_pkt_send)
    					d_busy = true;
    			}else{
    				// running, check window size
                    if(d_pend_map.size()>=d_windowsize){
                        //dout<<"pending map size reached window size...wait for notification"<<std::endl;
                        d_fctrl.wait(lock);
                        lock.unlock();
                    }else{
                        if(!d_timeout_map.empty()){
                            it = d_timeout_map.begin();
                            // send it.first
                            send_id = it->first;
                            d_timeout_map.erase(it);    
                        }else{
                            send_id = d_pkt_send++;
                        }
                        enqueue_pend(send_id);
                        lock.unlock();
                        pkt = genPkt(send_id);
                        message_port_pub(d_pkt_port,pmt::cons(pmt::PMT_NIL,pkt));
                        // thread timer here
                        gr::thread::thread t(boost::bind(&transmit_ctrl_sender_impl::thread_timer,this,send_id));
                        t.detach();
                    }
                    if(d_arq_head.load()==d_total_pkts){
                        d_busy = false;
                    }
    			}
    		}
    	}
    	
    	const int d_max_cap;
        const int d_windowsize;
        const int d_timeout_ms;
    	const pmt::pmt_t d_item_port;
    	const pmt::pmt_t d_ack_port;
    	const pmt::pmt_t d_pkt_port;
    	gr::thread::mutex d_mutex;
    	boost::shared_ptr<gr::thread::thread> d_sender_ctrl;
    	gr::thread::condition_variable d_fctrl;
    	bool d_finished;
    	bool d_busy;
    	char d_buffer[1024*1024];
    	char d_pkt_buf[8200];
    	uint16_t d_total_pkts;
        int d_last_bytes;
    	int d_pkt_send;
        std::atomic<int> d_arq_head;
    	int d_bytes_per_pkt;
        std::map<uint16_t,bool> d_pend_map;
        std::map<uint16_t,bool> d_timeout_map;
    };

    transmit_ctrl_sender::sptr
    transmit_ctrl_sender::make(int timeout_ms, int windowsize, int bytes_per_pkt)
    {
    	return gnuradio::get_initial_sptr(new transmit_ctrl_sender_impl(timeout_ms,windowsize,bytes_per_pkt));
    }

  } /* namespace applications */
} /* namespace gr */

