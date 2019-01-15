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
    #define CTRL_READY 0x0a
    class transmit_ctrl_sender_impl : public transmit_ctrl_sender
    {
    public:
    	transmit_ctrl_sender_impl(int timeout_ms, int windowsize, int bytes_per_pkt) : block("transmit_ctrl",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_item_port(pmt::mp("cnt_in")),
    		d_ack_port(pmt::mp("ack_in")),
    		d_pkt_port(pmt::mp("pkt_out")),
            d_ctrl_port(pmt::mp("ctrl_out")),
    		d_max_cap(1024*1024),
            d_windowsize(windowsize),
            d_timeout_ms(timeout_ms)
    	{
    		message_port_register_in(d_item_port);
    		message_port_register_in(d_ack_port);
    		message_port_register_out(d_pkt_port);
            message_port_register_out(d_ctrl_port);
    		set_msg_handler(d_item_port,boost::bind(&transmit_ctrl_sender_impl::content_in,this,_1));
    		set_msg_handler(d_ack_port,boost::bind(&transmit_ctrl_sender_impl::ack_in,this,_1));
    		d_busy.store(false);
            d_bytes_per_pkt = bytes_per_pkt; // for debugging
            d_total_pkts.store(0x0000);
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
                if(d_total_pkts.load()>0 && d_arq_head.load() == d_total_pkts.load()){
                    // reset map, index, busy
                    d_total_pkts.store(0);
                    d_busy.store(false);
                    message_port_pub(d_ctrl_port,pmt::cons(pmt::PMT_NIL,pmt::from_long(CTRL_READY)));
                    dout<<"arq head equals to total packets, reset system"<<std::endl;
                }
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
    		if(d_busy.load()== true){
                dout<<"called busy"<<std::endl;
    			return;
            }
    		memcpy(d_buffer, uvec, sizeof(char)* io);
            d_total_pkts = (uint16_t) std::ceil(io / (float)d_bytes_per_pkt);
            d_last_bytes = io - d_bytes_per_pkt* (int)std::floor(io/d_bytes_per_pkt);
            uint8_t* u8vec = (uint8_t*) & d_total_pkts;
            d_pkt_buf[0] = u8vec[0];
            d_pkt_buf[1] = u8vec[1];
            d_pkt_send = 0;
            d_timeout_map.clear();
            d_pend_map.clear();
            d_arq_head.store(0);
            d_busy.store(true);
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
            d_timeout_map.insert(std::pair<uint16_t,bool>(id,true));
        }
        void enqueue_pend(uint16_t id)
        {
            std::map<uint16_t,boost::posix_time::ptime>::iterator it = d_pend_map.find(id);
            if(it!=d_pend_map.end())
                return;
            d_pend_map.insert(std::pair<uint16_t,boost::posix_time::ptime>
                (id,boost::posix_time::microsec_clock::local_time()));
        }
        bool dequeue_pend(uint16_t id)
        {
            std::map<uint16_t,boost::posix_time::ptime>::iterator it = d_pend_map.find(id);
            if(it==d_pend_map.end())
                return false;
            d_pend_map.erase(it);
            return true;
        }
    private:
        void check_time()
        {
            std::map<uint16_t,boost::posix_time::ptime>::iterator it;
            boost::posix_time::time_duration diff; 
            for(it = d_pend_map.begin(); it != d_pend_map.end(); ++it ){
                diff = boost::posix_time::microsec_clock::local_time() - it->second;
                if(diff.total_milliseconds() >= d_timeout_ms){
                    enqueue_timeout(it->first);
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
    			// running, check window size
                if(d_pend_map.size()>=d_windowsize || d_busy.load() == false || d_pkt_send == d_total_pkts){
                    //dout<<"pending map size reached window size...wait for notification"<<std::endl;
                    gr::thread::scoped_lock lock(d_mutex);
                    d_fctrl.wait(lock);
                    lock.unlock();
                }else{
                    check_time(); // non detaching thread method
                    if(!d_timeout_map.empty()){
                        it = d_timeout_map.begin();
                        // send it.first
                        send_id = it->first;
                        d_timeout_map.erase(it);    
                    }else{
                        dout<<"run::sending new packet...id="<<d_pkt_send<<", total="<<d_total_pkts<<std::endl;
                        send_id = d_pkt_send++;
                    }
                    enqueue_pend(send_id);
                    pkt = genPkt(send_id);
                    message_port_pub(d_pkt_port,pmt::cons(pmt::PMT_NIL,pkt));
                    // non thread method, force sleep, but limited frame rate
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
    		}
    	}
    	
    	const int d_max_cap;
        const int d_windowsize;
        const int d_timeout_ms;
    	const pmt::pmt_t d_item_port;
    	const pmt::pmt_t d_ack_port;
    	const pmt::pmt_t d_pkt_port;
        const pmt::pmt_t d_ctrl_port;
    	gr::thread::mutex d_mutex;
    	boost::shared_ptr<gr::thread::thread> d_sender_ctrl;
    	gr::thread::condition_variable d_fctrl;
    	bool d_finished;
        std::atomic<bool> d_busy;
    	char d_buffer[1024*1024];
    	char d_pkt_buf[8200];
        std::atomic<uint16_t> d_total_pkts;
        int d_last_bytes;
    	int d_pkt_send;
        std::atomic<int> d_arq_head;
    	int d_bytes_per_pkt;
        std::map<uint16_t,boost::posix_time::ptime> d_pend_map;
        std::map<uint16_t,bool> d_timeout_map;
    };

    transmit_ctrl_sender::sptr
    transmit_ctrl_sender::make(int timeout_ms, int windowsize, int bytes_per_pkt)
    {
    	return gnuradio::get_initial_sptr(new transmit_ctrl_sender_impl(timeout_ms,windowsize,bytes_per_pkt));
    }

  } /* namespace applications */
} /* namespace gr */

