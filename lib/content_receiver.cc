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
    			d_pkt_in(pmt::mp("pkt_in")),
    			d_ack_out(pmt::mp("ack_out")),
    			d_cnt_out(pmt::mp("content_out"))
    	{
    		message_port_register_in(d_pkt_in);
    		message_port_register_out(d_ack_out);
    		message_port_register_out(d_cnt_out);
    		set_msg_handler(d_pkt_in,boost::bind(&content_receiver_impl::msg_in,this,_1));
    	}
    	~content_receiver_impl()
    	{

    	}
    	void msg_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    	}
    private:
    	const pmt::pmt_t d_pkt_in;
    	const pmt::pmt_t d_ack_out;
    	const pmt::pmt_t d_cnt_out;
    };

    content_receiver::sptr
    content_receiver::make()
    {
    	return gnuradio::get_initial_sptr(new content_receiver_impl());
    }

  } /* namespace applications */
} /* namespace gr */

