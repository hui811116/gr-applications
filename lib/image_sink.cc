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
#include <applications/image_sink.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace applications {

    class image_sink_impl : public image_sink
    {
    public:
    	image_sink_impl() : block("image_sink",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_in_port(pmt::mp("cnt_in"))
    	{
    		message_port_register_in(d_in_port);
    		set_msg_handler(d_in_port,boost::bind(&image_sink_impl::msg_in,this,_1));
    	}
    	~image_sink_impl()
    	{
    		
    	}
    	void msg_in(pmt::pmt_t msg)
    	{

    	}
    private:
    	const pmt::pmt_t d_in_port;

    };

  } /* namespace applications */
} /* namespace gr */

