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
#include <opencv2/opencv.hpp>
#include <string>

namespace gr {
  namespace applications {

    class image_sink_impl : public image_sink
    {
    public:
    	image_sink_impl(): block("image_sink",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_in_port(pmt::mp("img_in")),
    		d_win_name("Received Image")

    	{
    		message_port_register_in(d_in_port);
    		set_msg_handler(d_in_port,boost::bind(&image_sink_impl::img_in,this,_1));
            cv::namedWindow(d_win_name,cv::WINDOW_AUTOSIZE);
    	}
    	~image_sink_impl()
    	{

    	}
    	void img_in(pmt::pmt_t img)
    	{
    		pmt::pmt_t k = pmt::car(img);
    		pmt::pmt_t v = pmt::cdr(img);
    		size_t io(0);
    		const uint8_t* uvec = pmt::u8vector_elements(v,io);
    		d_image = cv::imdecode(cv::_InputArray(uvec,io),cv::IMREAD_ANYCOLOR);
    		if(!d_image.data){
    			// failed
    		}else{
    			cv::imshow(d_win_name,d_image);
    		}
    	}

    private:
    	const pmt::pmt_t d_in_port;
    	cv::Mat d_image;
    	const std::string d_win_name;
    };

    image_sink::sptr
    image_sink::make()
    {
    	return gnuradio::get_initial_sptr(new image_sink_impl());
    }

  } /* namespace applications */
} /* namespace gr */

