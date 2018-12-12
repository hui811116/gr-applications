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
#include <applications/image_source.h>
#include <gnuradio/block_detail.h>
#include <opencv2/opencv.hpp>
#include <sstream>

namespace gr {
  namespace applications {

    class image_source_impl : public image_source
    {
    public:
    	image_source_impl(const std::string& initFile): block("image_source",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_out_port(pmt::mp("bytes_out"))
    	{
    		message_port_register_out(d_out_port);
    		d_fname = initFile;
    		d_send_cnt = 0;
    	}
    	~image_source_impl()
    	{

    	}
    	bool start()
    	{
    		d_finished = false;
    		d_thread = boost::shared_ptr<gr::thread::thread>(
    			new gr::thread::thread(boost::bind(&image_source_impl::run,this)));
    		return block::start();
    	}

    	bool stop()
    	{
    		d_finished = true;
    		d_thread->interrupt();
    		d_thread->join();
    		return block::stop();
    	}
    	void set_fname(const std::string& filename){
    		gr::thread::scoped_lock guard(d_mutex);
    		if(!filename.empty() && d_fname!=filename){
    			d_fname = filename;
    		}
    	}
    	std::string get_fname() const{
    		return d_fname;
    	}
    	void set_resend(bool button){
    		if(button){
    			d_flow_ctrl.notify_one();
    		}
    	}

    private:
    	void run()
    	{
    		cv::Mat image;
    		while(!d_finished){
    			image = cv::imread( d_fname.c_str(), 1);
    			if( !image.data){
    				// no data
    			}else{
    				d_send_cnt++;
    				cv::namedWindow("Testing image", cv::WINDOW_AUTOSIZE);
    				std::stringstream ss;
    				ss<<"Testing imamge, total send:"<<d_send_cnt;
    				cv::imshow(ss.str(), image);
    				gr::thread::scoped_lock lock(d_mutex);
    				d_flow_ctrl.wait(lock);
    				//d_flow_ctrl.timed_wait(lock,boost::posix_time::milliseconds(1000));
    				lock.unlock();
    			}
    		}
    	}
    	const pmt::pmt_t d_out_port;
    	std::string d_fname;
    	boost::shared_ptr<gr::thread::thread> d_thread;
    	gr::thread::condition_variable d_flow_ctrl;
    	gr::thread::mutex d_mutex;
    	bool d_finished;
    	int d_send_cnt;
    };

    image_source::sptr
    image_source::make(const std::string& fname)
    {
    	return gnuradio::get_initial_sptr(new image_source_impl(fname));
    }

  } /* namespace applications */
} /* namespace gr */

