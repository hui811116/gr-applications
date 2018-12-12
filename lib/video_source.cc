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
#include <applications/video_source.h>
#include <gnuradio/block_detail.h>
#include <opencv2/opencv.hpp>
#include <sstream>

namespace gr {
  namespace applications {

    class video_source_impl : public video_source
    {
    	public:
    		video_source_impl(const std::string& initFile):block( "video_source",
    			gr::io_signature::make(0,0,0),
    			gr::io_signature::make(0,0,0)),
    			d_out_port(pmt::mp("bytes_out"))

    		{
    			message_port_register_out(d_out_port);
    			d_fname = initFile;
    			d_send_cnt = 0;
    		}
    		~video_source_impl()
    		{

    		}
    		bool start()
    		{
    			d_finished = false;
    			d_thread = boost::shared_ptr<gr::thread::thread>(
    				new gr::thread::thread(boost::bind(&video_source_impl::run,this)));
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
    			cv::VideoCapture video_in;
    			cv::Mat frame;
    			int frame_num;
    			while(!d_finished){
    				video_in.open(d_fname);
    				if( !video_in.isOpened()){
    					// no data
    				}else{
    					cv::namedWindow("Testing Video", cv::WINDOW_AUTOSIZE);
    					frame_num = 0;
    					while(!d_finished){
    						video_in >> frame;
    						if(frame.empty()){
    							// end of video content
    							break;
    						}
    						frame_num++;
    						cv::imshow("Testing Video", frame);
    						//int delay = 0; // DEBUG
    						//char c = (char)cv::waitKey(delay);
    						gr::thread::scoped_lock ffrate(d_mutex);
    						d_frame_rate.timed_wait(ffrate,boost::posix_time::milliseconds(25));
    						ffrate.unlock();
    						//if(c==27) break;
    					}
    					d_send_cnt++; // for debugging purpose
    					gr::thread::scoped_lock lock(d_mutex);
    					d_flow_ctrl.wait(lock);
    					lock.unlock();
    				}
    			}
    		}
    		const pmt::pmt_t d_out_port;
    		std::string d_fname;
    		boost::shared_ptr<gr::thread::thread> d_thread;
    		gr::thread::condition_variable d_flow_ctrl;
    		gr::thread::condition_variable d_frame_rate;
    		gr::thread::mutex d_mutex;
    		bool d_finished;
    		int d_send_cnt;

    };

    video_source::sptr
    video_source::make(const std::string& fname)
    {
    	return gnuradio::get_initial_sptr(new video_source_impl(fname));
    }

  } /* namespace applications */
} /* namespace gr */

