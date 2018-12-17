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
#include <applications/video_sink.h>
#include <gnuradio/block_detail.h>
#include <opencv2/opencv.hpp>
#include <queue>

namespace gr {
  namespace applications {
  	#define VIDEO_BUFFER_DELAY_MS 3000
	class video_sink_impl: public video_sink
	{
	public:
		video_sink_impl(int fps) : block("video_sink",
			gr::io_signature::make(0,0,0),
			gr::io_signature::make(0,0,0)),
			d_frame_port(pmt::mp("frm_in")),
			d_win_name("Video Sink")
		{
			message_port_register_in(d_frame_port);
			set_msg_handler(d_frame_port,boost::bind(&video_sink_impl::msg_in,this,_1));
			d_delay = 1000/fps; // milliseconds pause
		}
		~video_sink_impl()
		{

		}
		void msg_in(pmt::pmt_t msg)
		{
			pmt::pmt_t k = pmt::car(msg);
			pmt::pmt_t v = pmt::cdr(msg);
			size_t io(0);
			const uint8_t* uvec = pmt::u8vector_elements(v,io);
			cv::Mat frame = cv::imdecode(cv::_InputArray(uvec,io),cv::IMREAD_ANYCOLOR);
			if(!frame.data){
				// failed 
			}else{
				//gr::thread::scoped_lock guard(d_mutex);
				d_frameQ.push(frame);
				//guard.unlock();
			}
		}
		bool start()
		{
			d_finished = false;
			d_thread = boost::shared_ptr<gr::thread::thread>(
				new gr::thread::thread(boost::bind(&video_sink_impl::run,this)));
			return block::start();
		}
		bool stop()
		{
			d_finished = true;
			d_thread->interrupt();
			d_thread->join();
			return block::stop();
		}
	private:
		void run()
		{
			cv::namedWindow(d_win_name,cv::WINDOW_AUTOSIZE);
			while(!d_finished){
				while(!d_frameQ.empty() && !d_finished){
					gr::thread::scoped_lock lock(d_mutex);
					cv::imshow(d_win_name,d_frameQ.front());
					d_frameQ.pop();
					lock.unlock();
					boost::this_thread::sleep(boost::posix_time::milliseconds(d_delay));
				}
				if(d_finished)
					return;
				boost::this_thread::sleep(boost::posix_time::milliseconds(VIDEO_BUFFER_DELAY_MS));
			}
		}
		pmt::pmt_t d_frame_port;
		bool d_finished;
		boost::shared_ptr<gr::thread::thread> d_thread;
		gr::thread::mutex d_mutex;
		//gr::thread::condition_variable d_frame_ctrl;
		std::string d_win_name;
		std::queue<cv::Mat> d_frameQ;
		int d_delay;
	};

	video_sink::sptr
	video_sink::make(int fps)
	{
		return gnuradio::get_initial_sptr(new video_sink_impl(fps));
	}

  } /* namespace applications */
} /* namespace gr */

