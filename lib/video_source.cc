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
#include <queue>

namespace gr {
  namespace applications {
    #define d_debug 0
    #define dout d_debug && std::cout
    #define FRAME_JPEG_COMPRESSION_QUALITY 95
    class video_source_impl : public video_source
    {
    	public:
    		video_source_impl(const std::string& initFile,bool show):block( "video_source",
    			gr::io_signature::make(0,0,0),
    			gr::io_signature::make(0,0,0)),
    			d_out_port(pmt::mp("bytes_out")),
                d_ctrl_port(pmt::mp("ctrl_in")),
                d_winName("Video Source")
    		{
    			message_port_register_out(d_out_port);
                message_port_register_in(d_ctrl_port);
                set_msg_handler(d_ctrl_port,boost::bind(&video_source_impl::ctrl_in,this,_1));
    			d_fname = initFile;
                if(initFile.empty() || initFile=="camera" || initFile=="cam"){
                    d_video_in.open(0); // use default camera
                    d_cam = true;
                }else{
                    d_video_in.open(d_fname);    
                    d_cam = false;
                }
                if(!d_video_in.isOpened()){
                    throw std::invalid_argument("The video source cannot open or camera failed");
                }
                d_show = show;
                cv::namedWindow(d_winName, cv::WINDOW_AUTOSIZE);
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
                d_send_ctrl.notify_all();
    			d_thread->interrupt();
    			d_thread->join();
    			return block::stop();
    		}
    		void set_resend(bool button){
    			if(button && !d_video_in.isOpened()){
    				d_video_in.open(d_fname);
                    d_send_ctrl.notify_one();
    			}
    		}
            void ctrl_in(pmt::pmt_t msg){
                pmt::pmt_t k = pmt::car(msg);
                pmt::pmt_t v = pmt::cdr(msg);
                char ctrl_msg = pmt::to_long(v);
                if(ctrl_msg == 0xff){
                    // busy
                    dout<<"video_source::lower layer busy, drop frame"<<std::endl;
                }else if(ctrl_msg == 0x0a){
                    // ready
                    dout<<"video_source::lower layer complete one frame"<<std::endl;
                    d_send_ctrl.notify_one();// send next frame
                }else{
                    // undefined
                }
            }
    	private:
    		void run()
    		{
    			cv::Mat frame;
                std::vector<uint8_t> temp;
                std::vector<int> enc_params;
                enc_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                enc_params.push_back(FRAME_JPEG_COMPRESSION_QUALITY); // compression quality
                int frate = 1000/d_video_in.get(cv::CAP_PROP_FPS);
                if(d_cam){
                    while(!d_finished){
                        if(!d_video_in.isOpened()){
                            gr::thread::scoped_lock lock(d_mutex);
                            d_send_ctrl.wait(lock);
                            lock.unlock();
                            dout<<"camera closed"<<std::endl;
                        }else{
                            d_video_in >> frame;
                            if(frame.empty()){
                                d_video_in.release();
                            }else{
                                temp.clear();
                                cv::imencode(".jpg",frame,temp,enc_params);
                                d_bpkt = pmt::make_blob(temp.data(),temp.size());
                                message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,d_bpkt));
                                if(d_show)
                                    cv::imshow(d_winName,frame);
                            }
                        }
                    }
                }else{
                    while(!d_finished){
                        if( !d_video_in.isOpened()){
                            gr::thread::scoped_lock lock(d_mutex);
                            d_send_ctrl.wait(lock);
                            lock.unlock();
                        }else{
                            d_video_in >> frame;
                            if(frame.empty()){
                                // end of video content
                                d_video_in.release(); 
                            }else{
                                temp.clear();
                                cv::imencode(".jpg",frame,temp,enc_params);
                                d_bpkt = pmt::make_blob(temp.data(),temp.size());
                                message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,d_bpkt));
                                gr::thread::scoped_lock lock(d_mutex);
                                dout<<"video_srouce::send a frame to lower layer"<<std::endl;
                                if(d_show)
                                    cv::imshow(d_winName,frame);
                                d_send_ctrl.timed_wait(lock,boost::posix_time::milliseconds(frate));
                                lock.unlock();
                            }
                        }
                    }    
                }
    		}

    		const pmt::pmt_t d_out_port;
            const pmt::pmt_t d_ctrl_port;
            pmt::pmt_t d_bpkt;
    		std::string d_fname;
    		boost::shared_ptr<gr::thread::thread> d_thread;
            gr::thread::condition_variable d_send_ctrl;
    		gr::thread::mutex d_mutex;
    		bool d_finished;
            bool d_cam;
            bool d_show;
            cv::VideoCapture d_video_in;
            const std::string d_winName;
    };

    video_source::sptr
    video_source::make(const std::string& fname, bool show)
    {
    	return gnuradio::get_initial_sptr(new video_source_impl(fname, show));
    }

  } /* namespace applications */
} /* namespace gr */

