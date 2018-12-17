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
    #define FRAME_JPEG_COMPRESSION_QUALITY 90
    class video_source_impl : public video_source
    {
    	public:
    		video_source_impl(const std::string& initFile, int fps):block( "video_source",
    			gr::io_signature::make(0,0,0),
    			gr::io_signature::make(0,0,0)),
    			d_out_port(pmt::mp("bytes_out"))

    		{
    			message_port_register_out(d_out_port);
    			d_fname = initFile;
                d_frame_duration = 1000/fps;
                d_video_in.open(d_fname);
    		}
    		~video_source_impl()
    		{

    		}
    		bool start()
    		{
    			d_finished = false;
    			d_thread = boost::shared_ptr<gr::thread::thread>(
    				new gr::thread::thread(boost::bind(&video_source_impl::run,this)));
                d_display = boost::shared_ptr<gr::thread::thread>(
                    new gr::thread::thread(boost::bind(&video_source_impl::play,this)));;
    			return block::start();
    		}

    		bool stop()
    		{
    			d_finished = true;
                d_play_video.notify_one();
    			d_thread->interrupt();
                d_display->interrupt();
    			d_thread->join();
                d_display->join();
    			return block::stop();
    		}
    		void set_fname(const std::string& filename){
    			gr::thread::scoped_lock guard(d_mutex);
    			if(!filename.empty() && d_fname!=filename){
    				d_fname = filename;
                    d_video_in.open(d_fname);
    			}
    		}
    		std::string get_fname() const{
    			return d_fname;
    		}
    		void set_resend(bool button){
    			if(button && !d_video_in.isOpened()){
    				d_video_in.open(d_fname);
                    d_send_ctrl.notify_one();
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
                            d_playlist.push(d_fname);
                            d_play_video.notify_one(); // call play
    				    }else{
                            temp.clear();
                            cv::imencode(".jpg",frame,temp,enc_params);
                            d_bpkt = pmt::make_blob(temp.data(),temp.size());
                            message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,d_bpkt));    
                        }
    				}
    			}
                
    		}
            void play()
            {
                cv::VideoCapture video_in;
                cv::Mat frame;
                std::string win_name = "Video Source";
                cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
                while(!d_finished){
                    if(d_playlist.empty()){
                        gr::thread::scoped_lock lock(d_mutex);
                        d_play_video.wait(lock);
                        lock.unlock();    
                    }else{
                        video_in.open(d_playlist.front());
                        if(video_in.isOpened()){
                            video_in >> frame;
                            if(frame.empty()){
                                video_in.release();
                            }else{
                                cv::imshow(win_name,frame);
                                gr::thread::scoped_lock flow(d_mutex);
                                d_flow_ctrl.timed_wait(flow, boost::posix_time::milliseconds(d_frame_duration));
                                flow.unlock();
                            }
                        }
                        d_playlist.pop();
                    }
                }
            }

    		const pmt::pmt_t d_out_port;
            pmt::pmt_t d_bpkt;
    		std::string d_fname;
    		boost::shared_ptr<gr::thread::thread> d_thread;
            boost::shared_ptr<gr::thread::thread> d_display;
    		gr::thread::condition_variable d_flow_ctrl;
            gr::thread::condition_variable d_send_ctrl;
    		gr::thread::condition_variable d_play_video;
    		gr::thread::mutex d_mutex;
    		bool d_finished;
            int d_frame_duration;
            std::queue<std::string> d_playlist;
            cv::VideoCapture d_video_in;
    };

    video_source::sptr
    video_source::make(const std::string& fname, int fps)
    {
    	return gnuradio::get_initial_sptr(new video_source_impl(fname,fps));
    }

  } /* namespace applications */
} /* namespace gr */

