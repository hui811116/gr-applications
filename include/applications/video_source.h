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


#ifndef INCLUDED_APPLICATIONS_VIDEO_SOURCE_H
#define INCLUDED_APPLICATIONS_VIDEO_SOURCE_H

#include <applications/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace applications {

    /*!
     * \brief <+description+>
     *
     */
    class APPLICATIONS_API video_source : virtual public block
    {
    public:
      typedef boost::shared_ptr<video_source> sptr;
      static sptr make(const std::string& fname, bool show);
      virtual void set_resend(bool button) =0;
    };

  } // namespace applications
} // namespace gr

#endif /* INCLUDED_APPLICATIONS_VIDEO_SOURCE_H */

