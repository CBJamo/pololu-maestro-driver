/***************************************************************************//**
 * \file maestro_driver_nodelet.hpp
 *
 * \brief Single controller nodelet (header)
 * \author Scott K Logan
 * \date October 20, 2013
 *
 * API for the ROS nodelet
 *
 * \section license License (BSD-3)
 * Copyright (c) 2013, Scott K Logan\n
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Willow Garage, Inc. nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef _maestro_driver_nodelet_hpp
#define _maestro_driver_nodelet_hpp

#include "pololu_maestro_driver/maestro_driver.hpp"

#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace pololu_maestro_driver
{
	/*!
	 * \brief Maestro Driver Nodelet
	 *
	 * \author Scott K Logan
	 *
	 * Simple implementation of the maestro_driver_nodelet::MaestroDriver class
	 */
	class MaestroDriverNodelet : public nodelet::Nodelet
	{
	public:
		/*!
		 * \brief Constructor.
		 *
		 * \author Scott K Logan
		 *
		 * The only thing this does is set maestro to NULL
		 */
		MaestroDriverNodelet( );
		/*!
		 * \brief Destructor.
		 *
		 * \author Scott K Logan
		 *
		 * The only thing this does is free maestro
		 */
		~MaestroDriverNodelet( );
	private:
		/*!
		 * \brief Called when the nodelet manager is ready to start
		 *
		 * \author Scott K Logan
		 *
		 * This function actually instantiates the maestro_driver_nodelet::MaestroDriver
		 */
		virtual void onInit( );

		/*!
		 * \brief The instance of the driver
		 */
		pololu_maestro_driver::MaestroDriver *maestro;
	};
}

#endif /* _maestro_driver_nodelet_hpp */
