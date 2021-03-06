/***************************************************************************//**
 * \file maestro_driver_nodelet.cpp
 *
 * \brief Single controller nodelet
 * \author Scott K Logan
 * \date October 20, 2013
 *
 * This nodelet creates a single controller driver instance.
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

#include "pololu_maestro_driver_nodelet/maestro_driver_nodelet.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( pololu_maestro_driver, MaestroDriverNodelet, pololu_maestro_driver::MaestroDriverNodelet, nodelet::Nodelet )

namespace pololu_maestro_driver
{
	MaestroDriverNodelet::MaestroDriverNodelet( ) :
		maestro( NULL )
	{
	}

	MaestroDriverNodelet::~MaestroDriverNodelet( )
	{
		delete maestro;
	}

	void MaestroDriverNodelet::onInit( )
	{
		ros::NodeHandle nh_priv = getPrivateNodeHandle( );
		std::string sn;
		nh_priv.param( "maestro_serial", sn, (const std::string)"" );

		maestro = new MaestroDriver( nh_priv, sn );

		if( !maestro->MaestroOpen( ) )
			NODELET_ERROR( "MaestroDriverNodelet: Failed to open Maestro device" );

		NODELET_INFO( "MaestroDriverNodelet: Ready" );
	}
}

