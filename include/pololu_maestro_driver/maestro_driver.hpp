/***************************************************************************//**
 * \file maestro_driver.hpp
 *
 * \brief ROS Implementation of the C Driver (header)
 * \author Scott K Logan
 * \date October 20, 2013
 *
 * API for the ROS driver
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

#ifndef _maestro_driver_hpp
#define _maestro_driver_hpp

#include "pololu_maestro_driver/maestro.h"
#include "pololu_maestro_driver/MaestroDriverConfig.h"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

/*!
 * \brief Used for the ROS Maestro Driver.
 *
 * All elements of the ROS implementation including the wrapped driver and the
 * dynamic reconfigure elements fall within this namespace.
 */
namespace pololu_maestro_driver
{
	/*!
	 * \brief ROS-wrapped implementation of the Maestro driver.
	 *
	 * \author Scott K Logan
	 *
	 * This class interfaces with a single Maestro device. It automatically subscribes
	 * to the control feed and creates instances of the dynamic reconfigure server
	 * and the diagnostic updater.
	 */
	class MaestroDriver
	{
	public:
		/*!
		 * \brief Constructor.
		 *
		 * \author Scott K Logan
		 *
		 * This constructor initializes the standalone C driver and the various
		 * components of the ROS interface from that driver.
		 *
		 * \param _nh_priv Private node handle to use for all ROS communication
		 *   within for this device. There should never be two drivers sharing the
		 *   same private node handle. If one is not provided, it will be
		 *   automatically created.
		 * \param _serial Serial number of the target device. If no serial is given
		 *   or an empty string is given, the first available device will be used.
		 *   Please note that if there are multiple devices connected, this may not
		 *   be the same device every time!
		 */
		MaestroDriver( const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ),
			const std::string _serial = "" );
		/*!
		 * \brief Destructor.
		 *
		 * \author Scott K Logan
		 *
		 * Closes the interface to the driver, unsubscribes, unadvertises, and frees
		 * the standalone driver private data.
		 */
		~MaestroDriver( );

		/*!
		 * \brief Opens the interface with the device.
		 *
		 * \author Scott K Logan
		 *
		 * Calls the standalone driver routines for opening the USB communication
		 * pipeline with the device. Each time this is called, the settings displayed
		 * by dynamic reconfigure are updated according to the device settings. ROS
		 * subscriptions and advertisements are made at this time.
		 *
		 * \returns True if device was successfully opened.
		 */
		bool MaestroOpen( );
		/*!
		 * \brief Closes the interface with the device.
		 *
		 * \author Scott K Logan
		 *
		 * Closes the non-diagnostic interfaces with ROS and closes the interface
		 * with the driver and device.
		 */
		void MaestroClose( );
		/*!
		 * \brief Communication status of the device.
		 *
		 * \author Scott K Logan
		 *
		 * Tests for proper communication with the device driver.
		 *
		 * \returns False if there is a communications problem.
		 */
		bool MaestroStat( );
		/*!
		 * \brief Sets the speed of the motor.
		 *
		 * \author Scott K Logan
		 *
		 * Converts the generic float-style message to the driver-friendly values.
		 * If the device is currently in a disconnected state, this function will
		 * first attempt to connect to the device.
		 *
		 * \param spd Target speed for the device, between -1 and 1.
		 *
		 * \returns True on successful speed change.
		 */
		bool set_speed( float spd );
	private:
		/*!
		 * \brief ROS message callback for setting the joint trajectories
		 *
		 * \author Scott K Logan
		 *
		 * Calls the native MaestroDriver::set_target, MaestroDriver::set_speed and
		 * MaestroDriver::set_acceleration functions for the given channels
		 *
		 * Each channel that is specified as part of the message will be updated. Use the
		 * joint_names field to specify a number indicating the channel number.
		 *
		 * \param msg JointTrajectory message.
		 */
		void JointTrajectoryCB( const trajectory_msgs::JointTrajectoryPtr &msg );
		/*!
		 * \brief Dynamic Reconfigure Change Callback.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed this callback is called to
		 * interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void DynReCB( pololu_maestro_driver::MaestroDriverConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Callback for the Diagnostic Updater Timer
		 *
		 * \author Scott K Logan
		 *
		 * This timer is used to call the diagnostic update function at the configured
		 * interval. This function simply calls that function.
		 *
		 * \param e Timer event (not used)
		 */
		void TimerCB( const ros::WallTimerEvent &e );
		/*!
		 * \brief Diagnostic update callback
		 *
		 * \author Scott K Logan
		 *
		 * Whenever the diagnostic_updater deems it necessary to update the values
		 * therein, this callback is called to fetch the values from the device.
		 *
		 * \param[out] stat Structure in which to store the values for
		 * diagnostic_updater to report
		 */
		void DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat );
		/*!
		 * \brief Update the values in Dynamic Reconfigure from the device.
		 *
		 * \author Scott K Logan
		 *
		 * Queries the device for settings and updates the dynamic reconfigure
		 * values appropriately. This is called automatically every time the device
		 * is connected.
		 *
		 * \returns True if values were successfully updated.
		 */
		bool refresh_settings( );
		/*!
		 * \brief Converts an error code to a comma separated string.
		 *
		 * \author Scott K Logan
		 *
		 * \param error Error value to convert
		 *
		 * \returns C++ string containing the error descriptions
		 */
		std::string ErrorToStr( const uint16_t error ) const;
		/*!
		 * \brief Converts a performance error to a comma separated string.
		 *
		 * \author Scott K Logan
		 *
		 * \param error Performance error value to convert
		 *
		 * \returns C++ string containing the performance error descriptions
		 */
		std::string PerformanceErrorToStr( const uint8_t error ) const;

		/*!
		 * \brief Mutex used for dynamic reconfigure
		 */
		boost::recursive_mutex dyn_re_mutex;

		/*!
		 * \brief Private NodeHanlde used to interface with ROS
		 */
		ros::NodeHandle nh_priv;
		/*!
		 * \brief Subscription to speed control data
		 */
		ros::Subscriber joint_traj_sub;
		/*!
		 * \brief Dynamic reconfigure server
		 */
		dynamic_reconfigure::Server<pololu_maestro_driver::MaestroDriverConfig> dyn_re;
		/*!
		 * \brief Dynamic reconfigure callback handle
		 */
		dynamic_reconfigure::Server<pololu_maestro_driver::MaestroDriverConfig>::CallbackType dyn_re_cb_type;
		/*!
		 * \brief Timer for calling the diagnostic update function
		 */
		ros::WallTimer diag_timer;
		/*!
		 * \brief Diagnostic updater
		 */
		diagnostic_updater::Updater diag;
		/*!
		 * \brief Normal acceptable update rate minimum
		 */
		double min_update_rate;
		/*!
		 * \brief Normal acceptable update rate maximum
		 */
		double max_update_rate;
		/*!
		 * \brief Diagnostic rate for speed update
		 */
		diagnostic_updater::FrequencyStatus diag_up_freq;

		/*!
		 * \brief Maestro device handle to use when interfacing with the standalone
		 *   driver.
		 */
		int md;
		/*!
		 * \brief Serial number of the device with which we should be interfacing, or
		 *   a blank string for the first available device.
		 */
		std::string serial;
	};
}

#endif /* _maestro_driver_hpp */
