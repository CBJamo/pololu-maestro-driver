/***************************************************************************//**
 * \file maestro_driver.cpp
 *
 * \brief ROS Implementation of the C Driver
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * Defined here is a class which wraps the basic C driver and sets up data
 * pipelines with ROS. The features include basic position control, dynamic
 * reconfigure, diagnostics, and services for estop/safe-start.
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

#include "pololu_maestro_driver/maestro_driver.hpp"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>

#include <sstream>

namespace pololu_maestro_driver
{
	MaestroDriver::MaestroDriver( const ros::NodeHandle &_nh_priv, const std::string _serial ) :
		nh_priv( _nh_priv ),
		dyn_re( dyn_re_mutex, nh_priv ),
		dyn_re_cb_type( boost::bind( &MaestroDriver::DynReCB, this, _1, _2) ),
		min_update_rate( 10.0 ),
		max_update_rate( 100.0 ),
		diag_up_freq( diagnostic_updater::FrequencyStatusParam( &min_update_rate, &max_update_rate, 0.1, 5 ) ),
		md( -1 ),
		serial( _serial )
	{
		maestro_init( );
		diag.setHardwareID( "Pololu Maestro (unknown serial)" );
		diag.add( "Pololu Maestro Status", this, &MaestroDriver::DiagCB );
		diag.add( diag_up_freq );
		diag_timer = nh_priv.createWallTimer( ros::WallDuration( 1 ), &MaestroDriver::TimerCB, this );
	}

	MaestroDriver::~MaestroDriver( )
	{
		MaestroClose( );
		maestro_exit( );
	}

	bool MaestroDriver::set_speed( float spd )
	{
		if( !MaestroStat( ) )
			return false;

		short int dir = 1;
		if( spd < 0.0 )
		{
			dir = -1;
			spd *= -1;
		}
		spd *= 3200;
		
		return MAESTRO_ERROR_OTHER; // TODO
		//int ret = maestro_set_speed( md, (int)(spd + .5), dir, 2000 );
		//if( ret >= 0 )
		//	diag_up_freq.tick( );
		//return ret;
	}

	void MaestroDriver::JointTrajectoryCB( const trajectory_msgs::JointTrajectoryPtr &msg )
	{
		if( msg->points.size( ) != 1 )
		{
			ROS_WARN_THROTTLE( 60, "Received an invalid points list size" );
			return;
		}

		for( unsigned int i = 0; i < msg->joint_names.size( ); i++ )
		{
			int chan = 0;
			if( !( std::istringstream( msg->joint_names[i] ) >> chan ) )
				continue;

			if( chan < 0 || chan >= 6 ) // TODO
			{
				ROS_WARN( "Received an invalid channel number: %d", chan );
				continue;
			}

			if( msg->points[0].velocities.size( ) == msg->joint_names.size( ) )
				maestro_set_speed( md, chan, msg->points[0].velocities[i], 2000 );
			if( msg->points[0].accelerations.size( ) == msg->joint_names.size( ) )
				maestro_set_acceleration( md, chan, msg->points[0].accelerations[i], 2000 );
			if( msg->points[0].positions.size( ) == msg->joint_names.size( ) )
				maestro_set_target( md, chan, msg->points[0].positions[i], 2000 );
		}
	}

	void MaestroDriver::DynReCB( pololu_maestro_driver::MaestroDriverConfig &cfg, const uint32_t lvl )
	{
		if( !MaestroStat( ) )
			return;

		return; // TODO
		/*struct SmcSettings set;

		if( maestro_get_parameter( md, &set, 5000 ) < 0 )
			return;

		set.neverSuspend = cfg.neverSuspend;
		set.uartResponseDelay = cfg.uartResponseDelay;
		set.useFixedBaudRate = cfg.useFixedBaudRate;
		set.disableSafeStart = cfg.disableSafeStart;
		set.fixedBaudRateRegister = cfg.fixedBaudRateRegister;
		set.speedUpdatePeriod = cfg.speedUpdatePeriod;
		set.commandTimeout = cfg.commandTimeout;
		set.serialDeviceNumber = cfg.serialDeviceNumber;
		set.crcMode = cfg.crcMode;
		set.overTempMin = cfg.overTempMin;
		set.overTempMax = cfg.overTempMax;
		set.inputMode = cfg.inputMode;
		set.pwmMode = cfg.pwmMode;
		set.pwmPeriodFactor = cfg.pwmPeriodFactor;
		set.mixingMode = cfg.mixingMode;
		set.minPulsePeriod = cfg.minPulsePeriod;
		set.maxPulsePeriod = cfg.maxPulsePeriod;
		set.rcTimeout = cfg.rcTimeout;
		set.ignorePotDisconnect = cfg.ignorePotDisconnect;
		set.tempLimitGradual = cfg.tempLimitGradual;
		set.consecGoodPulses = cfg.consecGoodPulses;
		set.motorInvert = cfg.motorInvert;
		set.speedZeroBrakeAmount = cfg.speedZeroBrakeAmount;
		set.ignoreErrLineHigh = cfg.ignoreErrLineHigh;
		set.vinMultiplierOffset = cfg.vinMultiplierOffset;
		set.lowVinShutoffTimeout = cfg.lowVinShutoffTimeout;
		set.lowVinShutoffMv = cfg.lowVinShutoffMv;
		set.serialMode = cfg.serialMode;

		if( maestro_set_parameter( md, &set, 5000 ) < 0 )
			return;*/
	}

	bool MaestroDriver::MaestroOpen( )
	{
		const char *ser = NULL;

		if( !maestro_stat( md ) )
			return true;

		if( serial.length( ) )
			ser = serial.c_str( );

		if( ( md = maestro_open( ser ) ) < 0 )
			return false;

		char mySerial[256];
		maestro_get_serial( md, mySerial );
		diag.setHardwareIDf( "Pololu Maestro %s", mySerial );

		/*if( !refresh_settings( ) )
		{
			maestro_close( md );
			md = -1;
			return false;
		}*/

		//dyn_re.setCallback( dyn_re_cb_type );
		if( !joint_traj_sub )
			joint_traj_sub = nh_priv.subscribe( "joint_trajectory", 1, &MaestroDriver::JointTrajectoryCB, this );

		return true;
	}

	void MaestroDriver::MaestroClose( )
	{
		if( joint_traj_sub )
			joint_traj_sub.shutdown( );
		dyn_re.clearCallback( );
		maestro_close( md );
		md = -1;
	}

	bool MaestroDriver::MaestroStat( )
	{
		if( maestro_stat( md ) )
		{
			if( !MaestroOpen( ) )
				return false;
		}
		return true;
	}

	void MaestroDriver::TimerCB( const ros::WallTimerEvent &e )
	{
		diag.update( );

		// This seems to do two things:
		// - restart the timer (otherwise the diagnostic_updater limits us and we hit 1/2 of the time)
		// - update the timer if the diagnostic period changed
		diag_timer.setPeriod( ros::WallDuration( diag.getPeriod( ) ) );
	}

	void MaestroDriver::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		if( !MaestroStat( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "Maestro status OK" );

		int r;

		// Firmware Version
		unsigned short int maj;
		unsigned short int min;
		if( ( r = maestro_get_fw_version( md, &maj, &min, 1000 ) ) < 0 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Failed to fetch fw_version" );
			if( r == MAESTRO_ERROR_NO_DEVICE )
				MaestroClose( );
		}
		else
		{
			float fw_ver = min;
			while( fw_ver >= 1.0 )
				fw_ver /= 10;
			fw_ver += maj;
			stat.add( "fw_version", fw_ver );
		}

		// Other Variables
		struct MaestroVariables vars;
		if( maestro_is_micro( md ) )
		{
			struct MicroMaestroVariables tmp;
			if( ( r = maestro_get_variables_micro_maestro( md, &tmp, 5000 ) ) < 0 )
			{
				stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Failed to fetch Maestro variables" );
				if( r == MAESTRO_ERROR_NO_DEVICE )
					MaestroClose( );
				return;
			}
			vars.stackPointer = tmp.stackPointer;
			vars.callStackPointer = tmp.callStackPointer;
			vars.errors = tmp.errors;
			vars.programCounter = tmp.programCounter;
			vars.scriptDone = tmp.scriptDone;
			vars.performanceFlags = MaestroPerrorNone;
		}
		else
		{
			// TODO
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Not Implemented" );
			return;
		}
		if( vars.errors )
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Maestro is reporting errors" );
		stat.add( "errors", ErrorToStr( vars.errors ) );
		stat.add( "performanceFlags", PerformanceErrorToStr( vars.performanceFlags ) );
		stat.add( "stackPointer", (unsigned int)vars.stackPointer );
		stat.add( "callStackPointer", (unsigned int)vars.callStackPointer );
		stat.add( "programCounter", (unsigned int)vars.programCounter );
		stat.add( "scriptDone", (bool)vars.scriptDone );
	}

	std::string MaestroDriver::ErrorToStr( const uint16_t error ) const
	{
		std::string str;
		bool found = false;

		if( error & MaestroErrorSerialSignal )
		{
			str += "Serial Signal";
			found = true;
		}
		if( error & MaestroErrorSerialOverrun )
		{
			if( found )
				str += ", ";
			str += "Serial Overrun";
			found = true;
		}
		if( error & MaestroErrorSerialBufferFull )
		{
			if( found )
				str += ", ";
			str += "Serial Buffer Full";
			found = true;
		}
		if( error & MaestroErrorSerialCrc )
		{
			if( found )
				str += ", ";
			str += "Serial CRC";
			found = true;
		}
		if( error & MaestroErrorSerialProtocol )
		{
			if( found )
				str += ", ";
			str += "Serial Protocol";
			found = true;
		}
		if( error & MaestroErrorSerialTimeout )
		{
			if( found )
				str += ", ";
			str += "Serial Timeout";
			found = true;
		}
		if( error & MaestroErrorScriptStack )
		{
			if( found )
				str += ", ";
			str += "Script Stack";
			found = true;
		}
		if( error & MaestroErrorScriptCallStack )
		{
			if( found )
				str += ", ";
			str += "Script Call Stack";
			found = true;
		}
		if( error & MaestroErrorScriptProgramCounter )
		{
			if( found )
				str += ", ";
			str += "Script Program Counter";
			found = true;
		}

		if( !str.length( ) )
			str = "None";
		return str;
	}

	std::string MaestroDriver::PerformanceErrorToStr( const uint8_t error ) const
	{
		std::string str;
		bool found = false;

		if( error & MaestroPerrorAdvancedUpdate )
		{
			str += "Advanced Update";
			found = true;
		}
		if( error & MaestroPerrorBasicUpdate )
		{
			if( found )
				str += ", ";
			str += "Basic Update";
			found = true;
		}
		if( error & MaestroPerrorPeriod )
		{
			if( found )
				str += ", ";
			str += "Period";
			found = true;
		}

		if( !str.length( ) )
			str = "None";
		return str;
	}

	bool MaestroDriver::refresh_settings( )
	{
		if( !MaestroStat( ) )
			return false;

		return false; // TODO
		/*struct SmcSettings set;

		if( maestro_get_parameter( md, &set, 5000 ) < 0 )
			return false;

		struct pololu_maestro_driver::MaestroDriverConfig cfg;

		cfg.neverSuspend = set.neverSuspend;
		cfg.uartResponseDelay = set.uartResponseDelay;
		cfg.useFixedBaudRate = set.useFixedBaudRate;
		cfg.disableSafeStart = set.disableSafeStart;
		cfg.fixedBaudRateRegister = set.fixedBaudRateRegister;
		cfg.speedUpdatePeriod = set.speedUpdatePeriod;
		cfg.commandTimeout = set.commandTimeout;
		cfg.serialDeviceNumber = set.serialDeviceNumber;
		cfg.crcMode = set.crcMode;
		cfg.overTempMin = set.overTempMin;
		cfg.overTempMax = set.overTempMax;
		cfg.inputMode = set.inputMode;
		cfg.pwmMode = set.pwmMode;
		cfg.pwmPeriodFactor = set.pwmPeriodFactor;
		cfg.mixingMode = set.mixingMode;
		cfg.minPulsePeriod = set.minPulsePeriod;
		cfg.maxPulsePeriod = set.maxPulsePeriod;
		cfg.rcTimeout = set.rcTimeout;
		cfg.ignorePotDisconnect = set.ignorePotDisconnect;
		cfg.tempLimitGradual = set.tempLimitGradual;
		cfg.consecGoodPulses = set.consecGoodPulses;
		cfg.motorInvert = set.motorInvert;
		cfg.speedZeroBrakeAmount = set.speedZeroBrakeAmount;
		cfg.ignoreErrLineHigh = set.ignoreErrLineHigh;
		cfg.vinMultiplierOffset = set.vinMultiplierOffset;
		cfg.lowVinShutoffTimeout = set.lowVinShutoffTimeout;
		cfg.lowVinShutoffMv = set.lowVinShutoffMv;
		cfg.serialMode = set.serialMode;*/

		boost::recursive_mutex::scoped_lock lock( dyn_re_mutex );
		//dyn_re.updateConfig( cfg );

		return true;
	}
}

