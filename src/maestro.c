/***************************************************************************//**
 * \file maestro.c
 *
 * \brief Standalone C Driver for Pololu Maestro Servo Controllers
 * \author Scott K Logan
 * \date October 20, 2013
 *
 * This is a standolone C driver for the Pololu Maestro family of servo
 * controllers. It uses LibUSB to interface with the system's USB drivers, and
 * is interfaced with similarly to files, in which a device is opened and is
 * referenced with an integer handle.
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

#include "pololu_maestro_driver/maestro.h"

#include <libusb.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h> // TODO

/*!
 * \brief Maximum number of simultaneously open device handles.
 */
#define MAX_HANDLES 256

/*!
 * \brief Number of product IDs this driver can handle.
 */
#define NUM_PRODUCTS 5

/*!
 * \brief Vendor ID of Pololu.
 */
static const uint16_t idVendorTarget = 0x1FFB;

/*!
 * \brief Product IDs of the 4 Maestro models.
 */
static const uint16_t idProductTargetArr[NUM_PRODUCTS] = { 0x89, 0x8A, 0x8B, 0x8C };

/*!
 * \brief Servo Counts of the 4 Maestro models.
 */
static const uint8_t servoCountArr[NUM_PRODUCTS] = { 6, 12, 18, 24 };

/*!
 * \brief Internal codes for USB communication with the Maestros.
 *
 * Used to USB setup control transfers with the Maestro.
 */
enum MaestroRequest
{
	/*!
	 * \brief Used to read a parameter from the device.
	 */
	GetParameter = 0x81,
	/*!
	 * \brief Used to set a parameter on the device.
	 *
	 * These settings will be written to flash. This operation usually takes about
	 * 26ms. TODO - is this still true on the Maestro
	 */
	SetParameter = 0x82,
	/*!
	 * \brief Used Gets the current state of the device.
	 */
	GetVariables = 0x83,
	/*!
	 * \brief TODO
	 */
	SetServoVariable = 0x84,
	/*!
	 * \brief TODO
	 */
	SetTarget = 0x85,
	/*!
	 * \brief TODO
	 */
	ClearErrors = 0x86,
	/*!
	 * \brief TODO
	 */
	GetServoSettings = 0x87,
	/*!
	 * \brief TODO
	 *
	 * Used only on Mini Maestro
	 */
	GetStack = 0x88,
	/*!
	 * \brief TODO
	 *
	 * Used only on Mini Maestro
	 */
	GetCallStack = 0x89,
	/*!
	 * \brief TODO
	 */
	SetPwm = 0x8A,
	/*!
	 * \brief TODO
	 */
	Reinitialize = 0x90,
	/*!
	 * \brief TODO
	 */
	EraseScript = 0xA0,
	/*!
	 * \brief TODO
	 */
	WriteScript = 0xA1,
	/*!
	 * \brief TODO
	 *
	 * value.low.b is 0 for go, 1 for stop, 2 for single-step
	 */
	SetScriptDone = 0xA2,
	/*!
	 * \brief TODO
	 */
	RestartScriptAtSubroutine = 0xA3,
	/*!
	 * \brief TODO
	 */
	RestartScriptAtSubroutineWithParameter = 0xA4,
	/*!
	 * \brief TODO
	 */
	RestartScript = 0xA5,
	/*!
	 * \brief Used to cause the device to disconnect and enter bootloader mode.
	 *
	 * In bootloader mode, you can upgrade the firmware on the device. After using
	 * this, you should disconnect from the device and delete any references to it
	 * because it is no longer usable.
	 */
	StartBootloader = 0xFF
};

/*!
 * \brief Private structure for handling Maestro communication.
 *
 * Un-changing variables are stored here, as well as the LibUSB device handle.
 */
struct maestro_priv
{
	/*!
	 * \brief LibUSB device handle.
	 */
	libusb_device_handle *dev;
	/*!
	 * \brief Kernel driver status.
	 *
	 * Set to 1 if the kernel driver was active and needed to be unloaded before
	 * the driver could assume exclusive control of the device. If this is 1,
	 * kernel driver control is resumed when this driver is unloaded.
	 */
	short unsigned int kdriver_active;
	/*!
	 * \brief Major version of the firmware on the device.
	 */
	unsigned short int *ver_maj;
	/*!
	 * \brief Minor version of the firmware on the device.
	 */
	unsigned short int *ver_min;
	/*!
	 * \brief Serial number of the USB device.
	 */
	char *serial;
	/*!
	 * \brief Number of servos on the device.
	 */
	unsigned char servo_count;
};

/*!
 * \brief List of communication handles.
 */
static struct maestro_priv * maestro_list[MAX_HANDLES] = { NULL };

/*!
 * \brief LibUSB context for USB communication.
 */
static libusb_context *ctx = NULL;

/*!
 * \brief Grabs the next available device handle slot.
 *
 * Iterates through the ::MAX_HANDLES slots for the lowest available index.
 *
 * \returns Open slot index between 0 and ::MAX_HANDLES
 * \retval -1 No available slots
 */
static int next_available_handle( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( !maestro_list[i] )
			return i;
	}
	return -1;
}

int maestro_init( )
{
	int r = LIBUSB_SUCCESS;

	if( ( r = libusb_init( &ctx ) ) )
		return r;

	#if DEBUG
	libusb_set_debug( ctx, 3 );
	#endif

	return r;
}

int maestro_open( const char *serial )
{
	libusb_device **devs;
	ssize_t c;
	ssize_t i;
	int r;

	c = libusb_get_device_list( ctx, &devs );

	if( c < 0 )
		return c;

	for( i = 0; i < c; i++ )
	{
		struct libusb_device_descriptor desc;

		r = libusb_get_device_descriptor( devs[i], &desc );
		if( r || desc.idVendor != idVendorTarget )
			continue;

		unsigned short int j;
		for( j = 0; j < NUM_PRODUCTS; j++ )
		{
			if( desc.idProduct == idProductTargetArr[j] )
			{
				libusb_device_handle *dev_handle;
				unsigned short int kernel_driver_active = 0;

				r = libusb_open( devs[i], &dev_handle );
				if( r )
					break;

				char mySerial[256];

				r = libusb_get_string_descriptor_ascii( dev_handle, desc.iSerialNumber, (unsigned char *)mySerial, 256 );
				if( serial && ( r < 0 || strcmp( mySerial, serial ) ) )
				{
					libusb_close( dev_handle );
					break;
				}

				r = libusb_kernel_driver_active( dev_handle, 0 );
				if( r < 0 )
				{
					libusb_close( dev_handle );
					break;
				}
				else if( r == 1 )
				{
					r = libusb_detach_kernel_driver( dev_handle, 0 );
					if( r )
					{
						libusb_close( dev_handle );
						break;
					}
					kernel_driver_active = 1;
				}

				r = libusb_claim_interface( dev_handle, 0 );
				if( r )
				{
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					break;
				}

				int mydev = next_available_handle( );

				if( mydev < 0 )
				{
					libusb_release_interface( dev_handle, 0 );
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					return LIBUSB_ERROR_NO_MEM;
				}

				maestro_list[mydev] = malloc( sizeof( struct maestro_priv ) );
				if( !maestro_list[mydev] )
				{
					libusb_release_interface( dev_handle, 0 );
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					return LIBUSB_ERROR_NO_MEM;
				}
				memset( maestro_list[mydev], 0, sizeof( struct maestro_priv ) );
				maestro_list[mydev]->dev = dev_handle;
				maestro_list[mydev]->kdriver_active = kernel_driver_active;
				maestro_list[mydev]->serial = malloc( sizeof( char ) * ( strlen( mySerial ) + 1 ) );
				strcpy( maestro_list[mydev]->serial, mySerial );
				maestro_list[mydev]->servo_count = servoCountArr[j];

				libusb_free_device_list( devs, 1 );
				return mydev;
			}
		}
	}

	libusb_free_device_list( devs, 1 );
	return LIBUSB_ERROR_NOT_FOUND;
}

int maestro_get_fw_version( const int md, unsigned short int *major, unsigned short int *minor, const unsigned int to )
{
	if( maestro_list[md]->ver_maj && maestro_list[md]->ver_min )
	{
		*major = *maestro_list[md]->ver_maj;
		*minor = *maestro_list[md]->ver_min;
		return LIBUSB_SUCCESS;
	}

	int r;
	char buff[14];

	if( ( r = libusb_control_transfer( maestro_list[md]->dev, 0x80, 6, 0x0100, 0, (unsigned char *)buff, sizeof( buff ), to ) ) != sizeof( buff ) )
		return r;

	*minor = ( buff[12] & 0xF ) + ( ( buff[12] >> 4 & 0xF ) * 10 );
	*major = ( buff[13] & 0xF ) + ( ( buff[13] >> 4 & 0xF ) * 10 );

	maestro_list[md]->ver_maj = malloc( sizeof( unsigned short int ) );
	maestro_list[md]->ver_min = malloc( sizeof( unsigned short int ) );
	if( !maestro_list[md]->ver_maj || !maestro_list[md]->ver_min )
		return LIBUSB_ERROR_NO_MEM;

	*maestro_list[md]->ver_maj = *major;
	*maestro_list[md]->ver_min = *minor;

	return r;
}

int maestro_get_serial( const int md, char *sn )
{
	strcpy( sn, maestro_list[md]->serial );
	return strlen( maestro_list[md]->serial );
}

int maestro_is_micro( const int md )
{
	return ( maestro_list[md]->servo_count == servoCountArr[0] ) ? 1 : 0;
}

int maestro_reinitialize( const int md, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0x40, Reinitialize, 0, 0, NULL, 0, to );
}

int maestro_set_target( const int md, const uint8_t servo, uint16_t val, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0x40, SetTarget, val, servo, NULL, 0, to );
}

int maestro_set_speed( const int md, const uint8_t servo, uint16_t val, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0x40, SetServoVariable, val, servo, NULL, 0, to );
}

int maestro_set_acceleration( const int md, const uint8_t servo, uint16_t val, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0x40, SetServoVariable, val, (uint8_t)(servo | 0x80), NULL, 0, to );
}

int maestro_get_variables_micro_maestro( const int md, struct MicroMaestroVariables *vars, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0xC0, GetVariables, 0, 0, (unsigned char *)vars, sizeof( struct MicroMaestroVariables ), to );
}

int maestro_get_parameter( const int md, void *set, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0xC0, GetParameter, 0, 0, (unsigned char *)set, sizeof( void * ), to );
}

int maestro_set_parameter( const int md, void *set, const unsigned int to )
{
	return libusb_control_transfer( maestro_list[md]->dev, 0x40, SetParameter, 0, 0, (unsigned char *)set, sizeof( void * ), to );
}

int maestro_stat( const int md )
{
	if( md < 0 || !maestro_list[md] )
		return LIBUSB_ERROR_NOT_FOUND;

	/// \todo It would be nice to do some libusb checks here...

	return LIBUSB_SUCCESS;
}

void maestro_close( const int md )
{
		if( md < 0 || !maestro_list[md] )
			return;

		libusb_release_interface( maestro_list[md]->dev, 0 );
		if( maestro_list[md]->kdriver_active )
			libusb_attach_kernel_driver( maestro_list[md]->dev, 0 );
		libusb_close( maestro_list[md]->dev );

		free( maestro_list[md]->ver_maj );
		free( maestro_list[md]->ver_min );
		free( maestro_list[md]->serial );

		free( maestro_list[md] );
		maestro_list[md] = NULL;
}

void maestro_exit( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( maestro_list[i] )
		{
			maestro_close( i );
		}
	}

	if( ctx )
		libusb_exit( ctx );
	ctx = NULL;
}
