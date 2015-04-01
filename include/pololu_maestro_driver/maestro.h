/***************************************************************************//**
 * \file maestro.h
 *
 * \brief Standalone C Driver for Pololu Maestro Servo Controllers (header)
 * \author Scott K Logan
 * \date October 20, 2013
 *
 * API for the standalone C driver
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

#ifndef _maestro_h
#define _maestro_h

#include <stdint.h>
#include <libusb.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief TODO
 *
 * TODO
 */
struct MaestroServoStatus
{
	/*!
	 * \brief The position in units of quarter-microseconds.
	 */
	uint16_t position;
	/*!
	 * \brief The target position in units of quarter-microseconds.
	 */
	uint16_t target;
	/*!
	 * \brief The speed limit.  Units depends on your settings.
	 */
	uint16_t speed;
	/*!
	 * \brief The acceleration limit.  Units depend on your settings.
	 */
	uint8_t acceleration;
} __attribute__((packed));

/*!
 * \brief Maestro Variables
 *
 * Represents the variables that can be read from a Micro Maestro or Mini
 * Maestro using REQUEST_GET_VARIABLES, excluding channel information, the stack,
 * and the call stack.
 */
struct MaestroVariables
{
	/*!
	 * \brief The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
	 */
	uint8_t stackPointer;

	/*!
	 * \brief The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
	 */
	uint8_t callStackPointer;

	/*!
	 * The error register.  Each bit stands for a different error (see uscError).
	 *If the bit is one, then it means that error occurred some time since the last
	 *GET_ERRORS serial command or CLEAR_ERRORS USB command.
	 */
	uint16_t errors;

	/// <summary>
	/// The address (in bytes) of the next bytecode instruction that will be executed.
	/// </summary>
	uint16_t programCounter;

	/// <summary>
	/// 0 = script is running.
	/// 1 = script is done.
	/// 2 = script will be done as soon as it executes one more instruction
	///     (used to implement step-through debugging features)
	/// </summary>
	uint8_t scriptDone;

	/// <summary>
	/// The performance flag register.  Each bit represents a different flag.
	/// If it is 1, then it means that the flag occurred some time since the last
	/// getVariables request.  This register is always 0 for the Micro Maestro
	/// because performance flags only apply to the Mini Maestros.
	/// </summary>
	uint8_t performanceFlags;
};

struct MicroMaestroVariables
{
	/// <summary>
	/// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
	/// </summary>
	uint8_t stackPointer;

	/// <summary>
	/// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
	/// </summary>
	uint8_t callStackPointer;

	/// <summary>
	/// The error register.  Each bit stands for a different error (see uscError).
	/// If the bit is one, then it means that error occurred some time since the last
	/// GET_ERRORS serial command or CLEAR_ERRORS USB command.
	/// </summary>
	uint16_t errors;

	/// <summary>
	/// The address (in bytes) of the next bytecode instruction that will be executed.
	/// </summary>
	uint16_t programCounter;

	/// <summary>Meaningless bytes to protect the program from stack underflows.</summary>
	/// <remarks>This is public to avoid mono warning CS0169.</remarks>
	int16_t buffer[3];

	/// <summary>
	/// The data stack used by the script.  The values in locations 0 through stackPointer-1
	/// are on the stack.
	/// </summary>
	int16_t stack[32];

	/// <summary>
	/// The call stack used by the script.  The addresses in locations 0 through
	/// callStackPointer-1 are on the call stack.  The next return will make the
	/// program counter go to callStack[callStackPointer-1].
	/// </summary>
	uint16_t callStack[10];

	/// <summary>
	/// 0 = script is running.
	/// 1 = script is done.
	/// 2 = script will be done as soon as it executes one more instruction
	///     (used to implement step-through debugging features)
	/// </summary>
	uint8_t scriptDone;

	/// <summary>Meaningless byte to protect the program from call stack overflows.</summary>
	/// <remarks>This is public to avoid mono warning CS0169.</remarks>
	uint8_t buffer2;

	/*!
	 * \brief TODO
	 */
	struct MaestroServoStatus servoStatus[6];

} __attribute__((packed));

/*!
 * \brief The different serial modes the Maestro can be in.  The serial mode
 *   determines how the Command Port, TTL Port, the TTL-level UART, and the
 *   command processor are connected.
 */
enum MaestroSerialMode
{
	/*!
	 * \brief On the Command Port, user can send commands and receive responses.
	 *   TTLport/UART are connected to make a USB-to-serial adapter.
	 */
	MaestroSerialModeUsbDualPort = 0,
	/*!
	 * \brief On the Command Port, user can send commands to Maestro and
	 *   simultaneously transmit bytes on the UART TX line, and user can receive
	 *   bytes from the Maestro and the UART RX line.
	 *   TTL port does not do anything.
	 */
	MaestroSerialModeUsbChained = 1,
	/*!
	 * \brief On the UART, user can send commands and receive reponses after sending
	 *   a 0xAA byte to indicate the baud rate.
	 *   Command Port receives bytes from the RX line.
	 *   TTL Port does not do anything.
	 */
	MaestroSerialModeUartDetectBaudRate = 2,
	/*!
	 * \brief On the UART, user can send commands and receive reponses at a
	 *   predetermined, fixed baud rate.
	 *   Command Port receives bytes from the RX line.
	 *   TTL Port does not do anything.
	 */
	MaestroSerialModeUartFixedBaudRate = 3,
};

/*!
 * \brief Maestro Errors
 *
 * The correspondence between errors and bits in the two-byte error register.
 * For more details about what the errors mean, see the user's guide.
 */
enum MaestroError
{
	/*!
	 * \brief None.
	 */
	MaestroErrorNone = 0,
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialSignal = ( 1 << 0 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialOverrun = ( 1 << 1 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialBufferFull = ( 1 << 2 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialCrc = ( 1 << 3 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialProtocol = ( 1 << 4 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorSerialTimeout = ( 1 << 5 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorScriptStack = ( 1 << 6 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorScriptCallStack = ( 1 << 7 ),
	/*!
	 * \brief TODO
	 */
	MaestroErrorScriptProgramCounter = ( 1 << 8 ),
};

/*!
 * \brief TODO
 *
 * TODO
 */
enum MaestroPerformanceFlag
{
	/*!
	 * \brief None.
	 */
	MaestroPerrorNone = 0,
	/*!
	 * \brief TODO
	 */
	MaestroPerrorAdvancedUpdate = ( 1 << 0 ),
	/*!
	 * \brief TODO
	 */
	MaestroPerrorBasicUpdate = ( 1 << 1 ),
	/*!
	 * \brief TODO
	 */
	MaestroPerrorPeriod = ( 1 << 2 ),
};

/*!
 * \brief Error codes for the Maestro driver.
 *
 * Based on the error codes for LibUSB. For the most part, a return value of 0
 * indicates success.
 */
enum maestro_error
{
	/*!
	 * \brief Success (no error)
	 */
	MAESTRO_SUCCESS = LIBUSB_SUCCESS,
	/*!
	 * \brief Input/output error
	 */
	MAESTRO_ERROR_IO = LIBUSB_ERROR_IO,
	/*!
	 * \brief Invalid parameter
	 */
	MAESTRO_ERROR_INVALID_PARAM = LIBUSB_ERROR_INVALID_PARAM,
	/*!
	 * \brief Access denied (insufficient permissions)
	 */
	MAESTRO_ERROR_ACCESS = LIBUSB_ERROR_ACCESS,
	/*!
	 * \brief No such device (it may have been disconnected)
	 */
	MAESTRO_ERROR_NO_DEVICE = LIBUSB_ERROR_NO_DEVICE,
	/*!
	 * \brief Entity not found
	 */
	MAESTRO_ERROR_NOT_FOUND = LIBUSB_ERROR_NOT_FOUND,
	/*!
	 * \brief Resource busy
	 */
	MAESTRO_ERROR_BUSY = LIBUSB_ERROR_BUSY,
	/*!
	 * \brief Operation timed out
	 */
	MAESTRO_ERROR_TIMEOUT = LIBUSB_ERROR_TIMEOUT,
	/*!
	 * \brief Overflow
	 */
	MAESTRO_ERROR_OVERFLOW = LIBUSB_ERROR_OVERFLOW,
	/*!
	 * \brief Pipe error
	 */
	MAESTRO_ERROR_PIPE = LIBUSB_ERROR_PIPE,
	/*!
	 * \brief System call interrupted (perhaps due to signal)
	 */
	MAESTRO_ERROR_INTERRUPTED = LIBUSB_ERROR_INTERRUPTED,
	/*!
	 * \brief Insufficient memory
	 */
	MAESTRO_ERROR_NO_MEM = LIBUSB_ERROR_NO_MEM,
	/*!
	 * \brief Operation not supported or unimplemented on this platform
	 */
	MAESTRO_ERROR_NOT_SUPPORTED = LIBUSB_ERROR_NOT_SUPPORTED,
	/*!
	 * \brief Other error
	 */
	MAESTRO_ERROR_OTHER = LIBUSB_ERROR_OTHER,
};

/*!
 * \brief Maestro Driver initialization routine.
 *
 * \author Scott K Logan
 *
 * Allocates memory and initializes LibUSB. This function should be called
 * before any of the other driver functions are used.
 *
 * \returns ::maestro_error code
 */
int maestro_init( );

/*!
 * \brief Open a new connection to a Maestro device.
 *
 * \author Scott K Logan
 *
 * Searches through the connected USB devices for a device that matches the
 * Pololu vendor ID and one of the Maestro product IDs. If a serial number is
 * specified, only a matching serial number will result in a proper device
 * open.
 *
 * \param serial C-string of the serial number of the device to connect to. If
 *   the string is empty, the first available device is chosen.
 *
 * \returns New Maestro handle to be use for future references to that device
 * (which is greater than -1), otherwise an ::maestro_error code.
 */
int maestro_open( const char *serial );

/*!
 * \brief Retrieves the firmware version from the device.
 *
 * \author Scott K Logan
 *
 * Simple routine for fetching the firmware version from the device. Note that
 * these values will never change during this power cycle.
 *
 * \param md Maestro device handle
 * \param major Non-null pointer to a short int. If the function returns
 *   ::MAESTRO_SUCCESS, this will be populated with the major version number.
 * \param minor Non-null pointer to a short int. If the function returns
 *   ::MAESTRO_SUCCESS, this will be populated with the minor version number.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
int maestro_get_fw_version( const int md, unsigned short int *major,
	unsigned short int *minor, const unsigned int to );

/*!
 * \brief Retrieves the serial number from USB.
 *
 * \author Scott K Logan
 *
 * This routine fetches the serial number from the system, which was set when
 * the device was connected. Note that this value will never change during this
 * power cycle.
 *
 * \param md Maestro device handle
 * \param sn Non-null pointer to a character array of size >= 256
 *
 * \returns ::maestro_error code
 */
int maestro_get_serial( const int md, char *sn );

/*!
 * \brief Determines if the device is a Micro Maestro.
 *
 * \author Scott K Logan
 *
 * The Micro Maestro should be treated differently from the other Maestro
 * devices, and this routine is for determining which type of device is
 * connected to the driver.
 *
 * \param md Maestro device handle
 *
 * \retval 1 Device is a Micro Maestro
 * \retval 0 Device is not a Micro Maestro
 */
int maestro_is_micro( const int md );

/*!
 * \brief TODO
 *
 * \author Scott K Logan
 *
 * TODO
 *
 * \param md Maestro device handle
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
int maestro_reinitialize( const int md, const unsigned int to );

/*!
 * \brief Sets the speed of the motor.
 *
 * \author Scott K Logan
 *
 * This is probably called more often than any other function. It changes the
 * motor controller's duty cycle to change the speed of the motor.
 *
 * \param md Maestro device handle
 * \param val Value to set speed to. For forward and backward, range is from 0
 *   to 3200. For braking, range is from 0 to 32.
 * \param dir Direction of travel, 0 is for braking, 1 is foreward, -1 is
 *   backward.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
/*int maestro_set_speed( const int md, const unsigned int val,
	const short int dir, const unsigned int to );*/
int maestro_set_target( const int md, const uint8_t servo, uint16_t val,
	const unsigned int to );
int maestro_set_speed( const int md, const uint8_t servo, uint16_t val,
	const unsigned int to );
int maestro_set_acceleration( const int md, const uint8_t servo, uint16_t val,
	const unsigned int to );

/*!
 * \brief Gets the variables from the device. TODO
 *
 * \author Scott K Logan
 *
 * Fetches the non-ROM variables from the device, including many diagnostic
 * values.
 *
 * \param md Maestro device handle
 * \param vars Non-null pointer to an ::SmcVariables struct. If the function
 *   returns ::MAESTRO_SUCCESS, this will be populated with the variables from the
 *   device.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
int maestro_get_variables_micro_maestro( const int md,
	struct MicroMaestroVariables *vars, const unsigned int to );

/*!
 * \brief Gets the settings from the device. TODO
 *
 * \author Scott K Logan
 *
 * Fetches the ROM values from the device, including many configurable
 * parameters.
 *
 * \param md Maestro device handle
 * \param set Non-null pointer to an ::SmcSettings struct. If the function
 *   returns ::MAESTRO_SUCCESS, this will be populated with the current settings of
 *   the device.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
int maestro_get_parameter( const int md, void *set,
	const unsigned int to );

/*!
 * \brief Sends new settings to the device. TODO
 *
 * \author Scott K Logan
 *
 * Stores the given settings in the flash ROM of the device.
 *
 * \param md Maestro device handle
 * \param set Non-null pointer to an ::SmcSettings struct. The data at this
 *   address will be sent to the device and written to the flash.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::maestro_error code
 */
int maestro_set_parameter( const int md, void *set,
	const unsigned int to );

/*!
 * \brief Checks the internal status of a Maestro device handle.
 *
 * \author Scott K Logan
 *
 * Queries the internal workings of the communications link in the driver.
 *
 * \param md Maestro device handle
 *
 * \returns ::maestro_error code
 */
int maestro_stat( const int md );

/*!
 * \brief Closes communication with a Maestro device, and restores kernel control
 *   if necessary.
 *
 * \author Scott K Logan
 *
 * After calling this, all refrences to the device should be cleared, as the
 * handle is no longer valid.
 *
 * \param md Maestro device handle
 */
void maestro_close( const int md );

/*!
 * \brief Closes all remaining handles and frees driver memory.
 *
 * \author Scott K Logan
 *
 * This should be called at program termination to free memory. After calling
 * this, all references to Maestro device handles should be cleared, as they
 * are no longer valid. No other functions should be called until ::maestro_init
 * is called again.
 */
void maestro_exit( );

#ifdef __cplusplus
}
#endif

#endif /* _maestro_h */
