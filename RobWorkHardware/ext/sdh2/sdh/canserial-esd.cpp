//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_canserial_esd_cpp_general General file information

  \author   Dirk Osswald
  \date     2007-02-20

  \brief
  Implementation of class #SDH::cCANSerial_ESD, a class to access an ESD CAN interface  on cygwin/linux and Visual Studio.

  \section sdhlibrary_cpp_canserial_esd_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_canserial_esd_cpp_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2009-12-01 11:41:18 +0100 (Di, 01 Dez 2009) $
  \par SVN file revision:
  $Id: canserial-esd.cpp 5000 2009-12-01 10:41:18Z Osswald2 $

  \subsection sdhlibrary_cpp_canserial_esd_cpp_changelog Changelog of this file:
  \include canserial-esd.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
//#include <termios.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif
//#include <errno.h>
//#include <string.h>
//#include <sys/select.h>
//#include <sys/ioctl.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "canserial-esd.h"
#include "simpletime.h"
#include "util.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;


cCANSerial_ESD::cCANSerial_ESD( int _net, unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write )
    throw (cCANSerial_ESDException*)
{
    if ( _timeout < 0.0 )
        throw new cCANSerial_ESDException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

    ntcan_handle = (NTCAN_HANDLE) NTCAN_INVALID_HANDLE;
    net = _net;
    baudrate = _baudrate;
    SetTimeout( _timeout );
    id_read = _id_read;
    id_write = _id_write;

    ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_ESD::cCANSerial_ESD( NTCAN_HANDLE _ntcan_handle, double _timeout, Int32 _id_read, Int32 _id_write )
    throw (cCANSerial_ESDException*)
{
    if ( _timeout < 0.0 )
        throw new cCANSerial_ESDException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

    if ( _ntcan_handle == NTCAN_HANDLE( NTCAN_INVALID_HANDLE ) )
        throw new cCANSerial_ESDException( cMsg( "Cannot reuse invalid ESD CAN handle" ) );

    ntcan_handle = _ntcan_handle;
    net = -1;
    baudrate = 0;
    SetTimeout( _timeout );
    id_read = _id_read;
    id_write = _id_write;

    ungetch_valid = false;
}
//----------------------------------------------------------------------

char const* ESD_strerror( NTCAN_RESULT rc )
{
    switch (rc)
    {
        DEFINE_TO_CASECOMMAND( NTCAN_SUCCESS );
        DEFINE_TO_CASECOMMAND( NTCAN_RX_TIMEOUT );
        DEFINE_TO_CASECOMMAND( NTCAN_TX_TIMEOUT );
        DEFINE_TO_CASECOMMAND( NTCAN_TX_ERROR );
        DEFINE_TO_CASECOMMAND( NTCAN_CONTR_OFF_BUS );
        DEFINE_TO_CASECOMMAND( NTCAN_CONTR_BUSY );
        DEFINE_TO_CASECOMMAND( NTCAN_CONTR_WARN );
        DEFINE_TO_CASECOMMAND( NTCAN_NO_ID_ENABLED );
        DEFINE_TO_CASECOMMAND( NTCAN_ID_ALREADY_ENABLED );
        DEFINE_TO_CASECOMMAND( NTCAN_ID_NOT_ENABLED );

        DEFINE_TO_CASECOMMAND( NTCAN_INVALID_FIRMWARE );
        DEFINE_TO_CASECOMMAND( NTCAN_MESSAGE_LOST );
        DEFINE_TO_CASECOMMAND( NTCAN_INVALID_HARDWARE );

        DEFINE_TO_CASECOMMAND( NTCAN_PENDING_WRITE );
        DEFINE_TO_CASECOMMAND( NTCAN_PENDING_READ );
        DEFINE_TO_CASECOMMAND( NTCAN_INVALID_DRIVER );

        DEFINE_TO_CASECOMMAND( NTCAN_SOCK_CONN_TIMEOUT );
        DEFINE_TO_CASECOMMAND( NTCAN_SOCK_CMD_TIMEOUT );
        DEFINE_TO_CASECOMMAND( NTCAN_SOCK_HOST_NOT_FOUND );

        DEFINE_TO_CASECOMMAND( NTCAN_INVALID_PARAMETER );
        DEFINE_TO_CASECOMMAND( NTCAN_INVALID_HANDLE );
#ifndef OSNAME_LINUX
        // these errors are for Windows only ;-)
        DEFINE_TO_CASECOMMAND( NTCAN_IO_INCOMPLETE );
        DEFINE_TO_CASECOMMAND( NTCAN_IO_PENDING );
        DEFINE_TO_CASECOMMAND( NTCAN_HANDLE_FORCED_CLOSE );
        DEFINE_TO_CASECOMMAND( NTCAN_NOT_IMPLEMENTED );
        DEFINE_TO_CASECOMMAND( NTCAN_NOT_SUPPORTED );
#endif
        DEFINE_TO_CASECOMMAND( NTCAN_NET_NOT_FOUND );
        DEFINE_TO_CASECOMMAND( NTCAN_INSUFFICIENT_RESOURCES );

        DEFINE_TO_CASECOMMAND( NTCAN_OPERATION_ABORTED );
        DEFINE_TO_CASECOMMAND( NTCAN_WRONG_DEVICE_STATE );
    default:
        return "unknown";
    }
}
//-----------------------------------------------------------------


void cCANSerial_ESD::Open( void )
    throw (cCANSerial_ESDException*)
{
    NTCAN_RESULT rc;

    if ( ntcan_handle == NTCAN_HANDLE( NTCAN_INVALID_HANDLE ) )
    {
        // only open if we're not reusing an existing handle:

        //cerr << "opening can with timeout = " << timeout << ", in ms " << Int32( timeout * 1000.0 ) << "\n";
        rc = canOpen( net,
                      0,           // flags
                      CAN_ESD_TXQUEUESIZE,         // txquesize
                      CAN_ESD_RXQUEUESIZE,         // rxquesize
                      timeout_ms,
                      timeout_ms,
                      &ntcan_handle );

        if (rc != NTCAN_SUCCESS)
        {
            // open failed, so ensure that ntcan_handle is invalid
            ntcan_handle = NTCAN_HANDLE( NTCAN_INVALID_HANDLE );
            throw new cCANSerial_ESDException( cMsg( "Could not open ESD CAN net %d, rc = 0x%x = \"%s\"", net, int( rc ), ESD_strerror( rc ) ) );
        }

        rc = canSetBaudrate( ntcan_handle, BaudrateToBaudrateCode( baudrate ) );
        if (rc != NTCAN_SUCCESS)
            throw new cCANSerial_ESDException( cMsg( "Could not set baudrate to %lu on ESD CAN net %d, rc = 0x%x = \"%s\"", baudrate, net, int( rc ), ESD_strerror( rc ) ) );
    }

    rc = canIdAdd( ntcan_handle, id_read );
    if (rc != NTCAN_SUCCESS)
        throw new cCANSerial_ESDException( cMsg( "Could not add CAN ID 0x%03x on ESD CAN net %d, rc = 0x%x = \"%s\"", (unsigned int) id_read, net, int( rc ), ESD_strerror( rc ) ) );

    // (re)init member data:
    m_cmsg.len = 0;
    m_cmsg.msg_lost = 0;
    m_cmsg_next = 0;
}
//----------------------------------------------------------------------


bool cCANSerial_ESD::IsOpen( void )
    throw()
{
    return ( ntcan_handle!=NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );
}
//----------------------------------------------------------------------


void cCANSerial_ESD::Close( void )
    throw (cCANSerial_ESDException*)
{
    if ( ntcan_handle == NTCAN_HANDLE(NTCAN_INVALID_HANDLE) )
        throw new cCANSerial_ESDException( cMsg( "Could not close un-opened device" ) );

    canClose( ntcan_handle );
    ntcan_handle = NTCAN_HANDLE(NTCAN_INVALID_HANDLE);
}
//----------------------------------------------------------------------

UInt32 cCANSerial_ESD::BaudrateToBaudrateCode( unsigned long baudrate )
    throw (cCANSerial_ESDException*)
{
    switch (baudrate)
    {
    case 1000000: return NTCAN_BAUD_1000;
    case 800000: return NTCAN_BAUD_800;
    case 500000: return NTCAN_BAUD_500;
    case 250000: return NTCAN_BAUD_250;
    case 125000: return NTCAN_BAUD_125;
    case 100000: return NTCAN_BAUD_100;
    case 50000: return NTCAN_BAUD_50;
    case 20000: return NTCAN_BAUD_20;
    case 10000: return NTCAN_BAUD_10;
    }

    throw new cCANSerial_ESDException( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cCANSerial_ESD::write( char const *ptr, int len )
    throw (cCANSerial_ESDException*)
{
    assert( ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );

    //cerr << "in cCANSerial_ESD::write\n"; cerr.flush();
    if ( len == 0 )
        len = int( strlen( ptr ) );

    //cerr << "sending " << len << " bytes <" << ptr << "> to CAN net\n"; cerr.flush();

    // calculate number of CMSGS needed (max 8 data bytes per CMSG)
    Int32 len_cmsgs = len/8 + (((len%8)!=0) ? 1 : 0);

    if ( len_cmsgs > CAN_ESD_TXQUEUESIZE )
        throw new cCANSerial_ESDException( cMsg( "len_cmsgs = %d > %d, please adjust CAN_ESD_TXQUEUESIZE!", (int) len_cmsgs, CAN_ESD_TXQUEUESIZE ) );

    //---------------------
    // prepare CMSGs tor send:
#if SDH_USE_VCC
    // VCC cannot create variable size arrays on the stack; so use the heap
    CMSG* cmsg = new CMSG[ len_cmsgs ];
#else
    CMSG cmsg[ len_cmsgs ];
#endif
    for ( int i=0; i < len_cmsgs; i++)
    {
        cmsg[i].id = id_write;
        cmsg[i].len = min( 8, len-i*8 );
        for ( int j=0; j<cmsg[i].len; j++ )
            cmsg[i].data[ j ] = *(ptr++);
    }
    //---------------------

    //---------------------
    // now send the cmsgs and check return values (rc and len_cmsgs)
    Int32 len_cmsgs_save = len_cmsgs;
    NTCAN_RESULT rc;
    rc = canWrite( ntcan_handle, cmsg, (int32_t*) &len_cmsgs, NULL );
#if SDH_USE_VCC
    delete[] cmsg;
#endif
    if (rc != NTCAN_SUCCESS)
        throw new cCANSerial_ESDException( cMsg( "Could not write %d CMSGs on ESD CAN net %d, rc = 0x%x = \"%s\"", (int)len_cmsgs, net, int( rc ), ESD_strerror( rc ) ) );

    if ( len_cmsgs != len_cmsgs_save )
        throw new cCANSerial_ESDException( cMsg( "Could only send %d/%d CMSGs on ESD CAN net %d", (int)len_cmsgs, (int)len_cmsgs_save, net ) );

    return len;
}
//----------------------------------------------------------------------


ssize_t cCANSerial_ESD::Read( void *_data, ssize_t size, long timeout_us, bool return_on_less_data )
    throw (cCANSerial_ESDException*)
{
    assert( ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );

    char* data = (char*) _data;

    //---------------------
    // adjust rx timeout if necessary
    if ( long(timeout_ms) * 1000L != timeout_us )
    {
        SetTimeout( double(timeout_us) / 1E6 );
    }
    //---------------------

    //---------------------
    int bytes_read = 0;

    do
    {
        // copy remaining, not yet returned bytes from a previous canRead call to data
        for ( ; m_cmsg_next < m_cmsg.len  &&  bytes_read < size; m_cmsg_next++, bytes_read++ )
            *data++ = m_cmsg.data[ m_cmsg_next ];

        if ( bytes_read < size )
        {
            // if necessary read one more CMSGs with blocking call
            NTCAN_RESULT rc;
            Int32 len_cmsgs = 1;
            m_cmsg.len = 0;
            m_cmsg_next = 0;
            if ( timeout_us == 0 )
                rc = canTake( ntcan_handle, &m_cmsg, (int32_t*) &len_cmsgs );
            else
                rc = canRead( ntcan_handle, &m_cmsg, (int32_t*) &len_cmsgs, NULL );

            if (rc != NTCAN_SUCCESS)
                throw new cCANSerial_ESDException( cMsg( "Could not read CAN messages from ESD CAN net %d, rc = 0x%x = \"%s\"", net, int( rc ), ESD_strerror( rc ) ) );

            if ( len_cmsgs != 1  && timeout_us != 0 )
                throw new cCANSerial_ESDException( cMsg( "Could only read %d/%d CMSGs from ESD CAN net %d", int(len_cmsgs), 1, net ) );
            if ( len_cmsgs > 0 && m_cmsg.id != id_read )
                throw new cCANSerial_ESDException( cMsg( "Invalid CAN ID 0x%03x received, expected 0x%03x", (unsigned int) m_cmsg.id, (unsigned int) id_read ) );

            for ( ; m_cmsg_next < m_cmsg.len  &&  bytes_read < size; m_cmsg_next++, bytes_read++ )
                *data++ = m_cmsg.data[ m_cmsg_next ];
        }
    }
    while ( bytes_read < size  &&  !return_on_less_data );

    return bytes_read;
}
//----------------------------------------------------------------------


void cCANSerial_ESD::SetTimeout( double _timeout )
    throw (cSerialBaseException*)
{
    if ( _timeout < 0.0 )
        _timeout = 0.0;

    cSerialBase::SetTimeout( _timeout );
    timeout_ms = UInt32 (_timeout * 1000.0);

    if ( ntcan_handle != (NTCAN_HANDLE) NTCAN_INVALID_HANDLE )
    {
        // we already have a handle, so we must forward the timeout to the driver also:
        //cerr << "adjusting timeout = " << _timeout << ", in ms " << timeout_ms << "\n";

        NTCAN_RESULT rc;
        rc = canIoctl( ntcan_handle, NTCAN_IOCTL_SET_RX_TIMEOUT, &timeout_ms );

        if ( rc != NTCAN_SUCCESS )
            throw new cCANSerial_ESDException( cMsg( "Could not set new rx timeout for ESD CAN net %d, rc = 0x%x = \"%s\"", net, int( rc ), ESD_strerror( rc ) ) );
    }
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================