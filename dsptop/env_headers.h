/* =============================================================================
 *
 *            TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION
 *
 *  Property of Texas Instruments
 *  For Unrestricted Internal Use Only 
 *  Unauthorized reproduction and/or distribution is strictly
 *  prohibited.
 *  This product is protected under copyright law and trade secret law  
 *  as an unpublished work.
 *  Created 2010, (C) Copyright 2010 Texas Instruments.  All rights 
 *  reserved.
 */
/**
 *  @Component    ENV_HEADERS 
 * 
 *  @Filename     env_headers.h
 *
 *  @Description  wrapper header for evironment-wide headers like GlobalTypes.h,
 *                cram.h, baseaddress.h etc. This header simplifies reuse of code
 *                from IP to SoC as it is different in Topsim, IP prototyping
 *                env etc.
 *
 *//*======================================================================== */

#ifndef __ENV_HEADERS_H
#define __ENV_HEADERS_H

#ifndef _UINT_TYPES_FLAG
	#define _UINT_TYPES_FLAG
/* ----------------------------------------------------------------------------
 * TYPE: i8
 *
 * DESCRIPTION: signed 8-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef signed char  i8;
/* ----------------------------------------------------------------------------
 * TYPE: i16
 *
 * DESCRIPTION: signed 16-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef short i16;
/* ----------------------------------------------------------------------------
 * TYPE: i32
 *
 * DESCRIPTION: signed 32-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef int i32;
/* ----------------------------------------------------------------------------
 * TYPE: i64
 *
 * DESCRIPTION: signed 64-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
	#ifdef WIN32
		typedef __int64 i64;
	#else
		typedef long long i64;
	#endif

/* ----------------------------------------------------------------------------
 * TYPE: s8
 *
 * DESCRIPTION: signed 8-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef signed char  s8;
/* ----------------------------------------------------------------------------
 * TYPE: s16
 *
 * DESCRIPTION: signed 16-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef short s16;
/* ----------------------------------------------------------------------------
 * TYPE: s32
 *
 * DESCRIPTION: signed 32-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef int s32;
/* ----------------------------------------------------------------------------
 * TYPE: s64
 *
 * DESCRIPTION: signed 64-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
	#ifdef WIN32
		typedef __int64 s64;
	#else
		typedef long long s64;
	#endif    

/* ----------------------------------------------------------------------------
 * TYPE: u8
 *
 * DESCRIPTION: unsigned 8-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef unsigned char  u8;
/* ----------------------------------------------------------------------------
 * TYPE: u16
 *
 * DESCRIPTION: unsigned 16-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef unsigned short u16;
/* ----------------------------------------------------------------------------
 * TYPE: u32
 *
 * DESCRIPTION: unsigned 32-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
    typedef unsigned int u32;
/* ----------------------------------------------------------------------------
 * TYPE: u64
 *
 * DESCRIPTION: unsigned 64-bit integer type of exact size
 * -----------------------------------------------------------------------------
 */
	#ifdef WIN32
		typedef unsigned __int64 u64;
	#else
		typedef unsigned long long u64;
	#endif       

#endif

/* ----------------------------------------------------------------------------
* TYPE: boolean_t        
*
* DESCRIPTION:  Boolean Type True, False 
*
* -----------------------------------------------------------------------------
*/
#if !defined (BOOL_FLAG)
  #define BOOL_FLAG
  typedef enum boolean_label {
    False= 0, 
    True=1 
  } boolean_t, *pBoolean_t;

/* Boolean  Definition */
  #define BOOL           boolean_t
#endif

#endif	/* __ENV_HEADERS_H */
/* EOF */

