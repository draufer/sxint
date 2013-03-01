#ifndef HELPER_H
# define HELPER_H
/*
 * helper.h
 *
 *  Created on: 01.03.2013
 *  Changed on: 01.03.2013
 *  Version:    0.1
 *  Copyright:  Jan Seiffert, Frederic Endner-Duehr, Dirk Raufer
 *
 * SX functionality inspired from the original Arduino library of Michael Blank:
 *  http://www.oscale.net/selectrix
 *
 * USB code taken from PJRC.COM, LLC
 *  http://www.pjrc.com/teensy/usb_serial.html
 *  The USB code was modifed by us for our specific needs.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define anum(x) (sizeof(x)/sizeof(*(x)))
	/* divide while always rounding up */
#define DIV_ROUNDUP(a, b) \
	(((a) + (b) - 1) / (b))
	/* helper to check GCC version */
#if defined __GNUC__ && defined __GNUC_MINOR__
# define _GNUC_PREREQ(maj, min) \
	((__GNUC__ << 16) + __GNUC_MINOR__ >= ((maj) << 16) + (min))
#else
# define _GNUC_PREREQ(maj, min) 0
#endif
	/* can we use attributes */
#if defined(__GNUC__) && __GNUC__ >= 2
# define GCC_ATTRIB(x) __attribute__((x))
#else
# define GCC_ATTRIB(x)
# define __attribute__(xyz)	/* Ignore */
#endif
	/* help compiler on conditional jump decisions */
#if _GNUC_PREREQ (2,96)
# define likely(x)	__builtin_expect(!!(x), 1)
# define unlikely(x)	__builtin_expect(!!(x), 0)
#else
# define likely(x)	(x)
# define unlikely(x)	(x)
#endif
	/* prevent the compiler from inlining */
#if _GNUC_PREREQ (3,1)
# undef noinline
# define noinline GCC_ATTRIB(__noinline__)
#else
# ifndef noinline
#  define noinline
# endif
#endif
	/* deviate from the global optimisation setting */
#if _GNUC_PREREQ (4,4)
# define GCC_ATTR_OPTIMIZE(x) GCC_ATTRIB(__optimize__ (x))
#else
# define GCC_ATTR_OPTIMIZE(x)
#endif

#endif
