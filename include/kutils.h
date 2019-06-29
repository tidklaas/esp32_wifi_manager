/*
 * This file is part of the ESP WiFi Manager project.
 * Copyright (C) 2019  Tido Klaassen <tido_wmngr@4gh.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 *              
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */  

#ifndef KUTILS_H
#define KUTILS_H

/* This is a collection of various macros taken from the Linux kernel. */

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#endif

#if !defined(offsetof)
#define offsetof(type, member) ((size_t) &((type *)0)->member)
#endif

#if !defined(container_of)
#define container_of(ptr, type, member) ({                  \
    const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
    (type *)( (char *)__mptr - offsetof(type,member) );})
#endif

/* Jiffy overflow handling. */
#define typecheck(type,x) \
({  type __dummy; \
     typeof(x) __dummy2; \
     (void)(&__dummy == &__dummy2); \
     1; \
})

#define time_after(a, b)            \
    (typecheck(unsigned int, a) &&  \
     typecheck(unsigned int, b) &&  \
     ((long)((b) - (a)) < 0))
#define time_before(a, b)       time_after(b, a)

#define time_after_eq(a, b)         \
    (typecheck(unsigned long, a) && \
     typecheck(unsigned long, b) && \
     ((long)((a) - (b)) >= 0))
#define time_before_eq(a, b)    time_after_eq(b, a)

#define time_in_range(a, b, c)      \
    (time_after_eq(a, b) &&         \
     time_before_eq(a, c))

#define time_in_range_open(a, b, c) \
    (time_after_eq(a, b) &&         \
     time_before(a, c))

#endif // KUTILS_H
