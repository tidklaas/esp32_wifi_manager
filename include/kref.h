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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _KREF_H_
#define _KREF_H_

#include <stdint.h>
#include <stdatomic.h>

struct kref {
    atomic_int count;
};

static inline void kref_init(struct kref *kref)
{
    atomic_init(&(kref->count), 1);
}

static inline void kref_get(struct kref *kref)
{
    int old;

    old = atomic_fetch_add(&(kref->count), 1);
    configASSERT(old >= 1);
}

static inline int kref_put(struct kref *kref, void (*release)(struct kref *kref))
{
    int result;
    int old;

    configASSERT(release != NULL);

    result = 0;

    old = atomic_fetch_sub(&(kref->count), 1);
    configASSERT(old >= 1);

    if(old == 1){
        release(kref);
        result = 1;
    }

    return result;
}

#endif /* _KREF_H_ */
