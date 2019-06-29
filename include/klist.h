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
 
#ifndef _KLIST_H_
#define _KLIST_H_

#include "kutils.h"

/*
 * Inspired by / stolen from Linux kernel list.h
 */

struct klist_head {
	struct klist_head *next, *prev;
};


#define KLIST_HEAD_INIT(name) { &(name), &(name) }

#define KLIST_HEAD(name) \
	struct klist_head name = KLIST_HEAD_INIT(name)

static inline void INIT_KLIST_HEAD(struct klist_head *list)
{
	list->next = list;
	list->prev = list;
}

#define klist_entry(ptr, type, member) \
    container_of(ptr, type, member)

#define klist_first_entry(ptr, type, member) \
    klist_entry((ptr)->next, type, member)

#define klist_first_entry_or_null(ptr, type, member) \
    (!klist_empty(ptr) ? klist_first_entry(ptr, type, member) : NULL)

#define klist_next_entry(pos, member) \
    klist_entry((pos)->member.next, typeof(*(pos)), member)

#define klist_prev_entry(pos, member) \
    klist_entry((pos)->member.prev, typeof(*(pos)), member)

#define klist_for_each_entry(pos, head, member)                     \
    for (pos = klist_entry((head)->next, typeof(*pos), member);	    \
         &pos->member != (head);                                    \
         pos = klist_entry(pos->member.next, typeof(*pos), member))

#define klist_for_each_entry_safe(pos, n, head, member)             \
	for (pos = klist_entry((head)->next, typeof(*pos), member),     \
        n = klist_entry(pos->member.next, typeof(*pos), member);    \
        &pos->member != (head);                                     \
        pos = n, n = klist_entry(n->member.next, typeof(*n), member))

static inline int klist_empty(const struct klist_head *head)
{
    return head->next == head;
}

static inline int klist_is_first(const struct klist_head *list,
                                 const struct klist_head *head)
{
    return list->prev == head;
}

static inline int klist_is_last(const struct klist_head *list,
                                const struct klist_head *head)
{
        return list->next == head;
}

static inline void __klist_add(struct klist_head *_new,
                               struct klist_head *prev,
                               struct klist_head *next)
{
    next->prev = _new;
    _new->next = next;
    _new->prev = prev;
    prev->next = _new;
}

static inline void klist_add_tail(struct klist_head *_new,
                                  struct klist_head *head)
{
    __klist_add(_new, head->prev, head);
}

static inline void __klist_del(struct klist_head *prev,
                               struct klist_head *next)
{
    next->prev = prev;
    prev->next = next;
}

static inline void __klist_del_entry(struct klist_head *entry)
{
    __klist_del(entry->prev, entry->next);
}

static inline void klist_del(struct klist_head *entry)
{
    __klist_del(entry->prev, entry->next);
    entry->next = NULL;
    entry->prev = NULL;
}

static inline void klist_del_init(struct klist_head *entry)
{
    __klist_del_entry(entry);
    INIT_KLIST_HEAD(entry);
}

#endif // _KLIST_H_
