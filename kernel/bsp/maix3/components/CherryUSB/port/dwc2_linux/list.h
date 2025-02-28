#ifndef _LINUX_LIST_H
#define _LINUX_LIST_H

#include <stdlib.h>
#include "usb_list.h"

/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */
#define LIST_POISON1 (NULL)
#define LIST_POISON2 (NULL)

#define LIST_HEAD_INIT(name) USB_DLIST_OBJECT_INIT(name)

#define LIST_HEAD(name) USB_DLIST_DEFINE(name)

static inline void INIT_LIST_HEAD(usb_dlist_t *list)
{
    usb_dlist_init(list);
}

/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
static inline void list_add(usb_dlist_t *new, usb_dlist_t *head)
{
    usb_dlist_insert_after(head, new);
}

/**
 * list_add_tail - add a new entry
 * @new: new entry to be added
 * @head: list head to add it before
 *
 * Insert a new entry before the specified head.
 * This is useful for implementing queues.
 */
static inline void list_add_tail(usb_dlist_t *new, usb_dlist_t *head)
{
    usb_dlist_insert_before(head, new);
}

/**
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty() on entry does not return true after this, the entry is
 * in an undefined state.
 */
static inline void __list_del_entry(usb_dlist_t *entry)
{
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
}

static inline void list_del(usb_dlist_t *entry)
{
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;

    entry->next = LIST_POISON1;
    entry->prev = LIST_POISON2;
}

/**
 * list_replace - replace old entry by new one
 * @old : the element to be replaced
 * @new : the new element to insert
 *
 * If @old was empty, it will be overwritten.
 */
static inline void list_replace(usb_dlist_t *old,
                                usb_dlist_t *new)
{
    new->next = old->next;
    new->next->prev = new;
    new->prev = old->prev;
    new->prev->next = new;
}

static inline void list_replace_init(usb_dlist_t *old,
                                     usb_dlist_t *new)
{
    list_replace(old, new);
    INIT_LIST_HEAD(old);
}

/**
 * list_del_init - deletes entry from list and reinitialize it.
 * @entry: the element to delete from the list.
 */
static inline void list_del_init(usb_dlist_t *entry)
{
    __list_del_entry(entry);
    INIT_LIST_HEAD(entry);
}

/**
 * list_move - delete from one list and add as another's head
 * @list: the entry to move
 * @head: the head that will precede our entry
 */
static inline void list_move(usb_dlist_t *list, usb_dlist_t *head)
{
    __list_del_entry(list);
    usb_dlist_insert_after(head, list);
}

/**
 * list_move_tail - delete from one list and add as another's tail
 * @list: the entry to move
 * @head: the head that will follow our entry
 */
static inline void list_move_tail(usb_dlist_t *list,
                                  usb_dlist_t *head)
{
    __list_del_entry(list);
    usb_dlist_insert_before(head, list);
}


/**
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
static inline int list_empty(const usb_dlist_t *head)
{
    return usb_dlist_isempty(head);
}

/**
 * list_entry - get the struct for this entry
 * @ptr:	the &usb_dlist_t pointer.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 */
#define list_entry(ptr, type, member) \
    usb_dlist_entry(ptr, type, member)

/**
 * list_first_entry - get the first element from a list
 * @ptr:	the list head to take the element from.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_first_entry(ptr, type, member) \
    usb_dlist_first_entry(ptr, type, member)

/**
 * list_for_each_entry	-	iterate over list of given type
 * @pos:	the type * to use as a loop cursor.
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry(pos, head, member)				\
    usb_dlist_for_each_entry(pos, head, member)


/**
 * list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop cursor.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry_safe(pos, n, head, member)			\
    usb_dlist_for_each_entry_safe(pos, n, head, member)

/**
 * list_for_each_safe - iterate over a list safe against removal of list entry
 * @pos:	the &struct list_head to use as a loop cursor.
 * @n:		another &struct list_head to use as temporary storage
 * @head:	the head for your list.
 */
#define list_for_each_safe(pos, n, head) \
    usb_dlist_for_each_safe(pos, n, head)

#endif
