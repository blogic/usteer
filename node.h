/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Copyright (C) 2020 embedd.ch 
 *   Copyright (C) 2020 Felix Fietkau <nbd@nbd.name> 
 *   Copyright (C) 2020 John Crispin <john@phrozen.org> 
 */

#ifndef __APMGR_NODE_H
#define __APMGR_NODE_H

#include "usteer.h"

enum local_req_state {
	REQ_IDLE,
	REQ_CLIENTS,
	REQ_RRM_SET_LIST,
	REQ_RRM_GET_OWN,
	__REQ_MAX
};

struct usteer_local_node {
	struct usteer_node node;

	struct ubus_subscriber ev;
	struct uloop_timeout update;

	const char *iface;
	int ifindex;
	int wiphy;

	struct ubus_request req;
	struct uloop_timeout req_timer;
	int req_state;

	uint32_t obj_id;

	float load_ewma;
	int load_thr_count;

	uint64_t time, time_busy;

	struct {
		bool present;
		struct uloop_timeout update;
	} nl80211;
	struct {
		struct ubus_request req;
		bool req_pending;
		bool status_complete;
	} netifd;
};

struct interface;
struct usteer_remote_node {
	struct avl_node avl;
	const char *name;

	struct usteer_node node;
	struct interface *iface;

	int check;
};

extern struct avl_tree local_nodes;
extern struct avl_tree remote_nodes;

#endif
