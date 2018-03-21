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

#include "usteer.h"
#include "node.h"

static bool
below_assoc_threshold(struct sta_info *si_cur, struct sta_info *si_new)
{
	int n_assoc_cur = si_cur->node->n_assoc;
	int n_assoc_new = si_new->node->n_assoc;
	bool ref_5g = si_cur->node->freq > 4000;
	bool node_5g = si_new->node->freq > 4000;

	if (ref_5g && !node_5g)
		n_assoc_new += config.band_steering_threshold;
	else if (!ref_5g && node_5g)
		n_assoc_cur += config.band_steering_threshold;

	n_assoc_new += config.load_balancing_threshold;

	if (n_assoc_new > n_assoc_cur) {
		MSG_T_STA("band_steering_threshold,load_balancing_threshold",
			si_cur->sta->addr, "exeeded (bs=%u, lb=%u)\n",
			config.band_steering_threshold,
			config.load_balancing_threshold);
	}
	return n_assoc_new <= n_assoc_cur;
}

static bool
better_signal_strength(struct sta_info *si_cur, struct sta_info *si_new)
{
	const bool is_better = si_new->signal - si_cur->signal
				> (int) config.signal_diff_threshold;

	if (!config.signal_diff_threshold)
		return false;

	if (is_better) {
		MSG_T_STA("signal_diff_threshold", si_cur->sta->addr,
			"exceeded (config=%i) (real=%i)\n",
			config.signal_diff_threshold,
			si_new->signal - si_cur->signal);
	}
	return is_better;
}

static bool
below_load_threshold(struct sta_info *si)
{
	return si->node->n_assoc >= config.load_kick_min_clients &&
	       si->node->load > config.load_kick_threshold;
}

static bool
has_better_load(struct sta_info *si_cur, struct sta_info *si_new)
{
	return !below_load_threshold(si_cur) && below_load_threshold(si_new);
}

static bool
below_max_assoc(struct sta_info *si)
{
	struct usteer_node *node = si->node;

	return !node->max_assoc || node->n_assoc < node->max_assoc;
}

static bool
is_better_candidate(struct sta_info *si_cur, struct sta_info *si_new)
{
	if (!below_max_assoc(si_new))
		return false;

	return below_assoc_threshold(si_cur, si_new) ||
	       better_signal_strength(si_cur, si_new) ||
	       has_better_load(si_cur, si_new);
}

static struct sta_info *
find_better_candidate(struct sta_info *si_ref)
{
	struct sta_info *si;
	struct sta *sta = si_ref->sta;

	list_for_each_entry(si, &sta->nodes, list) {
		if (si == si_ref)
			continue;

		if (current_time - si->seen > config.seen_policy_timeout) {
			MSG_T_STA("seen_policy_timeout", si->sta->addr,
				"timeout exceeded (%u)\n", config.seen_policy_timeout);
			continue;
		}

		if (strcmp(si->node->ssid, si_ref->node->ssid) != 0)
			continue;

		if (is_better_candidate(si_ref, si) &&
		    !is_better_candidate(si, si_ref))
			return si;
	}
	return NULL;
}

static int
snr_to_signal(struct usteer_node *node, int snr)
{
	int noise = -95;

	if (snr < 0)
		return snr;

	if (node->noise)
		noise = node->noise;

	return noise + snr;
}

bool
usteer_check_request(struct sta_info *si, enum usteer_event_type type)
{
	struct sta_info *si_new;
	int min_signal;

	if (type == EVENT_TYPE_ASSOC)
		return true;

	if (si->stats[type].blocked_cur >= config.max_retry_band) {
		MSG_T_STA("max_retry_band", si->sta->addr,
			"max retry (%u) exceeded\n", config.max_retry_band);
		return true;
	}

	min_signal = snr_to_signal(si->node, config.min_connect_snr);
	if (si->signal < min_signal) {
		if (type != EVENT_TYPE_PROBE || config.debug_level >= MSG_DEBUG)
			MSG(VERBOSE, "Ignoring %s request from "MAC_ADDR_FMT" due to low signal (%d < %d)\n",
			    event_types[type], MAC_ADDR_DATA(si->sta->addr),
			    si->signal, min_signal);
		MSG_T_STA("min_connect_snr", si->sta->addr,
			"snr to low (config=%i) (real=%i)\n",
			min_signal, si->signal);
		return false;
	}

	if (current_time - si->created < config.initial_connect_delay) {
		if (type != EVENT_TYPE_PROBE || config.debug_level >= MSG_DEBUG)
			MSG(VERBOSE, "Ignoring %s request from "MAC_ADDR_FMT" during initial connect delay\n",
			    event_types[type], MAC_ADDR_DATA(si->sta->addr));
		MSG_T_STA("initial_connect_delay", si->sta->addr,
			"is below delay (%u)\n", config.initial_connect_delay);
		return false;
	}

	si_new = find_better_candidate(si);
	if (!si_new)
		return true;

	if (type != EVENT_TYPE_PROBE || config.debug_level >= MSG_DEBUG)
		MSG(VERBOSE, "Ignoring %s request from "MAC_ADDR_FMT", "
			"node (local/remote): %s/%s, "
			"signal=%d/%d, n_assoc=%d/%d\n", event_types[type],
			MAC_ADDR_DATA(si->sta->addr),
			usteer_node_name(si->node), usteer_node_name(si_new->node),
			si->signal, si_new->signal,
			si->node->n_assoc, si_new->node->n_assoc);

	return false;
}

static bool
is_more_kickable(struct sta_info *si_cur, struct sta_info *si_new)
{
	if (!si_cur)
		return true;

	if (si_new->kick_count > si_cur->kick_count)
		return false;

	return si_cur->signal > si_new->signal;
}

static void
usteer_roam_set_state(struct sta_info *si, enum roam_trigger_state state)
{
	static const char * const state_names[] = {
#define _S(n) [ROAM_TRIGGER_##n] = #n,
		__roam_trigger_states
#undef _S
	};

	si->roam_event = current_time;

	if (si->roam_state == state) {
		if (si->roam_state == ROAM_TRIGGER_IDLE) {
			si->roam_tries = 0;
			return;
		}

		si->roam_tries++;
	} else {
		si->roam_tries = 0;
	}

	si->roam_state = state;

	MSG(VERBOSE, "Roam trigger SM for client "MAC_ADDR_FMT": state=%s, tries=%d, signal=%d\n",
	    MAC_ADDR_DATA(si->sta->addr), state_names[state], si->roam_tries, si->signal);
}

static bool
usteer_roam_trigger_sm(struct sta_info *si)
{
	struct sta_info *si_new;
	int min_signal;

	min_signal = snr_to_signal(si->node, config.roam_trigger_snr);

	switch (si->roam_state) {
	case ROAM_TRIGGER_SCAN:
		if (current_time - si->roam_event < config.roam_scan_interval)
			break;

		if (find_better_candidate(si) ||
		    si->roam_scan_done > si->roam_event) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN_DONE);
			break;
		}

		if (config.roam_scan_tries &&
		    si->roam_tries >= config.roam_scan_tries) {
			usteer_roam_set_state(si, ROAM_TRIGGER_WAIT_KICK);
			break;
		}

		usteer_ubus_trigger_client_scan(si);
		usteer_roam_set_state(si, ROAM_TRIGGER_SCAN);
		break;

	case ROAM_TRIGGER_IDLE:
		if (find_better_candidate(si)) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN_DONE);
			break;
		}

		usteer_roam_set_state(si, ROAM_TRIGGER_SCAN);
		break;

	case ROAM_TRIGGER_SCAN_DONE:
		/* Check for stale scan results, kick back to SCAN state if necessary */
		if (current_time - si->roam_scan_done > 2 * config.roam_scan_interval) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN);
			break;
		}

		si_new = find_better_candidate(si);
		if (!si_new)
			break;

		usteer_roam_set_state(si, ROAM_TRIGGER_WAIT_KICK);
		break;

	case ROAM_TRIGGER_WAIT_KICK:
		if (si->signal > min_signal)
			break;

		usteer_roam_set_state(si, ROAM_TRIGGER_NOTIFY_KICK);
		usteer_ubus_notify_client_disassoc(si);
		break;
	case ROAM_TRIGGER_NOTIFY_KICK:
		if (current_time - si->roam_event < config.roam_kick_delay * 100)
			break;

		usteer_roam_set_state(si, ROAM_TRIGGER_KICK);
		break;
	case ROAM_TRIGGER_KICK:
		usteer_ubus_kick_client(si);
		usteer_roam_set_state(si, ROAM_TRIGGER_IDLE);
		return true;
	}

	return false;
}

static void
usteer_local_node_roam_check(struct usteer_local_node *ln)
{
	struct sta_info *si;
	int min_signal;

	if (config.roam_scan_snr)
		min_signal = config.roam_scan_snr;
	else if (config.roam_trigger_snr)
		min_signal = config.roam_trigger_snr;
	else
		return;

	usteer_update_time();
	min_signal = snr_to_signal(&ln->node, min_signal);

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (!si->connected || si->signal >= min_signal ||
		    current_time - si->roam_kick < config.roam_trigger_interval) {
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE);
			continue;
		}

		/*
		 * If the state machine kicked a client, other clients should wait
		 * until the next turn
		 */
		if (usteer_roam_trigger_sm(si))
			return;
	}
}

static void
usteer_local_node_snr_kick(struct usteer_local_node *ln)
{
	struct sta_info *si;
	int min_signal;

	if (!config.min_snr)
		return;

	min_signal = snr_to_signal(&ln->node, config.min_snr);

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (!si->connected)
			continue;

		if (si->signal >= min_signal)
			continue;

		si->kick_count++;

		MSG(VERBOSE, "Kicking client "MAC_ADDR_FMT" due to low SNR, signal=%d\n",
			MAC_ADDR_DATA(si->sta->addr), si->signal);

		usteer_ubus_kick_client(si);
		return;
	}
}

void
usteer_local_node_kick(struct usteer_local_node *ln)
{
	struct usteer_node *node = &ln->node;
	struct sta_info *kick1 = NULL, *kick2 = NULL;
	struct sta_info *candidate = NULL;
	struct sta_info *si;

	usteer_local_node_roam_check(ln);
	usteer_local_node_snr_kick(ln);

	if (!config.load_kick_enabled || !config.load_kick_threshold ||
	    !config.load_kick_delay)
		return;

	if (node->load < config.load_kick_threshold) {
		MSG_T("load_kick_threshold",
			"is below load for this node (config=%i) (real=%i)\n",
			config.load_kick_threshold, node->load);
		ln->load_thr_count = 0;
		return;
	}

	if (++ln->load_thr_count <=
	    DIV_ROUND_UP(config.load_kick_delay, config.local_sta_update)) {
		MSG_T("load_kick_delay", "delay kicking (config=%i)\n",
			config.load_kick_delay);
		return;
	}

	MSG(VERBOSE, "AP load threshold exceeded on %s (%d), try to kick a client\n",
	    usteer_node_name(node), node->load);

	ln->load_thr_count = 0;
	if (node->n_assoc < config.load_kick_min_clients) {
		MSG_T("load_kick_min_clients",
			"min limit reached, stop kicking clients on this node "
			"(n_assoc=%i) (config=%i)\n",
			node->n_assoc, config.load_kick_min_clients);
		return;
	}

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		struct sta_info *tmp;

		if (!si->connected)
			continue;

		if (is_more_kickable(kick1, si))
			kick1 = si;

		tmp = find_better_candidate(si);
		if (!tmp)
			continue;

		if (is_more_kickable(kick2, si)) {
			kick2 = si;
			candidate = tmp;
		}
	}

	if (!kick1)
		return;

	if (kick2)
		kick1 = kick2;

	MSG(VERBOSE, "Kicking client "MAC_ADDR_FMT", signal=%d, better_candidate=%s\n",
	    MAC_ADDR_DATA(kick1->sta->addr), kick1->signal,
		candidate ? usteer_node_name(candidate->node) : "(none)");

	kick1->kick_count++;
	usteer_ubus_kick_client(kick1);
}
