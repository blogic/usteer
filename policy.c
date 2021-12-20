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
#include "event.h"

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

	return n_assoc_new <= n_assoc_cur;
}

static bool
better_signal_strength(struct sta_info *si_cur, struct sta_info *si_new)
{
	const bool is_better = si_new->signal - si_cur->signal
				> (int) config.signal_diff_threshold;

	if (!config.signal_diff_threshold)
		return false;

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

static uint32_t
is_better_candidate(struct sta_info *si_cur, struct sta_info *si_new)
{
	uint32_t reasons = 0;

	if (!below_max_assoc(si_new))
		return 0;

	if (below_assoc_threshold(si_cur, si_new) &&
	    !below_assoc_threshold(si_new, si_cur))
		reasons |= (1 << UEV_SELECT_REASON_NUM_ASSOC);

	if (better_signal_strength(si_cur, si_new))
		reasons |= (1 << UEV_SELECT_REASON_SIGNAL);

	if (has_better_load(si_cur, si_new) &&
		!has_better_load(si_cur, si_new))
		reasons |= (1 << UEV_SELECT_REASON_LOAD);

	return reasons;
}

static struct sta_info *
find_better_candidate(struct sta_info *si_ref, struct uevent *ev, uint32_t required_criteria)
{
	struct sta_info *si;
	struct sta *sta = si_ref->sta;
	uint32_t reasons;

	list_for_each_entry(si, &sta->nodes, list) {
		if (si == si_ref)
			continue;

		if (current_time - si->seen > config.seen_policy_timeout)
			continue;

		if (strcmp(si->node->ssid, si_ref->node->ssid) != 0)
			continue;

		reasons = is_better_candidate(si_ref, si);
		if (!reasons)
			continue;

		if (!(reasons & required_criteria))
			continue;

		if (ev) {
			ev->si_other = si;
			ev->select_reasons = reasons;
		}

		return si;
	}

	return NULL;
}

int
usteer_snr_to_signal(struct usteer_node *node, int snr)
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
	struct uevent ev = {
		.si_cur = si,
	};
	int min_signal;
	bool ret = true;

	if (type == EVENT_TYPE_AUTH)
		goto out;

	if (type == EVENT_TYPE_ASSOC) {
		/* Check if assoc request has lower signal than min_signal.
		 * If this is the case, block assoc even when assoc steering is enabled.
		 *
		 * Otherwise, the client potentially ends up in a assoc - kick loop.
		 */
		if (config.min_snr && si->signal < usteer_snr_to_signal(si->node, config.min_snr)) {
			ev.reason = UEV_REASON_LOW_SIGNAL;
			ev.threshold.cur = si->signal;
			ev.threshold.ref = usteer_snr_to_signal(si->node, config.min_snr);
			ret = false;
			goto out;
		} else if (!config.assoc_steering) {
			goto out;
		}
	}

	min_signal = usteer_snr_to_signal(si->node, config.min_connect_snr);
	if (si->signal < min_signal) {
		ev.reason = UEV_REASON_LOW_SIGNAL;
		ev.threshold.cur = si->signal;
		ev.threshold.ref = min_signal;
		ret = false;
		goto out;
	}

	if (current_time - si->created < config.initial_connect_delay) {
		ev.reason = UEV_REASON_CONNECT_DELAY;
		ev.threshold.cur = current_time - si->created;
		ev.threshold.ref = config.initial_connect_delay;
		ret = false;
		goto out;
	}

	if (!find_better_candidate(si, &ev, UEV_SELECT_REASON_ALL))
		goto out;

	ev.reason = UEV_REASON_BETTER_CANDIDATE;
	ev.node_cur = si->node;
	ret = false;

out:
	switch (type) {
	case EVENT_TYPE_PROBE:
		ev.type = UEV_PROBE_REQ_ACCEPT;
		break;
	case EVENT_TYPE_ASSOC:
		ev.type = UEV_ASSOC_REQ_ACCEPT;
		break;
	case EVENT_TYPE_AUTH:
		ev.type = UEV_AUTH_REQ_ACCEPT;
		break;
	default:
		break;
	}

	if (!ret)
		ev.type++;

	if (!ret && si->stats[type].blocked_cur >= config.max_retry_band) {
		ev.reason = UEV_REASON_RETRY_EXCEEDED;
		ev.threshold.cur = si->stats[type].blocked_cur;
		ev.threshold.ref = config.max_retry_band;
	}
	usteer_event(&ev);

	return ret;
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
usteer_roam_set_state(struct sta_info *si, enum roam_trigger_state state,
		      struct uevent *ev)
{
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
	usteer_event(ev);
}

static bool
usteer_roam_trigger_sm(struct sta_info *si)
{
	struct uevent ev = {
		.si_cur = si,
	};
	int min_signal;

	min_signal = usteer_snr_to_signal(si->node, config.roam_trigger_snr);

	switch (si->roam_state) {
	case ROAM_TRIGGER_SCAN:
		if (current_time - si->roam_event < config.roam_scan_interval)
			break;

		if (find_better_candidate(si, &ev, (1 << UEV_SELECT_REASON_SIGNAL)) ||
		    si->roam_scan_done > si->roam_event) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN_DONE, &ev);
			break;
		}

		if (config.roam_scan_tries &&
		    si->roam_tries >= config.roam_scan_tries) {
			usteer_roam_set_state(si, ROAM_TRIGGER_WAIT_KICK, &ev);
			break;
		}

		usteer_ubus_trigger_client_scan(si);
		usteer_roam_set_state(si, ROAM_TRIGGER_SCAN, &ev);
		break;

	case ROAM_TRIGGER_IDLE:
		if (find_better_candidate(si, &ev, (1 << UEV_SELECT_REASON_SIGNAL))) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN_DONE, &ev);
			break;
		}

		usteer_roam_set_state(si, ROAM_TRIGGER_SCAN, &ev);
		break;

	case ROAM_TRIGGER_SCAN_DONE:
		/* Check for stale scan results, kick back to SCAN state if necessary */
		if (current_time - si->roam_scan_done > 2 * config.roam_scan_interval) {
			usteer_roam_set_state(si, ROAM_TRIGGER_SCAN, &ev);
			break;
		}

		if (find_better_candidate(si, &ev, (1 << UEV_SELECT_REASON_SIGNAL)))
			usteer_roam_set_state(si, ROAM_TRIGGER_WAIT_KICK, &ev);

		break;

	case ROAM_TRIGGER_WAIT_KICK:
		if (si->signal > min_signal)
			break;

		usteer_roam_set_state(si, ROAM_TRIGGER_NOTIFY_KICK, &ev);
		usteer_ubus_notify_client_disassoc(si);
		break;
	case ROAM_TRIGGER_NOTIFY_KICK:
		if (current_time - si->roam_event < config.roam_kick_delay * 100)
			break;

		usteer_roam_set_state(si, ROAM_TRIGGER_KICK, &ev);
		break;
	case ROAM_TRIGGER_KICK:
		usteer_ubus_kick_client(si);
		usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
		return true;
	}

	return false;
}

static void
usteer_local_node_roam_check(struct usteer_local_node *ln, struct uevent *ev)
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
	min_signal = usteer_snr_to_signal(&ln->node, min_signal);

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (si->connected != STA_CONNECTED || si->signal >= min_signal ||
		    current_time - si->roam_kick < config.roam_trigger_interval) {
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, ev);
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
	struct uevent ev = {
		.node_local = &ln->node,
	};
	struct sta_info *si;
	int min_signal;

	if (!config.min_snr)
		return;

	min_signal = usteer_snr_to_signal(&ln->node, config.min_snr);
	ev.threshold.ref = min_signal;

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (si->connected != STA_CONNECTED)
			continue;

		if (si->signal >= min_signal)
			continue;

		si->kick_count++;

		ev.type = UEV_SIGNAL_KICK;
		ev.threshold.cur = si->signal;
		ev.count = si->kick_count;
		usteer_event(&ev);

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
	struct uevent ev = {
		.node_local = &ln->node,
	};
	unsigned int min_count = DIV_ROUND_UP(config.load_kick_delay, config.local_sta_update);

	usteer_local_node_roam_check(ln, &ev);
	usteer_local_node_snr_kick(ln);

	if (!config.load_kick_enabled || !config.load_kick_threshold ||
	    !config.load_kick_delay)
		return;

	if (node->load < config.load_kick_threshold) {
		if (!ln->load_thr_count)
			return;

		ln->load_thr_count = 0;
		ev.type = UEV_LOAD_KICK_RESET;
		ev.threshold.cur = node->load;
		ev.threshold.ref = config.load_kick_threshold;
		goto out;
	}

	if (++ln->load_thr_count <= min_count) {
		if (ln->load_thr_count > 1)
			return;

		ev.type = UEV_LOAD_KICK_TRIGGER;
		ev.threshold.cur = node->load;
		ev.threshold.ref = config.load_kick_threshold;
		goto out;
	}

	ln->load_thr_count = 0;
	if (node->n_assoc < config.load_kick_min_clients) {
		ev.type = UEV_LOAD_KICK_MIN_CLIENTS;
		ev.threshold.cur = node->n_assoc;
		ev.threshold.ref = config.load_kick_min_clients;
		goto out;
	}

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		struct sta_info *tmp;

		if (si->connected != STA_CONNECTED)
			continue;

		if (is_more_kickable(kick1, si))
			kick1 = si;

		tmp = find_better_candidate(si, NULL, (1 << UEV_SELECT_REASON_LOAD));
		if (!tmp)
			continue;

		if (is_more_kickable(kick2, si)) {
			kick2 = si;
			candidate = tmp;
		}
	}

	if (!kick1) {
		ev.type = UEV_LOAD_KICK_NO_CLIENT;
		goto out;
	}

	if (kick2)
		kick1 = kick2;

	kick1->kick_count++;

	ev.type = UEV_LOAD_KICK_CLIENT;
	ev.si_cur = kick1;
	ev.si_other = candidate;
	ev.count = kick1->kick_count;

	usteer_ubus_kick_client(kick1);

out:
	usteer_event(&ev);
}
