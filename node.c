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

void usteer_node_set_blob(struct blob_attr **dest, struct blob_attr *val)
{
	int new_len;
	int len;

	if (!val) {
		free(*dest);
		*dest = NULL;
		return;
	}

	len = *dest ? blob_pad_len(*dest) : 0;
	new_len = blob_pad_len(val);
	if (new_len != len)
		*dest = realloc(*dest, new_len);
	memcpy(*dest, val, new_len);
}
