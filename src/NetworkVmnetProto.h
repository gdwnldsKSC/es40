/* ES40 emulator.
 *
 * WWW    : https://github.com/ES40-Emu/es40
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
 *
 * Although this is not required, the author would appreciate being notified of,
 * and receiving any modifications you may make to the source code that might serve
 * the general public.
 */

/**
 * \file
 * Private protocol between ES40 and its macOS vmnet helper.
 *
 * Each message is one UNIX-domain datagram.  Packet messages consist of an
 * es40_vmnet_msg header followed by one complete Ethernet frame.
 **/
#ifndef INCLUDED_NETWORK_VMNET_PROTO_H_
#define INCLUDED_NETWORK_VMNET_PROTO_H_

#include <net/if.h>
#include <stdint.h>

#define ES40_VMNET_HELPER_NAME    "es40_vmnet_helper"
#define ES40_VMNET_HELPER_FD      3
#define ES40_VMNET_PROTO_MAGIC    0x30345345u
#define ES40_VMNET_PROTO_VERSION  2u
#define ES40_VMNET_MAX_FRAME      65536u

enum es40_vmnet_msg_type {
	ES40_VMNET_MSG_OPEN = 1,
	ES40_VMNET_MSG_OPEN_REPLY = 2,
	ES40_VMNET_MSG_PACKET = 3,
	ES40_VMNET_MSG_STATUS = 4,
	ES40_VMNET_MSG_SHUTDOWN = 5
};

enum es40_vmnet_operation {
	ES40_VMNET_OP_READ = 1,
	ES40_VMNET_OP_WRITE = 2
};

#define ES40_VMNET_STATUS_OK            0u
#define ES40_VMNET_STATUS_HELPER_ERROR  0xffffffffu

struct es40_vmnet_msg {
	uint32_t type;
};

struct es40_vmnet_open_req {
	struct es40_vmnet_msg msg;
	uint32_t magic;
	uint32_t version;
	char     adapter[IFNAMSIZ];
};

struct es40_vmnet_open_reply {
	struct es40_vmnet_msg msg;
	uint32_t magic;
	uint32_t version;
	uint32_t status;
	uint32_t helper_euid;
	uint32_t max_packet_size;
	int32_t  system_error;
};

struct es40_vmnet_status {
	struct es40_vmnet_msg msg;
	uint32_t operation;
	uint32_t status;
};

#endif /* !defined(INCLUDED_NETWORK_VMNET_PROTO_H_) */
