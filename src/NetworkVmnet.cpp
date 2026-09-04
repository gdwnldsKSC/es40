/* ES40 Emulator.
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
 *
 */

#include "StdAfx.h"

#if defined(HAVE_VMNET)

#include "NetworkVmnet.h"
#include "NetworkVmnetIPC.h"
#include "Configurator.h"

#include <vmnet/vmnet.h>
#include <xpc/xpc.h>

#include <errno.h>
#include <string.h>

CNetworkVmnet::CNetworkVmnet():
	ipc(nullptr),
	max_packet_size(0),
	rx_buf(nullptr),
	dead(false),
	devid_for_log(nullptr) {
}

CNetworkVmnet::~CNetworkVmnet() { close(); }

const char *strvmnetstatus (vmnet_return_t status) {
	switch (status) {
        case VMNET_SUCCESS:
        	return "Successfully completed.";
        case VMNET_FAILURE:
        	return "General failure.";
        case VMNET_MEM_FAILURE:
        	return "Memory allocation failure.";
        case VMNET_INVALID_ARGUMENT:
        	return "Invalid argument specified.";
        case VMNET_SETUP_INCOMPLETE:
        	return "Interface setup is not complete.";
        case VMNET_INVALID_ACCESS:
        	return "Permission denied.";
        case VMNET_PACKET_TOO_BIG:
        	return "Packet size larger than MTU.";
        case VMNET_BUFFER_EXHAUSTED:
        	return "Buffers exhausted in kernel.";
        case VMNET_TOO_MANY_PACKETS:
        	return "Packet count exceeds limit.";
        case VMNET_SHARING_SERVICE_BUSY:
        	return "vmnet interface cannot be started as conflicting sharing service is in use.";
        case VMNET_NOT_AUTHORIZED:
        	return "The operation could not be completed due to missing authorization.";
        default:
        	return "Unknown status code";
        }
}

bool CNetworkVmnet::init(const char *devid_string, CConfigurator *cfg)
{
	bool success;
	xpc_object_t interface_names = vmnet_copy_shared_interface_list ();
        int n_interfaces, i;
	char *adapter = cfg->get_text_value ("adapter");
	char *interface;
	char selected_adapter[IFNAMSIZ];
	struct es40_vmnet_open_reply reply;

	devid_for_log = devid_string;
	dead = false;

        if (interface_names == nullptr) {
		printf ("%s: Cannot get list of vmnet capable interfaces.\n", devid_for_log);
		return false;
        } else
		n_interfaces = xpc_array_get_count (interface_names);

        if (adapter && adapter[0]) {
		// Verify that vmnet can use this adapter
		for (i = 0; i < n_interfaces; i++) {
			interface = (char *) xpc_array_get_string (interface_names, i);
                        if (strncmp (adapter, interface, IFNAMSIZ) == 0)
                        	break;
                }
                if (n_interfaces == i) {
                	printf ("%s: Adapter %s cannot serve as a bridge.\n", devid_for_log, adapter);
                        xpc_release (interface_names);
                        return false;
                }
        }

        else {
        	// Use the first vmnet capable interface
                if (n_interfaces == 0) {
                        xpc_release (interface_names);
                	printf ("%s: No network interfaces available to serve as a bridge.\n", devid_for_log);
                        return false;
                }
                adapter = (char *) xpc_array_get_string (interface_names, 0);
        }

	strlcpy (selected_adapter, adapter, sizeof(selected_adapter));
	xpc_release (interface_names);

	ipc = new CNetworkVmnetIPC();
	memset(&reply, 0, sizeof(reply));
	success = ipc->start(devid_string, selected_adapter, &reply);

        // Drop privileges if allowed
	if (cfg->get_bool_value ("drop_privileges", true)) {
		seteuid (getuid ());
		setegid (getgid ());
	}

	if (!success) {
		close();
		return false;
	}
	if (reply.msg.type != ES40_VMNET_MSG_OPEN_REPLY ||
		reply.magic != ES40_VMNET_PROTO_MAGIC ||
		reply.version != ES40_VMNET_PROTO_VERSION) {
		printf("%s: %s uses an incompatible vmnet protocol.\n",
			devid_for_log, ipc->get_helper_path());
		close();
		return false;
	}
	if (reply.status != ES40_VMNET_STATUS_OK) {
		if (reply.status == ES40_VMNET_STATUS_HELPER_ERROR) {
			printf("%s: vmnet helper failed: %s\n", devid_for_log,
				strerror(reply.system_error));
		} else {
			printf("%s: Unable to start vmnet on %s: %s\n",
				devid_for_log, selected_adapter,
				strvmnetstatus((vmnet_return_t)reply.status));
		}
		if (reply.helper_euid != 0) {
			printf("%s: %s must be installed setuid root.\n",
				devid_for_log, ipc->get_helper_path());
		}
		close();
		return false;
	}
	if (reply.max_packet_size == 0 ||
		reply.max_packet_size > ES40_VMNET_MAX_FRAME) {
		printf("%s: vmnet reported an invalid maximum packet size (%u).\n",
			devid_for_log, reply.max_packet_size);
		close();
		return false;
	}

	max_packet_size = reply.max_packet_size;
	rx_buf = malloc(max_packet_size);
	if (rx_buf == nullptr) {
		printf("%s: Cannot allocate vmnet receive buffer.\n",
			devid_for_log);
		close();
		return false;
	}

	return true;
}

void CNetworkVmnet::report_dead(const char *why)
{
	if (!dead)
		printf("%s: vmnet helper connection lost: %s\n",
			devid_for_log, why);
	dead = true;
}

int CNetworkVmnet::send(const u8 *data, int len)
{
	if (ipc == nullptr || dead)
		return -1;
	if (len <= 0 || (uint64_t)len > max_packet_size)
		return -1;
	if (ipc->send_packet(data, (size_t)len) == 0)
		return 0;
	report_dead(strerror(errno));
	return -1;
}

int CNetworkVmnet::receive(const u8 **data, int *len)
{
	if (ipc == nullptr || dead)
		return -1;

	for (;;) {
		struct es40_vmnet_status status;
		int result = ipc->receive(rx_buf, max_packet_size, len, &status);
		if (result == 1) {
			*data = (const u8 *)rx_buf;
			return 1;
		}
		if (result == 0)
			return 0;
		if (result < 0) {
			report_dead(strerror(errno));
			return -1;
		}

		const char *operation = status.operation == ES40_VMNET_OP_READ ?
			"receiving" : "sending";
		printf("%s: Error %s packet: %s\n", devid_for_log, operation,
			strvmnetstatus((vmnet_return_t)status.status));
	}
}

void CNetworkVmnet::set_filter (u8 mac_list[][6], int num_macs,
                                bool promiscuous, bool receive_all,
                                bool pass_multicast, bool inverse)
{
	// The vmnet framework only delivers packets intended for this interface.
	// The emulated NIC's own setup-frame handling still does perfect/hash
	// filtering in software for anything delivered here.
	(void) mac_list;
	(void) num_macs;
	(void) promiscuous;
	(void) receive_all;
	(void) pass_multicast;
	(void) inverse;

	return;
}

void CNetworkVmnet::close()
{
	if (ipc != nullptr) {
		delete ipc;
		ipc = nullptr;
	}
	if (rx_buf != nullptr) {
		free(rx_buf);
		rx_buf = nullptr;
	}
	max_packet_size = 0;
	dead = false;
}

#endif /* defined(HAVE_VMNET)  */
