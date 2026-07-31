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

#include "StdAfx.h"

#if defined(HAVE_VMNET)

#include "NetworkVmnet.h"
#include "Configurator.h"

#include <ifaddrs.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>

CNetworkVmnet::CNetworkVmnet():
	vmnet_if(nullptr),
	dispatch_q(nullptr),
	dispatch_sem(nullptr),
	rx_buf(nullptr),
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
	__block bool success = false;
	xpc_object_t interface_names = vmnet_copy_shared_interface_list ();
        int n_interfaces, i;
        struct ifaddrs *ifap, *ifa;
	__block char *adapter = cfg->get_text_value ("adapter");
	char *interface;

	devid_for_log = devid_string;

        if (interface_names == nullptr) {
		printf ("%s: Cannot get list of vmnet capable interfaces.\n", devid_for_log);
		return false;
        } else
		n_interfaces = xpc_array_get_count (interface_names);

        if (getifaddrs (&ifap) < 0) {
		printf ("%s: Unable to obtain network interface addresses.\n", devid_for_log);
                xpc_release (interface_names);
                return false;
        }

        if (adapter && adapter[0]) {
		// Verify that vmnet can use this adapter
		for (i = 0; i < n_interfaces; i++) {
			interface = (char *) xpc_array_get_string (interface_names, i);
                        if (strncmp (adapter, interface, IFNAMSIZ) == 0)
                        	break;
                }
                if (n_interfaces == i) {
                	printf ("%s: Adapter %s cannot serve as a bridge.\n", devid_for_log, adapter);
                        freeifaddrs (ifap);
                        xpc_release (interface_names);
                        return false;
                }
                // Verify that the adapter has an Ethernet address
                ifa = ifap;
                while (ifa != NULL) {
                	if (strncmp (adapter, ifa->ifa_name, IFNAMSIZ) == 0) {
                        	if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET)
                                	break;
                        }
                }
                ifa = ifa->ifa_next;
                if (ifa == nullptr) {
                	printf ("%s: Adapter %s does not have an Ethernet address.\n", devid_for_log, adapter);
                        freeifaddrs (ifap);
                        xpc_release (interface_names);
                        return false;
                }
        }

        else {
        	// Use the first available interface with an Ethernet address
		for (i = 0; i < n_interfaces; i++) {
                	adapter = (char *) xpc_array_get_string (interface_names, i);
                        ifa = ifap;
                        while (ifa != NULL) {
                        	if (strncmp (adapter, ifa->ifa_name, IFNAMSIZ) == 0) {
                                	if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET) {
                                        	break;
                                        }
                                }
                                ifa = ifa->ifa_next;
                        }
                        if (ifa != NULL)
                        	break;
                }
                if (n_interfaces == i) {
                        freeifaddrs (ifap);
                        xpc_release (interface_names);
                	printf ("%s: No network interfaces available to serve as a bridge.\n", devid_for_log);
                        return false;
                }
                adapter = strndup (adapter, IFNAMSIZ);
        }

        freeifaddrs (ifap);
        xpc_release (interface_names);

	vmnet_interface_event_callback_t receive_handler = ^(interface_event_t event_mask, xpc_object_t event) {
        	if (event_mask & VMNET_INTERFACE_PACKETS_AVAILABLE) {
                  available_packets = xpc_dictionary_get_uint64 (event, vmnet_estimated_packets_available_key);
                }
        };

	vmnet_start_interface_completion_handler_t finish_init = ^(vmnet_return_t status,
                                                                   xpc_object_t __nullable interface_param) {
        	if (status == VMNET_SUCCESS) {
                	max_packet_size = xpc_dictionary_get_uint64 (interface_param, vmnet_max_packet_size_key);
                        rx_buf = malloc (max_packet_size);
                        status = vmnet_interface_set_event_callback (vmnet_if, VMNET_INTERFACE_PACKETS_AVAILABLE,
                                                                     dispatch_q, receive_handler);
                        if (status == VMNET_SUCCESS)
                        	success = true;
                        else
                        	printf ("%s: Cannot setup %s to received packets: %s\n", devid_for_log, adapter,
                                        strvmnetstatus (status));
                } else
                	printf ("%s: Unable to start vmnet on %s: %s\n", devid_for_log, adapter, strvmnetstatus (status));
                dispatch_semaphore_signal (dispatch_sem);
        };

        dispatch_q = dispatch_queue_create (devid_string, DISPATCH_QUEUE_SERIAL);
        dispatch_retain (dispatch_q);

        dispatch_sem = dispatch_semaphore_create (0);
        dispatch_retain (dispatch_sem);

        xpc_object_t interface_desc = xpc_dictionary_create (NULL, NULL, 0);
        xpc_dictionary_set_uint64 (interface_desc, vmnet_operation_mode_key, VMNET_BRIDGED_MODE);
	xpc_dictionary_set_string (interface_desc, vmnet_shared_interface_name_key, adapter);
	xpc_dictionary_set_bool (interface_desc, vmnet_allocate_mac_address_key, false);

	vmnet_if = vmnet_start_interface (interface_desc, dispatch_q, finish_init);
        dispatch_semaphore_wait (dispatch_sem, DISPATCH_TIME_FOREVER);

        // Drop privileges if allowed
	if (cfg->get_bool_value ("drop_privileges", true)) {
        	seteuid (getuid ());
                setegid (getgid ());
        }

	return success;
}

int CNetworkVmnet::send (const u8 *data, int len) {
	struct iovec iovec;
	struct vmpktdesc pktspec;
	int pktCount = 1;
	vmnet_return_t status;

        if (vmnet_if == nullptr)
        	return -1;

	iovec.iov_base = (void*)data;
	iovec.iov_len = len;
	pktspec.vm_pkt_size = len;
        pktspec.vm_pkt_iov = &iovec;
        pktspec.vm_pkt_iovcnt = 1;
        pktspec.vm_flags = 0;

        status = vmnet_write (vmnet_if, &pktspec, &pktCount);
        if (status != VMNET_SUCCESS) {
		printf ("%s: Error sending packet: %s\n", devid_for_log, strvmnetstatus (status));
		return -1;
        }

	return 0;
}

int CNetworkVmnet::receive (const u8 **data, int *len)
{
	struct iovec iovec;
	struct vmpktdesc pktspec;
	int pktCount = 1;
	vmnet_return_t status;

	if (vmnet_if == nullptr)
		return -1;

        if (available_packets == 0)
		return 0;       // No packets available

	iovec.iov_base = rx_buf;
        iovec.iov_len = max_packet_size;
        pktspec.vm_pkt_size = max_packet_size;
        pktspec.vm_pkt_iov = &iovec;
        pktspec.vm_pkt_iovcnt = 1;
        pktspec.vm_flags = 0;

	status = vmnet_read (vmnet_if, &pktspec, &pktCount);
	available_packets--;

	if (status == VMNET_SUCCESS) {
                *data = (const u8*)rx_buf;
		*len = (int)pktspec.vm_pkt_size;
		return 1;
	} else
		printf ("%s: Error receiving packet: %s\n", devid_for_log, strvmnetstatus (status));

	return -1;
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

void CNetworkVmnet::close ()
{
  vmnet_interface_completion_handler_t finish_cleanup = ^(vmnet_return_t status) {
    dispatch_release (dispatch_q);
    dispatch_q = nullptr;
    vmnet_if = nullptr;
  };
        if (vmnet_if != nullptr) {
          vmnet_interface_set_event_callback (vmnet_if, 0, NULL, NULL);
          vmnet_stop_interface (vmnet_if, dispatch_q, finish_cleanup);
        }

        if (dispatch_sem != nullptr) {
          dispatch_release (dispatch_sem);
          dispatch_q = nullptr;
        }

        if (rx_buf != nullptr) {
          free (rx_buf);
          rx_buf = nullptr;
        }

	return;
}

#endif /* defined(HAVE_VMNET)  */
