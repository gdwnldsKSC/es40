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
 *
 */

#if defined(HAVE_VMNET)

/**
 * \file
 * Privileged packet service for the macOS vmnet framework.
 *
 * ES40 retains configuration, adapter selection, diagnostics, filtering,
 * and process management.  This process only owns the opaque vmnet handle.
 **/

#include <dispatch/dispatch.h>
#include <vmnet/vmnet.h>
#include <xpc/xpc.h>

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include "NetworkVmnetProto.h"

static int socket_fd = ES40_VMNET_HELPER_FD;
static interface_ref vmnet_if = nullptr;
static dispatch_queue_t dispatch_q = nullptr;
static uint64_t max_packet_size = 0;
static void *rx_buf = nullptr;
static uid_t initial_euid = 0;
static bool forwarding = false;

static bool check_socket()
{
	struct sockaddr_storage address;
	socklen_t address_len = sizeof(address);
	int type = 0;
	socklen_t type_len = sizeof(type);

	if (getsockopt(socket_fd, SOL_SOCKET, SO_TYPE, &type, &type_len) != 0 ||
		type != SOCK_DGRAM)
		return false;
	if (getpeername(socket_fd, (struct sockaddr *)&address,
		&address_len) != 0)
		return false;
	return address.ss_family == AF_UNIX;
}

static bool send_reply(uint32_t status, uint32_t max_packet,
	int system_error)
{
	struct es40_vmnet_open_reply reply;

	memset(&reply, 0, sizeof(reply));
	reply.msg.type = ES40_VMNET_MSG_OPEN_REPLY;
	reply.magic = ES40_VMNET_PROTO_MAGIC;
	reply.version = ES40_VMNET_PROTO_VERSION;
	reply.status = status;
	reply.helper_euid = (uint32_t)initial_euid;
	reply.max_packet_size = max_packet;
	reply.system_error = system_error;
	ssize_t sent;
	do {
		sent = send(socket_fd, &reply, sizeof(reply), MSG_DONTWAIT);
	} while (sent < 0 && errno == EINTR);
	return sent == (ssize_t)sizeof(reply);
}

static void send_status(uint32_t operation, vmnet_return_t status)
{
	struct es40_vmnet_status message;

	message.msg.type = ES40_VMNET_MSG_STATUS;
	message.operation = operation;
	message.status = (uint32_t)status;
	ssize_t sent;
	do {
		sent = send(socket_fd, &message, sizeof(message), MSG_DONTWAIT);
	} while (sent < 0 && errno == EINTR);
}

static vmnet_return_t start_vmnet(const char *adapter)
{
	dispatch_semaphore_t started;
	xpc_object_t description;
	__block vmnet_return_t status = VMNET_FAILURE;

	dispatch_q = dispatch_queue_create("es40.vmnet",
		DISPATCH_QUEUE_SERIAL);
	if (dispatch_q == nullptr)
		return VMNET_MEM_FAILURE;
	started = dispatch_semaphore_create(0);
	if (started == nullptr)
		return VMNET_MEM_FAILURE;
	description = xpc_dictionary_create(NULL, NULL, 0);
	if (description == nullptr) {
		dispatch_release(started);
		return VMNET_MEM_FAILURE;
	}

	xpc_dictionary_set_uint64(description, vmnet_operation_mode_key,
		VMNET_BRIDGED_MODE);
	xpc_dictionary_set_string(description, vmnet_shared_interface_name_key,
		adapter);
	xpc_dictionary_set_bool(description, vmnet_allocate_mac_address_key,
		false);
	vmnet_if = vmnet_start_interface(description, dispatch_q,
		^(vmnet_return_t result, xpc_object_t __nullable parameters) {
			status = result;
			if (result == VMNET_SUCCESS) {
				max_packet_size = xpc_dictionary_get_uint64(parameters,
					vmnet_max_packet_size_key);
			}
			dispatch_semaphore_signal(started);
		});
	xpc_release(description);

	/* A null interface does not run the completion block. */
	if (vmnet_if != nullptr)
		dispatch_semaphore_wait(started, DISPATCH_TIME_FOREVER);
	dispatch_release(started);
	return status;
}

static bool stop_vmnet()
{
	bool stopped_cleanly = true;

	if (dispatch_q != nullptr) {
		dispatch_sync(dispatch_q, ^{
			forwarding = false;
		});
	}
	if (vmnet_if != nullptr) {
		dispatch_semaphore_t stopped = dispatch_semaphore_create(0);
		vmnet_interface_set_event_callback(vmnet_if, 0, NULL, NULL);
		dispatch_sync(dispatch_q, ^{});
		if (stopped == nullptr)
			return false;
		vmnet_return_t status = vmnet_stop_interface(vmnet_if, dispatch_q,
			^(vmnet_return_t result) {
				(void)result;
				dispatch_semaphore_signal(stopped);
			});
		if (status == VMNET_SUCCESS) {
			dispatch_semaphore_wait(stopped, DISPATCH_TIME_FOREVER);
			vmnet_if = nullptr;
		} else {
			stopped_cleanly = false;
		}
		dispatch_release(stopped);
	}
	if (dispatch_q != nullptr && stopped_cleanly) {
		dispatch_release(dispatch_q);
		dispatch_q = nullptr;
	}
	return stopped_cleanly;
}

static bool drop_privileges(int *saved_error)
{
	uid_t uid = getuid();
	gid_t gid = getgid();

	if (setgid(gid) != 0 || setuid(uid) != 0) {
		*saved_error = errno;
		return false;
	}
	if (geteuid() != uid || getegid() != gid) {
		*saved_error = EPERM;
		return false;
	}
	return true;
}

static bool send_packet(const void *data, size_t len)
{
	struct es40_vmnet_msg header = { ES40_VMNET_MSG_PACKET };
	struct iovec iov[2] = {
		{ &header, sizeof(header) },
		{ (void *)data, len }
	};
	struct msghdr message;
	memset(&message, 0, sizeof(message));
	message.msg_iov = iov;
	message.msg_iovlen = 2;

	ssize_t sent;
	do {
		sent = sendmsg(socket_fd, &message, MSG_DONTWAIT);
	} while (sent < 0 && errno == EINTR);
	if (sent == (ssize_t)(sizeof(header) + len))
		return true;
	return sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
		errno == ENOBUFS);
}

static void forward_packets(uint64_t estimated)
{
	if (!forwarding)
		return;
	(void)estimated;
	uint64_t budget = 256;

	while (budget-- > 0) {
		struct iovec iov = { rx_buf, (size_t)max_packet_size };
		struct vmpktdesc packet;
		int count = 1;

		packet.vm_pkt_size = max_packet_size;
		packet.vm_pkt_iov = &iov;
		packet.vm_pkt_iovcnt = 1;
		packet.vm_flags = 0;
		vmnet_return_t status = vmnet_read(vmnet_if, &packet, &count);
		if (status != VMNET_SUCCESS) {
			send_status(ES40_VMNET_OP_READ, status);
			return;
		}
		if (count == 0)
			return;
		if (packet.vm_pkt_size > max_packet_size ||
			!send_packet(rx_buf, (size_t)packet.vm_pkt_size))
			return;
	}

	/* Yield to queued control work, then continue draining the interface. */
	dispatch_async(dispatch_q, ^{
		forward_packets(0);
	});
}

static void pump_packets(void *tx_buf, pid_t parent_pid)
{
	struct pollfd poll_fd;
	poll_fd.fd = socket_fd;
	poll_fd.events = POLLIN;

	for (;;) {
		int ready = poll(&poll_fd, 1, 1000);
		if (ready < 0) {
			if (errno == EINTR)
				continue;
			return;
		}
		if (ready == 0) {
			if (getppid() != parent_pid)
				return;
			continue;
		}
		if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL))
			return;

		for (;;) {
			struct es40_vmnet_msg header;
			struct iovec iov[2] = {
				{ &header, sizeof(header) },
				{ tx_buf, (size_t)max_packet_size }
			};
			struct msghdr message;
			memset(&message, 0, sizeof(message));
			message.msg_iov = iov;
			message.msg_iovlen = 2;

			ssize_t received = recvmsg(socket_fd, &message, MSG_DONTWAIT);
			if (received < 0) {
				if (errno == EINTR)
					continue;
				if (errno == EAGAIN || errno == EWOULDBLOCK)
					break;
				return;
			}
			if (received < (ssize_t)sizeof(header))
				return;
			if (header.type == ES40_VMNET_MSG_SHUTDOWN &&
				received == (ssize_t)sizeof(header))
				return;
			if ((message.msg_flags & MSG_TRUNC) ||
				header.type != ES40_VMNET_MSG_PACKET ||
				received <= (ssize_t)sizeof(header))
				return;

			size_t len = (size_t)received - sizeof(header);
			struct iovec packet_iov = { tx_buf, len };
			struct vmpktdesc packet;
			int count = 1;

			packet.vm_pkt_size = len;
			packet.vm_pkt_iov = &packet_iov;
			packet.vm_pkt_iovcnt = 1;
			packet.vm_flags = 0;
			vmnet_return_t status = vmnet_write(vmnet_if, &packet,
				&count);
			if (status != VMNET_SUCCESS)
				send_status(ES40_VMNET_OP_WRITE, status);
			else if (count == 0)
				send_status(ES40_VMNET_OP_WRITE,
					VMNET_BUFFER_EXHAUSTED);
		}
	}
}

int main()
{
	struct es40_vmnet_open_req request;
	vmnet_return_t status;
	void *tx_buf = nullptr;
	int saved_error = 0;
	pid_t parent_pid = getppid();

	initial_euid = geteuid();
	if (!check_socket()) {
		fprintf(stderr, ES40_VMNET_HELPER_NAME
			": must be started by the es40 emulator\n");
		return 2;
	}

	long max_fd = sysconf(_SC_OPEN_MAX);
	for (int fd = ES40_VMNET_HELPER_FD + 1;
		max_fd > 0 && fd < max_fd; fd++)
		close(fd);
	signal(SIGPIPE, SIG_IGN);

	struct timeval timeout = { 10, 0 };
	setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
		sizeof(timeout));
	struct iovec request_iov = { &request, sizeof(request) };
	struct msghdr request_message;
	memset(&request_message, 0, sizeof(request_message));
	request_message.msg_iov = &request_iov;
	request_message.msg_iovlen = 1;
	ssize_t received;
	do {
		received = recvmsg(socket_fd, &request_message, 0);
	} while (received < 0 && errno == EINTR);
	if (received != (ssize_t)sizeof(request) ||
		(request_message.msg_flags & MSG_TRUNC))
		return 1;
	if (request.msg.type != ES40_VMNET_MSG_OPEN ||
		request.magic != ES40_VMNET_PROTO_MAGIC ||
		request.version != ES40_VMNET_PROTO_VERSION) {
		send_reply(ES40_VMNET_STATUS_HELPER_ERROR, 0, EPROTO);
		return 1;
	}
	request.adapter[IFNAMSIZ - 1] = '\0';
	if (request.adapter[0] == '\0') {
		send_reply(ES40_VMNET_STATUS_HELPER_ERROR, 0, EINVAL);
		return 1;
	}

	status = start_vmnet(request.adapter);
	if (status != VMNET_SUCCESS) {
		send_reply((uint32_t)status, 0, 0);
		stop_vmnet();
		return 1;
	}
	if (max_packet_size == 0 ||
		max_packet_size > ES40_VMNET_MAX_FRAME) {
		send_reply(ES40_VMNET_STATUS_HELPER_ERROR, 0, EOVERFLOW);
		stop_vmnet();
		return 1;
	}
	rx_buf = malloc((size_t)max_packet_size);
	tx_buf = malloc((size_t)max_packet_size);
	if (rx_buf == nullptr || tx_buf == nullptr) {
		send_reply(ES40_VMNET_STATUS_HELPER_ERROR, 0, ENOMEM);
		stop_vmnet();
		free(rx_buf);
		free(tx_buf);
		return 1;
	}

	status = vmnet_interface_set_event_callback(vmnet_if,
		VMNET_INTERFACE_PACKETS_AVAILABLE, dispatch_q,
		^(interface_event_t events, xpc_object_t event) {
			if (events & VMNET_INTERFACE_PACKETS_AVAILABLE) {
				forward_packets(xpc_dictionary_get_uint64(event,
					vmnet_estimated_packets_available_key));
			}
		});
	if (status != VMNET_SUCCESS) {
		send_reply((uint32_t)status, 0, 0);
		stop_vmnet();
		free(rx_buf);
		free(tx_buf);
		return 1;
	}

	/* Complete all privileged vmnet setup before permanently dropping root. */
	if (!drop_privileges(&saved_error)) {
		send_reply(ES40_VMNET_STATUS_HELPER_ERROR, 0, saved_error);
		if (stop_vmnet()) {
			free(rx_buf);
			free(tx_buf);
		}
		return 1;
	}

	if (send_reply(ES40_VMNET_STATUS_OK,
		(uint32_t)max_packet_size, 0)) {
		dispatch_sync(dispatch_q, ^{
			forwarding = true;
			forward_packets(1);
		});
		pump_packets(tx_buf, parent_pid);
	}

	if (stop_vmnet()) {
		free(rx_buf);
		free(tx_buf);
	}
	return 0;
}

#endif /* defined(HAVE_VMNET) */
