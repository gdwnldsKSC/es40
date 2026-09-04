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

#include "StdAfx.h"

#if defined(HAVE_VMNET)

#include "NetworkVmnetIPC.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <mach-o/dyld.h>
#include <signal.h>
#include <spawn.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

static const int socket_buffer_size = 1024 * 1024;
static const int helper_timeout_seconds = 10;

CNetworkVmnetIPC::CNetworkVmnetIPC():
	fd(-1),
	helper_pid(0) {
}

CNetworkVmnetIPC::~CNetworkVmnetIPC() { close(); }

static std::string helper_next_to_executable()
{
	char path[PATH_MAX];
	char resolved[PATH_MAX];
	uint32_t size = sizeof(path);

	if (_NSGetExecutablePath(path, &size) != 0 ||
		realpath(path, resolved) == nullptr)
		return std::string();
	char *slash = strrchr(resolved, '/');
	if (slash == nullptr)
		return std::string();
	*(slash + 1) = '\0';
	return std::string(resolved) + ES40_VMNET_HELPER_NAME;
}

static std::string find_helper()
{
	std::string adjacent = helper_next_to_executable();
	if (!adjacent.empty() && access(adjacent.c_str(), X_OK) == 0)
		return adjacent;
	if (geteuid() == 0)
		return std::string();
	return ES40_VMNET_HELPER_NAME;
}

static bool set_cloexec(int socket_fd)
{
	int flags = fcntl(socket_fd, F_GETFD, 0);
	return flags >= 0 &&
		fcntl(socket_fd, F_SETFD, flags | FD_CLOEXEC) == 0;
}

static bool prepare_socket(int socket_fd)
{
	int one = 1;
	int size = socket_buffer_size;
	struct timeval timeout;

	timeout.tv_sec = helper_timeout_seconds;
	timeout.tv_usec = 0;
	if (!set_cloexec(socket_fd) ||
		setsockopt(socket_fd, SOL_SOCKET, SO_NOSIGPIPE, &one,
			sizeof(one)) != 0 ||
		setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
			sizeof(timeout)) != 0)
		return false;

	/* Larger queues reduce drops, but the platform default still works. */
	setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
	setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
	return true;
}

bool CNetworkVmnetIPC::start(const char *devid_string, const char *adapter,
	struct es40_vmnet_open_reply *reply)
{
	int sockets[2] = { -1, -1 };

	if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0 ||
		!prepare_socket(sockets[0]) || !prepare_socket(sockets[1])) {
		printf("%s: Cannot create vmnet helper socket: %s\n",
			devid_string, strerror(errno));
		if (sockets[0] >= 0)
			::close(sockets[0]);
		if (sockets[1] >= 0)
			::close(sockets[1]);
		return false;
	}

	helper_path = find_helper();
	if (helper_path.empty()) {
		printf("%s: Cannot find %s next to the ES40 executable.\n",
			devid_string, ES40_VMNET_HELPER_NAME);
		::close(sockets[0]);
		::close(sockets[1]);
		return false;
	}

	int child_fd = fcntl(sockets[1], F_DUPFD_CLOEXEC, 10);
	if (child_fd < 0) {
		printf("%s: Cannot prepare vmnet helper socket: %s\n",
			devid_string, strerror(errno));
		::close(sockets[0]);
		::close(sockets[1]);
		return false;
	}

	posix_spawn_file_actions_t actions;
	int rc = posix_spawn_file_actions_init(&actions);
	bool actions_initialized = (rc == 0);
	if (rc == 0)
		rc = posix_spawn_file_actions_adddup2(&actions, child_fd,
			ES40_VMNET_HELPER_FD);
	if (rc == 0)
		rc = posix_spawn_file_actions_addclose(&actions, child_fd);

	pid_t pid = 0;
	char *const argv[] = { (char *)ES40_VMNET_HELPER_NAME, nullptr };
	if (rc == 0) {
		rc = posix_spawnp(&pid, helper_path.c_str(), &actions, nullptr,
			argv, environ);
	}
	if (actions_initialized)
		posix_spawn_file_actions_destroy(&actions);
	::close(child_fd);
	::close(sockets[1]);

	if (rc != 0) {
		printf("%s: Cannot start %s: %s\n", devid_string,
			helper_path.c_str(), strerror(rc));
		::close(sockets[0]);
		return false;
	}

	fd = sockets[0];
	helper_pid = pid;
	struct es40_vmnet_open_req request;
	memset(&request, 0, sizeof(request));
	request.msg.type = ES40_VMNET_MSG_OPEN;
	request.magic = ES40_VMNET_PROTO_MAGIC;
	request.version = ES40_VMNET_PROTO_VERSION;
	strlcpy(request.adapter, adapter, sizeof(request.adapter));

	ssize_t sent;
	do {
		sent = ::send(fd, &request, sizeof(request), 0);
	} while (sent < 0 && errno == EINTR);
	ssize_t received = -1;
	if (sent == (ssize_t)sizeof(request)) {
		struct iovec iov = { reply, sizeof(*reply) };
		struct msghdr message;
		memset(&message, 0, sizeof(message));
		message.msg_iov = &iov;
		message.msg_iovlen = 1;
		do {
			received = recvmsg(fd, &message, 0);
		} while (received < 0 && errno == EINTR);
		if (message.msg_flags & MSG_TRUNC)
			received = -1;
	}
	if (sent != (ssize_t)sizeof(request) ||
		received != (ssize_t)sizeof(*reply)) {
		printf("%s: %s did not answer the vmnet open request.\n",
			devid_string, helper_path.c_str());
		close();
		return false;
	}
	return true;
}

int CNetworkVmnetIPC::send_packet(const void *data, size_t len)
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
		sent = sendmsg(fd, &message, MSG_DONTWAIT);
	} while (sent < 0 && errno == EINTR);
	if (sent == (ssize_t)(sizeof(header) + len))
		return 0;
	if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
		errno == ENOBUFS))
		return 0;
	if (sent >= 0)
		errno = EIO;
	return -1;
}

int CNetworkVmnetIPC::receive(void *data, size_t capacity, int *len,
	struct es40_vmnet_status *status)
{
	struct es40_vmnet_msg header;
	struct iovec iov[2] = {
		{ &header, sizeof(header) },
		{ data, capacity }
	};
	struct msghdr message;
	memset(&message, 0, sizeof(message));
	message.msg_iov = iov;
	message.msg_iovlen = 2;

	ssize_t received;
	do {
		received = recvmsg(fd, &message, MSG_DONTWAIT);
	} while (received < 0 && errno == EINTR);
	if (received < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return 0;
		return -1;
	}
	if ((message.msg_flags & MSG_TRUNC) ||
		received < (ssize_t)sizeof(header)) {
		errno = EPROTO;
		return -1;
	}

	size_t payload_len = (size_t)received - sizeof(header);
	if (header.type == ES40_VMNET_MSG_PACKET) {
		if (payload_len == 0) {
			errno = EPROTO;
			return -1;
		}
		*len = (int)payload_len;
		return 1;
	}
	if (header.type == ES40_VMNET_MSG_STATUS &&
		payload_len == sizeof(*status) - sizeof(header)) {
		status->msg = header;
		memcpy((char *)status + sizeof(header), data, payload_len);
		return 2;
	}

	errno = EPROTO;
	return -1;
}

static bool reap(pid_t pid, int milliseconds)
{
	int status;
	int attempts = milliseconds / 50;

	for (int i = 0; i < attempts; i++) {
		pid_t result = waitpid(pid, &status, WNOHANG);
		if (result == pid || (result < 0 && errno == ECHILD))
			return true;
		if (result < 0 && errno != EINTR)
			return false;
		usleep(50000);
	}
	return false;
}

void CNetworkVmnetIPC::close()
{
	if (fd >= 0) {
		struct es40_vmnet_msg shutdown = { ES40_VMNET_MSG_SHUTDOWN };
		::send(fd, &shutdown, sizeof(shutdown), MSG_DONTWAIT);
		::close(fd);
		fd = -1;
	}
	if (helper_pid > 0) {
		if (!reap(helper_pid, 2000)) {
			kill(helper_pid, SIGTERM);
			if (!reap(helper_pid, 1000)) {
				kill(helper_pid, SIGKILL);
				reap(helper_pid, 1000);
			}
		}
		helper_pid = 0;
	}
}

#endif /* defined(HAVE_VMNET) */
