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

#ifndef INCLUDED_NETWORK_VMNET_IPC_H_
#define INCLUDED_NETWORK_VMNET_IPC_H_

#include "NetworkVmnetProto.h"

#include <stddef.h>
#include <sys/types.h>

#include <string>

class CNetworkVmnetIPC {
public:
	CNetworkVmnetIPC();
	~CNetworkVmnetIPC();

	bool start(const char *devid_string, const char *adapter,
		struct es40_vmnet_open_reply *reply);
	int send_packet(const void *data, size_t len);
	int receive(void *data, size_t capacity, int *len,
		struct es40_vmnet_status *status);
	const char *get_helper_path() const { return helper_path.c_str(); }
	void close();

private:
	int fd;
	pid_t helper_pid;
	std::string helper_path;
};

#endif /* !defined(INCLUDED_NETWORK_VMNET_IPC_H_) */
