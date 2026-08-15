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
 * TAP device network backend.
 *
 * Host support:
 *  - Linux: Opens the shared clone device /dev/net/tun and binds
 *           it to a named tap interface via the TUNSETIFF ioctl.
 *  - *BSD:  A TAP device is a character device node (usually
 *           /dev/tapN).
 **/
#ifndef INCLUDED_NETWORK_TAP_H_
#define INCLUDED_NETWORK_TAP_H_

#include "NetworkBackend.h"

#if defined(HAVE_TAP_NET)

#include <net/if.h>

class CNetworkTap: public CNetworkBackend {
public:
	CNetworkTap();
	virtual ~CNetworkTap();

	virtual bool init(const char *devid_string, CConfigurator *cfg);
	virtual int  send(const u8 *data, int len);
	virtual int  receive(const u8 **data, int *len);
	virtual void set_filter(u8 mac_list[][6], int num_macs,
	                        bool promiscuous, bool receive_all,
	                        bool pass_multicast, bool inverse);
	virtual void close();
	virtual int  get_wait_fd() { return tap_fd; }

private:
	int  tap_fd;
	char ifname[IFNAMSIZ];
	unsigned char rx_buf[2048]; /* Max ethernet frame, rounded up.  */

	const char *devid_for_log;

	// --- device open/create: platform-specific implementation lives in
	//     NetworkTap.cpp, split by #if defined(__linux__) / BSD. ---
	bool tap_open(const char *devid_string, const char *name);
};

#endif /* defined(HAVE_TAP_NET)  */

#endif /* !defined(INCLUDED_NETWORK_TAP_H_)  */
