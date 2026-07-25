/* ES40 emulator.
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

#if defined(HAVE_PCAP) || defined(HAVE_TAP_NET)

#include "NetworkBackend.h"
#include "Configurator.h"

#if defined(HAVE_PCAP)
#include "NetworkPcap.h"
#endif

#if defined(HAVE_TAP_NET)
#include "NetworkTap.h"
#endif

CNetworkBackend* create_network_backend(const char *devid_string,
                                        CConfigurator *cfg)
{
	char *type = cfg->get_text_value("type");

	if (type && !strcasecmp(type, "tap")) {
#if defined(HAVE_TAP_NET)
		return new CNetworkTap();
#endif
		printf ("%s: \"tap\" support not compiled in.");
		return nullptr;
	}

	if (type && !strcasecmp(type, "pcap")) {
#if defined(HAVE_PCAP)
		return new CNetworkPcap();
#else
		printf ("%s: \"pcap\" support not compiled in.");
		return nullptr;
#endif
	}

	if (type)
		printf("%s: Unknown network backend type \"%s\"; expected \"pcap\" or "
		       "\"tap\".\n", devid_string, type);

	return nullptr;

}

#endif /* defined(HAVE_PCAP) || defined(HAVE_TAP_NET)  */
