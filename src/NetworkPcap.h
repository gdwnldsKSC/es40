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
#ifndef INCLUDED_NETWORK_PCAP_H_
#define INCLUDED_NETWORK_PCAP_H_

#include "NetworkBackend.h"

#if defined(WIN32)
#define HAVE_REMOTE
#include <winsock.h>
#else
#include <pcap.h>
#endif

#if defined(WIN32)
typedef int          bpf_int32;
typedef unsigned int bpf_u_int32;

/*
 * The instruction data structure.
 */
struct bpf_insn {
	unsigned short code;
	unsigned char  jt;
	unsigned char  jf;
	bpf_u_int32    k;
};

/*
 * Structure for "pcap_compile()", "pcap_setfilter()", etc..
 */
struct bpf_program {
	unsigned int     bf_len;
	struct bpf_insn* bf_insns;
};

typedef struct pcap_if pcap_if_t;

#define PCAP_ERRBUF_SIZE 256

struct pcap_pkthdr {
	struct timeval ts;
	bpf_u_int32    caplen;
	bpf_u_int32    len;
};

struct pcap_if {
	struct pcap_if* next;
	char*           name;
	char*           description;
	void*           addresses;
	bpf_u_int32     flags;
};

struct pcap_send_queue {
	unsigned int maxlen; /* Maximum size of the queue, in bytes. This
	                        variable contains the size of the buffer field.  */
	unsigned int len;    /* Current size of the queue, in bytes.  */
	char *buffer;        /* Buffer containing the packets to be sent.  */
};

typedef struct pcap_send_queue pcap_send_queue;

typedef void (*pcap_handler)(unsigned char *user, const struct pcap_pkthdr *h, const unsigned char *bytes);

typedef struct pcap pcap_t;
typedef struct pcap_dumper pcap_dumper_t;
typedef struct pcap_if pcap_if_t;
typedef struct pcap_addr pcap_addr_t;
#endif

/**
 * \brief libpcap / npcap network backend.
 *
 * On Windows, wpcap.dll is loaded dynamically at runtime (from the Npcap
 * install directory) the first time a CNetworkPcap is constructed, mirroring
 * what CDEC21143 used to do directly.
 **/
class CNetworkPcap : public CNetworkBackend {
public:
	CNetworkPcap();
	virtual ~CNetworkPcap();

	virtual bool init(const char *devid_string, CConfigurator *cfg);
	virtual int  send(const u8 *data, int len);
	virtual int  receive(const u8 **data, int *len);
	virtual void set_filter(u8 mac_list[][6], int num_macs,
	                        bool promiscuous, bool receive_all,
	                        bool pass_multicast, bool inverse);
	virtual void close();

private:
	pcap_t*             fp;
	struct bpf_program  fcode;
	bool                opened;
};

#endif /* INCLUDED_NETWORK_PCAP_H_  */
