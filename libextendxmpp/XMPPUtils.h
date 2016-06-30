/**@file XMPPUtils.h
 *
 * @date Jul 30, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#ifndef XMPPUTILS_H_
#define XMPPUTILS_H_

#include <string>

namespace XMPP {

struct JID {
	std::string user;
	std::string domain;
	std::string resource;

	JID();
	JID(std::string jid);
	JID(const char *jid);
	void decompose(std::string jid);
	void decompose(const char *jid);
	std::string full();
	std::string bare();
	std::string pubsub();
};


std::string pubsub_jid(std::string jid);

}

#endif /* XMPPUTILS_H_ */
