/**@file XMPPUtils.cc
 *
 * @date Jul 30, 2015
 * @author Craig Hesling <craig@hesling.com>
 */


#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cstring>
#include <cassert>

#include "XMPPUtils.h"

using namespace std;

namespace XMPP {

JID::JID() {
}

JID::JID(string jid) {
	decompose(jid);
}

JID::JID(const char* jid) {
	decompose(jid);
}

/**
 * A beautiful parser that can decipher any jid input without errors.
 * @param jid The string JID to parse.
 * @note We ignore multiple occurrences of @ and /.
 *       Additional occurrences may be grouped into the resource and domain.
 */
void JID::decompose(string jid) {
	decompose(jid.c_str());
}

/**
 * A beautiful parser that can decipher any jid input without errors.
 * @param jid The string JID to parse.
 * @note We ignore multiple occurrences of @ and /.
 *       Additional occurrences may be grouped into the resource and domain.
 */
void JID::decompose(const char *jid) {
	char *jid_copy;
	char *cursor;
	char *at, *res;
	assert(jid);

	jid_copy = strdup(jid);
	if(!jid_copy) {
		perror("Could not make local copy of jid\n");
	}

	/* blank previous parts */
	user.clear();
	domain.clear();
	resource.clear();

	at  = strchr(jid_copy, '@');
	res = strchr(jid_copy, '/');

	if (at && res && (res < at)) {
		/* too malformed to use */
		goto cleanup;
	}

	cursor = jid_copy;

	/* try to grab user portion */
	if(at == NULL) {
		/* no user part to grab */
	} else {
		*at = '\0';
		user.assign(cursor);
		cursor = at+1; //could be null terminator
	}

	/* try to grab domain portion */
	if(res == NULL) {
		/* may assign a blank string */
		domain.assign(cursor);
		goto cleanup;
	} else {
		*res = '\0';
		domain.assign(cursor);
		cursor = res+1; //could be null terminator
	}

	/* try to grab resource */
	resource.assign(cursor);

cleanup:
	free(jid_copy);
	return ;
}

/**
 * Tries to combine user, domain, and resource.
 * This may generate weird results if domain and user are not specified.
 * @return The full JID string
 */
std::string JID::full() {
	std::string u = (user.empty()?"":(user+'@'));
	std::string d = (domain.empty()?"":domain);
	std::string r = (resource.empty()?"":("/"+resource));
	return u+d+r;
}

/**
 * Tries to combine user and domain.
 * This may generate weird results if domain and user are not specified.
 * @return The bare JID string
 */
std::string JID::bare() {
	std::string u = (user.empty()?"":(user+'@'));
	std::string d = (domain.empty()?"":domain);
	return u+d;
}

/**
 * Tries to generate the local PubSub JID.
 * @return The pubsub JID string
 */
std::string JID::pubsub() {
	return (domain.empty()?"":("pubsub."+domain));
}

/**
 * We guess what the local PubSub JID would be from some supplied JID.
 * @param jid The jid that we will derive the pubsub jid from
 * @return The PubSub JID string
 */
string  pubsub_jid(string jid) {
	JID j(jid);
	return j.pubsub();
}

}
