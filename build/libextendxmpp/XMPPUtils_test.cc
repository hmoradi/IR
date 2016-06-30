/**@file XMPPUtils_test.cc
 *
 * @date Jul 30, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstddef>

#include "XMPPUtils.h"

using namespace std;

int main(int argc, char *argv[]) {
	if(argc != 2) {
		fprintf(stderr, "I need a jid argument\n");
		exit(EXIT_FAILURE);
	}

	XMPP::JID j(argv[1]);

	cout << "j.user = " << j.user << endl;
	cout << "j.domain = " << j.domain << endl;
	cout << "j.resource = " << j.resource << endl;
	cout << "j.full() = " << j.full() << endl;
	cout << "j.bare() = " << j.bare() << endl;
	cout << "j.pubsub() = " << j.pubsub() << endl;

	exit(EXIT_SUCCESS);
}
