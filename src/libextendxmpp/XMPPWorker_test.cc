/**@file XMPPWorker_test.cc
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <iostream>
#include <unistd.h>
#include "Transaction.h"
#include "XMPPWorker.h"

#define XMPP_JID "core@xmpp.boschsc.ddns.net"
#define XMPP_PASS "boschsc"

using namespace std;

int main() {
	XMPPWorker xk(XMPP_JID, XMPP_PASS);
	Stanza *msg, *body, *body_text;
	Transaction *trans = new Transaction();

	msg = new Stanza(xk.getCTX());
	body = new Stanza(xk.getCTX());
	body_text = new Stanza(xk.getCTX());

	/* <message type="chat" to="core2@192.168.1.108"/> */
	msg->setName("message");
	msg->setType("chat");
	msg->setAttribute("to", "core2@192.168.1.108");

	/* <body/> */
	body->setName("body");

	/* <body/>'s text */
	body_text->setText("Hello core2!");

	/* build stanza tree */
	body->addChild(body_text);
	msg->addChild(body);

	/* build transaction */
	trans->setRequest(msg);

	delete msg;
	delete body;
	delete body_text;

	/* send */
	xk.send(trans);

	xk.start();
	if(xk.isRunning() == false) {
		cerr << "Error - Connection dropped!" << endl;
	}
	/* stop before being able to send */
	xk.stop();

	/* we assume here that we stopped the worker
	 * thread before it could send. This is just
	 * for testing.
	 */

	xk.start();
	while (xk.getPending()) {
		if(xk.isRunning() == false) {
			cerr << "Error - Connection dropped!" << endl;
			break;
		}
	}
	xk.stop();


	return 0;
}
