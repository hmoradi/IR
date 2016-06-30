/**@file TransactionQueue_test.cc
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <iostream>
#include <strophe.h>
#include "Stanza.h"
#include "TransactionQueue.h"

using namespace std;

int main() {
	TransactionQueue tq;
	Transaction *t1, *t2;
	xmpp_ctx_t *ctx;
	xmpp_log_t *log;
	Stanza *one, *two, *three, *four;
	Transaction *seek; // used during retrieval

	log = xmpp_get_default_logger(XMPP_LEVEL_ERROR);
	ctx = xmpp_ctx_new(0, log);

	one = new Stanza(ctx);
	two = new Stanza(ctx);
	three = new Stanza(ctx);
	four = new Stanza(ctx);

	t1 = new Transaction();
	t2 = new Transaction();

	/* setup stanzas */
	one->setName("body");
	one->setID("bobby1");

	two->setName("body");
	two->setID("bobby2");

	three->setName("person");
	three->setAttribute("name", "craig");

	four->setName("person");
	four->setAttribute("name", "billy");

	one->addChild(three);
	two->addChild(four);

	/* set stanzas as requests for t1 and t2 */
	t1->setRequest(one);
	t2->setRequest(two);

	delete one;
	delete two;
	delete three;
	delete four;

	/* enqueue t1 and t2 */
	tq.enqueue(t1);
	tq.enqueue(t2);

	/* show size and exercise findByID */
	cout << "Size: " << tq.size() << endl;
	cout << "Find bobby2 id: " << tq.findByID("bobby2")->getRequest()->toText() << endl;
	cout << "Find bobby1 id: " << tq.findByID("bobby1")->getRequest()->toText() << endl;

	/* dequeue and show */
	cout << "Size: " << tq.size() << endl;
	seek = tq.dequeue();
	cout << "Dequeue: " << seek->getRequest()->toText() << endl;
	delete seek;

	/* dequeue and show*/
	cout << "Size: " << tq.size() << endl;
	seek = tq.dequeue();
	cout << "Dequeue: " << seek->getRequest()->toText() << endl;
	delete seek;

	cout << "Size: " << tq.size() << endl;

//	delete t1;
//	delete t2;

	xmpp_ctx_free(ctx);

	return 0;
}
