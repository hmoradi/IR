/**@file Transaction.cc
 *
 * @date Jul 8, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <cstddef> /* NULL */
#include <cstring>
#include <cassert>
#include <strophe.h>

#include "Stanza.h"
#include "Transaction.h"

Transaction::Transaction() :
		conn(NULL), request(NULL), response(NULL), handler(NULL),
		handler_userdata(NULL) {
}

Transaction::Transaction(xmpp_conn_t* conn) :
		conn(conn), request(NULL), response(NULL), handler(NULL),
		handler_userdata(NULL) {
}

Transaction::~Transaction() {
	delete request;
	request = NULL;
	delete response;
	response = NULL;
}

Stanza* Transaction::getRequest() {
	return request;
}

Stanza* Transaction::getResponse() {
	return response;
}

transaction_handler_t Transaction::getHandler() {
	return handler;
}

void Transaction::setRequest(Stanza* request) {
	this->request = request->clone();
}

void Transaction::setResponse(Stanza* response) {
	this->response = response->clone();
}

void Transaction::setHandler(transaction_handler_t handler, void *userdata) {
	this->handler = handler;
	this->handler_userdata = userdata;
}

/**
 *
 * @return 0 upon success, true otherwise
 */
int Transaction::send() {
	/* the Transaction(conn) constructor must have been used */
	if(!conn) {
		// error out
		return 1;
	}

	if(request->isValid()) {
		char *id = request->getID();
		if(id == NULL) {
			int ret = request->setIDRandom();
			assert(ret == 0);
			id = request->getID();
		}
		assert(id && (strlen(id)>0));

		if(handler) {
			xmpp_id_handler_add(conn, xmpp_handler, id, (void *) this);
		}
		xmpp_send(conn, request->getStanza());
		return 0;
	}
	return 1;
}

/**
 * Send using specified connection \a conn
 * @param[in] conn The specified connection to use
 * @return 0 upon success, true otherwise
 */
int Transaction::send(xmpp_conn_t* conn) {
	/* the Transaction(conn) constructor must have been used */
	if(!conn) {
		// error out
		return 1;
	}

	if(request->isValid()) {
		char *id = request->getID();
		if(id == NULL) {
			int ret = request->setIDRandom();
			assert(ret == 0);
			id = request->getID();
		}
		assert(id && (strlen(id)>0));

		if(handler) {
			xmpp_id_handler_add(conn, xmpp_handler, id, (void *) this);
		}
		xmpp_send(conn, request->getStanza());
		return 0;
	}
	return 1;
}

void Transaction::fireEvent() {
	handler(this, handler_userdata);
}

int Transaction::xmpp_handler(	xmpp_conn_t* const conn,
								xmpp_stanza_t* const stanza,
								void* const userdata) {
	Transaction *trans = (Transaction *) userdata;
	Stanza *stanza_obj = new Stanza(stanza, true);

	/* sanity check that we are placing a
	 * response in a valid request - otherwise id mismatch? */
	assert(trans->getRequest()->isValid());
	trans->setResponse(stanza_obj);
	delete stanza_obj;
	trans->fireEvent();

	/* deregister handler */
	return 0;
}
