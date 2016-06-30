/**@file Transaction.h
 *
 * @date Jul 8, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#ifndef TRANSACTION_H_
#define TRANSACTION_H_

#include <strophe.h>
#include "Stanza.h"

class Transaction;
typedef void (*transaction_handler_t)(Transaction* trans, void* userdata);
/**
 * @note May want to revise the procedure for deleting a
 *       transaction in a transaction handler. It could
 *       be that we make the handler return an int like the
 *       xmpp handlers to indicate we are done with the
 *       transcription. Alternatively, we could add a copy
 *       function to the Transcription class to allow duplicating
 *       the Transcription for further use outside the handler.
 */
class Transaction {
public:
	Transaction();
	Transaction(xmpp_conn_t *conn);
	virtual ~Transaction();
	Stanza* getRequest();
	Stanza* getResponse();
	transaction_handler_t getHandler();
	void setRequest(Stanza* request);
	void setResponse(Stanza* response);
	void setHandler(transaction_handler_t handler, void* userdata);
	int send();
	int send(xmpp_conn_t* conn);
	void fireEvent();
protected:
	xmpp_conn_t* conn;
	Stanza* request;
	Stanza* response;
	transaction_handler_t handler;
	void* handler_userdata;
	static int xmpp_handler(xmpp_conn_t * const conn,
							xmpp_stanza_t * const stanza,
							void * const userdata);
};

#endif /* TRANSACTION_H_ */
