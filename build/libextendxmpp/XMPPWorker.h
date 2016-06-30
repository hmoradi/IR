/**@file XMPPWorker.h
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#ifndef XMPPWORKER_H_
#define XMPPWORKER_H_

#include <pthread.h>
#include "Stanza.h"
#include "TransactionQueue.h"
#include "Transaction.h"

/**
 * @note The methods for managing the thread in this class, with
 *       exception of \ref send, are not, themselves, thread-safe.
 */
class XMPPWorker {
public:
	XMPPWorker(	const char *jid = 0,
				const char *pass = 0,
				unsigned short altport = 0,
				const char * const altdomain = 0,
				const char *node = NULL,
				const char *parent_node = NULL,
				bool enable_actuation = false,
				bool enable_distributed_coordination = false
				);
	virtual ~XMPPWorker();
	void setJID(const char *jid);
	void setPass(const char *pass);
	void setAltPort(unsigned short port);
	void setAltDomain(const char *domain);
	void setXMPPLogLevel(xmpp_log_level_t log_level);
	xmpp_ctx_t *getCTX();
	bool isRunning();
	size_t getPending();
	int start();
	int stop();
	void send(Transaction *trans);
	int get_actuation_count();
	bool isActuationEnabled();
	bool isDistributedCoordinationEnabled();
private:
	/* XMPP Session */
	const char *jid;
	const char *pass;
	unsigned short altport;
	const char *altdomain;
	bool enable_actuation;
	bool enable_distributed_coordination;
	const char *node;
	const char *parent_node;
	char *actuation_node;
	int actuation_count;
	char previous_timestamp[25];
	xmpp_log_level_t xmpp_log_level;
	xmpp_log_t *logg;
	xmpp_ctx_t *ctx;
	xmpp_conn_t *conn;
	bool isConnReady;
	int conn_start();
	void conn_stop();
	static void conn_handler(	xmpp_conn_t * const conn,
								const xmpp_conn_event_t status,
								const int error,
								xmpp_stream_error_t * const stream_error,
								void * const userdata);

	static int handler_actuation(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
					void * const userdata);
	static void parse_stanza(xmpp_ctx_t *ctx, xmpp_stanza_t * const stanza, XMPPWorker *XMPPWork);

	/* Transaction Management */
	pthread_mutex_t queue_lock;
	TransactionQueue queue;

	/* Thread Management */
	/// Signal to worker thread to exit
	bool doExit;
	/// Simply indicates if \a start or \a stop has been called
	bool isStarted;
	/// Indicates if the worker thread is running
	bool isWorkerRunning;
	/* The following two variable are primarily for signaling
	 * to the main thread for some event. */
	pthread_mutex_t worker_status_lock;
	pthread_cond_t worker_status_cond;
	pthread_t worker;
	static void *worker_routine(void *data);
	
	/**
 * The blob of data to transfer to filter handlers
 */
	struct handler_data {
		xmpp_ctx_t *ctx;
		const char *op1;
		const char *op2;
		const char *op3;
		XMPPWorker *XMPPWork;
	};

};

#endif /* XMPPWORKER_H_ */
