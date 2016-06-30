/**@file XMPPWorker.cc
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <iostream>
#include <cassert>
#include <cstddef> /* NULL */
#include <cstdarg> /* print_info, print_dbg, and print_err */
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include "Stanza.h"
#include "XMPPWorker.h"
#define XMPP_RUNONCE_TIMEOUT 1

using namespace std;

static void print_info(const char *fmt, ...) {
	va_list arguments;

	va_start(arguments, fmt);
	std::string fmt_str("# INFO: ");
	fmt_str.append(fmt);

	vfprintf(stderr, fmt_str.c_str(), arguments);
	va_end(arguments);
}

static void print_dbg(const char *fmt, ...) {
	va_list arguments;

	va_start(arguments, fmt);
	std::string fmt_str("# DEBUG: ");
	fmt_str.append(fmt);
#ifdef DEBUG
	vfprintf(stderr, fmt_str.c_str(), arguments);
#endif
	va_end(arguments);
}

static void print_err(const char *fmt, ...) {
	va_list arguments;

	va_start(arguments, fmt);
	std::string fmt_str("# ERR: ");
	fmt_str.append(fmt);

	vfprintf(stderr, fmt_str.c_str(), arguments);
	va_end(arguments);
}

XMPPWorker::XMPPWorker(	const char *jid,
						const char *pass,
						unsigned short altport,
						const char * const altdomain,
						const char *node,
						const char *parent_node,
						bool enable_actuation,
						bool enable_distributed_coordination) :
		jid(jid), pass(pass), altport(altport), altdomain(altdomain),  
		node(strdup(node)), parent_node(strdup(parent_node)), enable_actuation(enable_actuation),
		enable_distributed_coordination(enable_distributed_coordination),
		xmpp_log_level(XMPP_LEVEL_WARN), isConnReady(false), doExit(false),
		isStarted(false), isWorkerRunning(false) {
		
	previous_timestamp[0] = 0;

	/*
	 * Initializing the queue_lock now allows the user to
	 * queue transactions with the send method without the
	 * worker thread running.
	 */
	pthread_mutex_init(&queue_lock, NULL);
	pthread_mutex_init(&worker_status_lock, NULL);
	pthread_cond_init(&worker_status_cond, NULL);

	/* initialize library */
	xmpp_initialize();

	/* create a context */
	logg = xmpp_get_default_logger(xmpp_log_level);
	ctx = xmpp_ctx_new(NULL, logg);

	/* create a connection */
	conn = xmpp_conn_new((xmpp_ctx_t *) ctx);

	/* setup authentication information */
	setJID(jid);
	setPass(pass);
	
	/* determine the name of the actuation node */
	actuation_node = (char *)malloc(strlen(node) + 5);
	strcpy(actuation_node, node);
	strcat(actuation_node, "_act");	
	cout << "Actuation node name: " << actuation_node << endl;
}

bool XMPPWorker::isActuationEnabled()
{
	return enable_actuation;
}

bool XMPPWorker::isDistributedCoordinationEnabled()
{
	return enable_distributed_coordination;
}


XMPPWorker::~XMPPWorker() {
	/* release our connection and context */
	xmpp_conn_release(conn);
	xmpp_ctx_free((xmpp_ctx_t *) ctx);

	free((void *) node);            
	free((void *) parent_node);            
	free ((void *) actuation_node);

	/* final shutdown of the library */
	xmpp_shutdown();

	pthread_mutex_destroy(&queue_lock);
	pthread_cond_destroy(&worker_status_cond);
	pthread_mutex_destroy(&worker_status_lock);
}

void XMPPWorker::setJID(const char *jid) {
	this->jid = jid;
}

void XMPPWorker::setPass(const char *pass) {
	this->pass = pass;
}

void XMPPWorker::setAltPort(unsigned short port) {
	this->altport = port;
}

void XMPPWorker::setAltDomain(const char *domain) {
	this->altdomain = domain;
}

/**
 * Sets the libstrophe log level. The log level is only
 * committed upon calling \ref start.
 * Possible values are the following:
 * 	- XMPP_LEVEL_DEBUG
 * 	- XMPP_LEVEL_INFO
 * 	- XMPP_LEVEL_WARN
 * 	- XMPP_LEVEL_ERROR
 * @param log_level The chosen log level.
 */
void XMPPWorker::setXMPPLogLevel(xmpp_log_level_t log_level) {
	xmpp_log_level = log_level;
}

xmpp_ctx_t *XMPPWorker::getCTX() {
	return ctx;
}

bool XMPPWorker::isRunning() {
	return isWorkerRunning;
}

size_t XMPPWorker::getPending() {
	size_t pending;
	pthread_mutex_lock(&queue_lock);
	pending = queue.size();
	pthread_mutex_unlock(&queue_lock);
	return pending;
}

/**
 * Start the xmpp worker thread that sends the xmpp messages.
 * This function sleeps until the worker thread has responded
 * that it is running(\a isWorkerRunning).
 * @return 0 upon success, 1 upon error
 */
int XMPPWorker::start() {
	if (isStarted) {
		print_err("start: Error - XMPPWorker thread has already been started\n");
		return 1;
	}
	isStarted = true;

	doExit = false;

	/* stop the worker thread on this locked mutex */
	pthread_mutex_lock(&worker_status_lock);

	if (pthread_create(&worker, NULL, worker_routine, (void *) this) == 0) {
		/* sleep until we hear from the worker thread */
		print_dbg("start: waiting for worker thread to respond\n");
		pthread_cond_wait(&worker_status_cond, &worker_status_lock);
		/* worker thread has set the initial status - main has lock now */
		assert(isWorkerRunning);
		print_dbg("start: worker thread responded\n");
		pthread_mutex_unlock(&worker_status_lock);
		return 0;
	} else {
		/* error - worker thread could not be created */
		isStarted = false;
		print_err("start: failed to create the worker thread\n");
		pthread_mutex_unlock(&worker_status_lock);
		return 1;
	}

}

int XMPPWorker::stop() {
	void *ret;

	if (!isStarted) {
		print_err("stop: Error - XMPPWorker thread is already stopped\n");
		return 1;
	}

	/* signal to worker thread to exit */
	print_dbg("stop: signaling worker thread to stop\n");
	doExit = true;
	/* wait for thread to exit */
	print_dbg("stop: waiting for worker thread to join\n");
	pthread_join(worker, &ret);
	print_dbg("stop: worker thread joined\n");

	/* cleanup */
	isStarted = false;

	return 0;
}


 void XMPPWorker::parse_stanza(xmpp_ctx_t *ctx, xmpp_stanza_t * const stanza, XMPPWorker *XMPPWork) {
	char *rawtext;
	size_t rawtext_size;
	xmpp_stanza_to_text(stanza, &rawtext, &rawtext_size);

	int print_raw_message = 1;
	if(print_raw_message)	
		fprintf(stdout, "%s \n", rawtext);
	
	
	//parse node ID
	char *current_str;
	current_str = strstr(rawtext, XMPPWork->actuation_node);
	
	if(current_str == NULL){
		fprintf(stdout, "FORK doesn't exist in the message!\n");
		fprintf(stdout, "%s", rawtext);
		return;
	}
	
	
	//parse count
	char count[25];
	count[0] = '0', count[1] = '\0';
	current_str = strstr(rawtext, "transducerData value=");
	if(current_str != NULL){
		count[0] = *(current_str + 22);
		int i = 1;
		for(;current_str[22+i] && isdigit(current_str[22+i]); i++)
			count[i] = current_str[22+i];
		count[i] = 0;			
	}
	else
	{
		current_str = strstr(rawtext, "Person Count&quot;:");
		if (current_str != NULL){
			count[0] = *(current_str + 20);
			int i = 1;
			for(;current_str[20+i] && isdigit(current_str[20+i]); i++)
				count[i] = current_str[20+i];
			count[i] = 0;			
		}
	}

	//timestamp
	char timestamp[25];
	current_str = strstr(rawtext, "timestamp=");
	if(current_str != NULL) {
		strncpy(timestamp, current_str + 11, 19);
		timestamp[20] = 0;
	}
	else
	{
		current_str = strstr(rawtext, "timestamp&quot;: &quot;");
		if (current_str != NULL) {
			strncpy(timestamp, current_str + 23, 19);
			timestamp[20] = 0;			
		}
	}
	fprintf(stdout, "Timestamp extracted: %s\n", timestamp);
	
	if(!strcmp(timestamp, XMPPWork->previous_timestamp)) { //we have received this message before
		fprintf(stdout, "Ignoring actuation request. This request is received before (same timestamp).\n");	
		return;
	}
	else
		strcpy(XMPPWork->previous_timestamp, timestamp);


	XMPPWork->actuation_count = atoi(count);
	fprintf(stdout, "Actuation count: %d\n", atoi(count));
	
	raise(SIGUSR1); //it will update the count
	
	fflush(stdout);
	//xmpp_free(ctx, rawtext);
	delete rawtext;
}

int XMPPWorker::get_actuation_count()
{
	return actuation_count;	
}



int XMPPWorker::handler_actuation(	xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
					void * const userdata) {
	xmpp_stanza_t *event;
	xmpp_stanza_t *items;
	struct handler_data *hdata = (struct handler_data *) userdata;
	const char *node = hdata->op1;
	XMPPWorker *XMPPWork = hdata->XMPPWork;
	assert(hdata->op3 == NULL);

	event = xmpp_stanza_get_child_by_ns(stanza, "http://jabber.org/protocol/pubsub#event");
	if(event == NULL) {
		/* not PubSub */
		return 1;
	}

	/*
	if(node == NULL) {
		parse_stanza(hdata->ctx, stanza, XMPPWork);
		return 1;
	} */

	items = xmpp_stanza_get_children(event);
	while (items) {
		/* check that it is an item */
		if(strcmp(xmpp_stanza_get_name(items), "items") == 0) {
			/* check node id */
			if(strcmp(xmpp_stanza_get_attribute(items, "node"), XMPPWork->actuation_node) == 0) {
				parse_stanza(hdata->ctx, items, XMPPWork);
			}
		}
		items = xmpp_stanza_get_next(items);
	}

	return 1;
}


int XMPPWorker::conn_start() {
	isConnReady = false;

	if(jid == NULL) {
		return 1;
	}
	if(pass == NULL) {
		return 2;
	}

	xmpp_conn_set_jid(conn, jid);
	xmpp_conn_set_pass(conn, pass);

	/* initiate connection */
	return xmpp_connect_client(conn, altdomain, altport, conn_handler,
								(void *) this);		
}

/**
 * Send a transaction asynchronously
 * @param trans The transaction to send
 */

void XMPPWorker::send(Transaction* trans) {
	pthread_mutex_lock(&queue_lock);
	queue.enqueue(trans);
	pthread_mutex_unlock(&queue_lock);
}



void XMPPWorker::conn_stop() {
	isConnReady = false;

	//	xmpp_stop(ctx);
	xmpp_disconnect(conn);
}

/* define a handler for connection events */
void XMPPWorker::conn_handler(	xmpp_conn_t * const conn,
								const xmpp_conn_event_t status,
								const int error,
								xmpp_stream_error_t * const stream_error,
								void * const userdata) {
	XMPPWorker *xmppworker = (XMPPWorker *) userdata;

	if (status == XMPP_CONN_CONNECT) {
		xmpp_stanza_t* pres;
		print_info("connection connected\n");

		/* Send initial <presence/> so that we appear online to contacts */
		pres = xmpp_stanza_new(xmppworker->ctx);
		xmpp_stanza_set_name(pres, "presence");
		xmpp_send(xmppworker->conn, pres);
		xmpp_stanza_release(pres);
		xmppworker->isConnReady = true;
		print_info("conn_handler: connection is ready\n");
	} else if (status == XMPP_CONN_FAIL) {
		print_info("connection failed\n");
		xmppworker->isConnReady = false;
		xmppworker->doExit = true;
	} else {
		print_info("connection disconnected\n");
		xmppworker->isConnReady = false;
		xmppworker->doExit = true;
	}
}

/**
 * This is the function that the worker thread is created with.
 * It contains a runtime loop for sending outgoing messages and
 * jogging the \ref xmpp_run_once function.
 * @param data
 */
void *XMPPWorker::worker_routine(void *data) {
	XMPPWorker *work = (XMPPWorker *) data;

	print_info("worker_routine: starting\n");

	pthread_mutex_lock(&work->worker_status_lock);
	work->isWorkerRunning = true;
	pthread_cond_signal(&work->worker_status_cond);
	pthread_mutex_unlock(&work->worker_status_lock);

	if (work->conn_start() != 0) {
		print_err("worker_routine: conn_start failed - worker thread exiting\n");
		work->doExit = true;
	}

	/*Add a hander: This is the actuation interface */
	
	struct handler_data hdata;	
	hdata.op1 = work->node;
	hdata.XMPPWork = work;
	if( work->enable_actuation){
		xmpp_handler_add(work->conn, handler_actuation, NULL, "message", "headline", (void *) &hdata);
	}
	
	/*
	 * Welcome to the worker thread runtime loop.
	 * Effectively, we spend all the time in the xmpp_run_once function.
	 * Since we need to run xmpp_run_once to receive incoming message,
	 * we cannot sleep on a semaphore associated with the outgoing queue.
	 * We can only hope that xmpp_run_once implements some yielding mechanism
	 * when it is not in use for the timeout specified.
	 */
	while (!work->doExit) {
		if (work->isConnReady) {
			Transaction *trans;

			/* send queued transactions */
			pthread_mutex_lock(&work->queue_lock);
			trans = work->queue.dequeue();
			pthread_mutex_unlock(&work->queue_lock);
			if (trans) {
				/* figure out if a handler was specified before sending */
				bool defaultHandler = !trans->getHandler();

				print_dbg("worker_routine: sending the following stanza:\n%s\n",
							trans->getRequest()->toText().c_str());
				/* send */
				trans->send(work->conn);

				/* we must delete the transaction if no handler was specified */
				if(defaultHandler) {
					delete trans;
				}
			}
		}

		/* run the libstrophe event loop for a bit */
		xmpp_run_once(work->ctx, 1);
	}

	print_info("worker_routine: exiting with %lu transactions pending\n",
				work->queue.size());

	work->conn_stop();
	pthread_mutex_lock(&work->worker_status_lock);
	work->isWorkerRunning = false;
	pthread_mutex_unlock(&work->worker_status_lock);
	pthread_exit(NULL);
}
