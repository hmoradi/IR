/** @file PubSub.h
 *  @author Craig Hesling
 */

#ifndef PUBSUB_H_
#define PUBSUB_H_

#include <string.h>
#include <stdlib.h>
#include <strophe.h>

#include <iostream>
#include <vector>
#include <string>
#include <map>

typedef int (*node_event_handler_t)(xmpp_stanza_t *stanza);

class PubSub {
private:
	xmpp_ctx_t *ctx;
	xmpp_conn_t *conn;
	xmpp_log_t *log;
	char *jid;
	char *pass;
	char *pubsub_jid;
	/** The parent node to publish child nodes to */
	char *collection_node;
	int verbosity_level;

	/**
	 * - 1 = ready
	 */
	int status;
//	struct Node;
//	std::map<std::string, Node> nodes;
	std::map<std::string, node_event_handler_t> event_handlers;

	xmpp_stanza_t *send_and_wait(xmpp_stanza_t *stanza);

	std::string next_id();
public:
	PubSub(const char *jid, const char *pass, const char *collection_node);
	virtual ~PubSub();
	void verbosity(int level);
	int connect();
	void run();
	void run_once(unsigned long timeout);
	void stop();

	std::vector<std::string> node_discovery();
	int  node_delete(const char *child_node);
	void node_nuke();
	int node_create(const char *child_node);
	void node_event(const char *child_node, node_event_handler_t handler);
	void publish(const char *child_node, const char *item_id, char *msg);
	void subscribe(const char *child_node);

	void report(const char *fmt, ...);
	void test();

	int static pubsub_event_handler(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
			void * const userdata);
	/** Messy hack */
	void (*handler)(xmpp_stanza_t *stanza);
	void ready();
};

#endif /* PUBSUB_H_ */
