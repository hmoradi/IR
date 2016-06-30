/** @file PubSub.cc
 *  @author Craig Hesling
 */

#include "PubSub.h"

#include <string.h>
#include <stdlib.h>
#include <strophe.h>
#include <assert.h>

#include <cstdarg>
#include <cstdlib>
#include <vector>
#include <map>

using namespace std;

struct Response {
	xmpp_stanza_t *stanza;

	Response() {
		stanza = NULL;
	}

	static int handler(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
			void * const userdata) {

		Response *resp = (Response *)userdata;
		resp->stanza = xmpp_stanza_clone(stanza);
		// do not reregister
		return 0;
	}
};

//struct Event {
//	xmpp_stanza_t *stanza;
//
//	Event() {
//		stanza = NULL;
//	}
//
//	static int handler(
//			xmpp_conn_t * const conn,
//			xmpp_stanza_t * const stanza,
//			void * const userdata) {
//
//		Event *resp = (Event *)userdata;
//		resp->stanza = xmpp_stanza_clone(stanza);
//		// do not reregister
//		return 1;
//	}
//};

//struct Node {
//	/** This is the MAC address of the node */
//	char addr[8];
//	/** This data type maps item IDs to handlers */
//	map<string, int (*)(void *)> handlers;
//
//	void set_addr(char *addr) {
//		memcpy(this->addr, addr, sizeof this->addr);
//	}
//	string get_addr_str() {
//		// 8*2 + 1 = 17
//		char str[17] = {0};
//		unsigned char *addr = (unsigned char *) this->addr;
//
//		snprintf(str, sizeof str, "%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",
//				addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6],
//				addr[7]);
//
//		return string(str);
//	}
//
//	static int event_handler(
//			xmpp_conn_t * const conn,
//			xmpp_stanza_t * const stanza,
//			void * const userdata) {
//
//		Node *node = (Node *)userdata;
//
//		node->stanza = xmpp_stanza_clone(stanza);
//		// do not reregister
//		return 1;
//	}
//};

int version_handler(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
		void * const userdata) {
	xmpp_stanza_t *reply, *query, *name, *version, *text;
	char *ns;
	xmpp_ctx_t *ctx = (xmpp_ctx_t*) userdata;

	printf("Received version request from %s\n",
			xmpp_stanza_get_attribute(stanza, "from"));

	reply = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(reply, "iq");
	xmpp_stanza_set_type(reply, "result");
	xmpp_stanza_set_id(reply, xmpp_stanza_get_id(stanza));
	xmpp_stanza_set_attribute(reply, "to",
			xmpp_stanza_get_attribute(stanza, "from"));

	query = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(query, "query");
	ns = xmpp_stanza_get_ns(xmpp_stanza_get_children(stanza));
	if (ns) {
		xmpp_stanza_set_ns(query, ns);
	}

	name = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(name, "name");
	xmpp_stanza_add_child(query, name);

	text = xmpp_stanza_new(ctx);
	xmpp_stanza_set_text(text, "libstrophe example bot");
	xmpp_stanza_add_child(name, text);

	version = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(version, "version");
	xmpp_stanza_add_child(query, version);

	text = xmpp_stanza_new(ctx);
	xmpp_stanza_set_text(text, "1.0");
	xmpp_stanza_add_child(version, text);

	xmpp_stanza_add_child(reply, query);

	xmpp_send(conn, reply);
	xmpp_stanza_release(reply);
	return 1;
}

int message_handler(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza,
		void * const userdata) {

	xmpp_stanza_t *reply, *body, *text;
	char *intext, *replytext;
	xmpp_ctx_t *ctx = (xmpp_ctx_t*) userdata;

	/* look for body tag - end if it is not found */
	if (!xmpp_stanza_get_child_by_name(stanza, "body"))
		return 1;

	if (xmpp_stanza_get_attribute(stanza, "type") != NULL
			&& !strcmp(xmpp_stanza_get_attribute(stanza, "type"), "error"))
		return 1;

	/* grab the body's text */
	intext = xmpp_stanza_get_text(
			xmpp_stanza_get_child_by_name(stanza, "body"));

	/* check for close command */
	if (strcmp(intext, "close") == 0) {
		xmpp_stop(ctx);
		return 0;
	}

	/* check for publish now command */
	if (strcmp(intext, "publish now") == 0) {
		return 1;
	}

	printf("Incoming message from %s: %s\n",
			xmpp_stanza_get_attribute(stanza, "from"), intext);

	/* <message /> */
	reply = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(reply, "message");
	xmpp_stanza_set_type(reply,
			xmpp_stanza_get_type(stanza) ?
					xmpp_stanza_get_type(stanza) : "chat");
	xmpp_stanza_set_attribute(reply, "to",
			xmpp_stanza_get_attribute(stanza, "from"));

	/* <body /> */
	body = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(body, "body");

	replytext = (char *) malloc(strlen(" to you too!") + strlen(intext) + 1);
	strcpy(replytext, intext);
	strcat(replytext, " to you too!");

	text = xmpp_stanza_new(ctx);
	//xmpp_stanza_set_name(text, "text");
	xmpp_stanza_set_text(text, replytext);
	xmpp_stanza_add_child(body, text);
	xmpp_stanza_add_child(reply, body);

	xmpp_send(conn, reply);

	xmpp_stanza_release(reply);
	xmpp_free(ctx, intext);
	free(replytext);
	return 1;
}

int PubSub::pubsub_event_handler(xmpp_conn_t * const conn, xmpp_stanza_t * const stanza, void * const userdata) {

	PubSub *me = (PubSub *)userdata;
	xmpp_ctx_t *ctx = me->ctx;
	xmpp_stanza_t *seek = NULL;
	xmpp_stanza_t *items = NULL;

	/*
	 * <message type="headline">
	 *   <event xmlns="">
	 *     <items node="id">
	 *       <item id="other_id">
	 *         <some_tag></some_tag>
	 *       </item>
	 *     </item>
	 *   </event>
	 * </message>
	 *
	 */

	assert(strcmp(xmpp_stanza_get_name(stanza), "message") == 0);
	seek = xmpp_stanza_get_child_by_name(stanza, "event");

	/* get <items node=""/> */
	items = xmpp_stanza_get_children(seek);
	assert(strcmp(xmpp_stanza_get_name(items), "items") == 0);

	do {
		seek = xmpp_stanza_get_children(items);
		if (strcmp(xmpp_stanza_get_name(seek), "item") == 0) {
			node_event_handler_t handler;
			string node = string(xmpp_stanza_get_attribute(seek, "node"));
			/* find handler for the given id */
			handler = me->event_handlers[node];
			/* run handler if found */
			if (handler != NULL) {
				handler(seek);
			}
		}
	} while (items = xmpp_stanza_get_next(items));

	fprintf(stderr, "\nGOT PUBSUB EVENT!\n");

	char *buf;
	size_t buf_size;
	xmpp_stanza_to_text(stanza, &buf, &buf_size);
	fprintf(stderr, "Publishing: %s\n", buf);
	xmpp_free(ctx, buf);

	return 1;
}

/* define a handler for connection events */
void conn_handler(xmpp_conn_t * const conn, const xmpp_conn_event_t status,
		const int error, xmpp_stream_error_t * const stream_error,
		void * const userdata) {

	PubSub *pubsub = (PubSub *) userdata;

	if (status == XMPP_CONN_CONNECT) {
		fprintf(stderr, "DEBUG: connection connected\n");
		pubsub->ready();
	} else if (status == XMPP_CONN_FAIL) {
		fprintf(stderr, "DEBUG: connection failed\n");
		pubsub->stop();
	} else {
		fprintf(stderr, "DEBUG: connection disconnected\n");
		pubsub->stop();
	}
}

/**
 * Initialize a Publish-Subscribe instance.
 * @param jid The XMPP JID to bind to.
 * @param pass The JID's password.
 * @param collection_node The PubSub root node to publish to sub nodes to
 */
PubSub::PubSub(const char *jid, const char *pass, const char *collection_node) {
	int index = 0;

	assert(jid && (strlen(jid) > 3));
	assert(pass && (strlen(pass) >= 0));
	assert(collection_node && (strlen(collection_node) > 0));


	this->jid = const_cast<char *>(jid);
	this->pass = const_cast<char *>(pass);
	this->collection_node = const_cast<char *>(collection_node);
	this->status = 0;
	this->verbosity_level = 0;

	/* initialize libstrophe */
	xmpp_initialize();

	/* create a context */
//	log = xmpp_get_default_logger(XMPP_LEVEL_DEBUG); /* pass NULL instead to silence output */
	log = xmpp_get_default_logger(XMPP_LEVEL_ERROR); /* pass NULL instead to silence output */
	ctx = xmpp_ctx_new(NULL, log);

	/* create a connection */
	conn = xmpp_conn_new(ctx);

	/* setup authentication information */
	xmpp_conn_set_jid(conn, jid);
	xmpp_conn_set_pass(conn, pass);

	/* determine and set pubsub_jid */
	// alloc/copy jid to pubsub_jid
	this->pubsub_jid = new char[strlen(this->jid) + 7 + 1];
	if(this->pubsub_jid == NULL) {
		fprintf(stderr, "Error - Could not set pubsub_jid\n");
		exit(1);
	}
	strcpy(this->pubsub_jid, "pubsub.");
	strcpy(&this->pubsub_jid[7], strchr(this->jid, '@')+1);
	// terminate at optional resource specifier
	if(strchr(this->pubsub_jid, '/')) {
		*strchr(this->pubsub_jid, '/') = '\0';
	}
}

PubSub::~PubSub() {
	fprintf(stderr, "Shutting down\n");

	/* release our connection and context */
	xmpp_conn_release(conn);
	xmpp_ctx_free(ctx);

	/* final shutdown of the library */
	xmpp_shutdown();

	delete this->pubsub_jid;
}

void PubSub::report(const char *fmt, ...) {
	va_list arguments;

	va_start(arguments, fmt);
	if (this->verbosity_level) {
		std::string fmt_str("## PubSub: ");
		fmt_str.append(fmt);

		vfprintf(stderr, fmt_str.c_str(), arguments);
	}
	va_end(arguments);
}

void PubSub::verbosity(int level) {
	this->verbosity_level = level;
}


/*-------------------------------*
 *     Connection Handling       *
 *-------------------------------*/


int PubSub::connect() {
	xmpp_stanza_t* pres;
	int ret;

	/* initiate connection */
	ret = xmpp_connect_client(conn, NULL, 0, conn_handler, this);
	if (ret < 0) {
		return ret;
	}

	/* wait/allow connection to become connected */
	while (!status) {
		// 1 millisecond is the default timeout - see DEFAULT_TIMEOUT in libstrophe/event.c
		xmpp_run_once(ctx, 1);
	}

	/* add in handlers */
	xmpp_handler_add(conn, version_handler, "jabber:iq:version", "iq", NULL, ctx);
	xmpp_handler_add(conn, message_handler, NULL, "message", NULL, ctx);
	xmpp_handler_add(conn, PubSub::pubsub_event_handler,
			"http://jabber.org/protocol/pubsub#event", NULL, NULL, this);

	/* Send initial <presence/> so that we appear online to contacts */
	pres = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(pres, "presence");
	xmpp_send(conn, pres);
	xmpp_stanza_release(pres);

	/* if the connection died along the way */
	if (status == 0) {
		return 1;
	}

	/* all seems to be good */
	return 0;
}

void PubSub::run() {

	/* enter the event loop -
	 our connect handler will trigger an exit */
	xmpp_run(ctx);
}

void PubSub::run_once(unsigned long timeout) {
	xmpp_run_once(ctx, timeout);
}

void PubSub::stop() {
	status = 0;
	xmpp_stop(ctx);
}


/**
 * Send a stanza and wait for the reply
 * @param stanza
 * @return
 */
xmpp_stanza_t *PubSub::send_and_wait(xmpp_stanza_t *stanza) {
	Response resp;
	const char *id = xmpp_stanza_get_id(stanza);

	/* if no id was set, set our own */
	if(strcmp(id, "") == 0) {
		id = this->next_id().c_str();
		xmpp_stanza_set_id(stanza, id);
	}

	/* add response handler */
	xmpp_id_handler_add(this->conn,
						Response::handler,
						id,
						(void *) &resp);
	this->report("Added handler for id %s\n", xmpp_stanza_get_id(stanza));

	xmpp_send(this->conn, stanza);

	char *stan;
	size_t stan_len;
	xmpp_stanza_to_text(stanza, &stan, &stan_len);
	this->report("Sent stanza: %s\n", stan);
	xmpp_free(this->ctx, stan);

	while(resp.stanza == NULL) {
		this->run_once(1);
	}

	return resp.stanza;
}

/**
 * Subscribe this object's jid to \a child_node.
 * @param child_node The node to subscribe to
 * @return
 */
void PubSub::subscribe(const char *child_node) {
	xmpp_stanza_t *iq;
	xmpp_stanza_t *pubsub;
	xmpp_stanza_t *subscribe;
	xmpp_stanza_t *response;

	/* <iq type="get" id="unique_id" to="this->pubsub_jid" /> */
	iq = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(iq, "iq");
	xmpp_stanza_set_type(iq, "set");
	xmpp_stanza_set_id(iq, this->next_id().c_str());
	xmpp_stanza_set_attribute(iq, "to", this->pubsub_jid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub" /> */
	pubsub = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(pubsub, "pubsub");
	xmpp_stanza_set_ns(pubsub, "http://jabber.org/protocol/pubsub");

	/* <subscribe node="child_node" jid="this->jid" /> */
	subscribe = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(subscribe, "subscribe");
	xmpp_stanza_set_attribute(subscribe, "node", child_node);
	xmpp_stanza_set_attribute(subscribe, "jid", this->jid);

	/* build tree */
	xmpp_stanza_add_child(pubsub, subscribe);
	xmpp_stanza_add_child(iq, pubsub);

	 char *buf;
	 size_t buf_size;
	 xmpp_stanza_to_text(iq, &buf, &buf_size);
	 this->report("Publishing: %s\n", buf);
	 xmpp_free(ctx, buf);

	/* send query */
	response = this->send_and_wait(iq);
	xmpp_stanza_release(iq);

	/* process response */
	if(strcmp(xmpp_stanza_get_type(response), "error") == 0) {
		//XXX Error
		this->report("Error subscribing\n");
	}

	xmpp_stanza_release(response);
}

/** Signals the object that the connection is ready for use.
 *  @note Only used by connection manager to signal that the connection is ready.
 */
void PubSub::ready() {
	status = 1;
}

/*-------------------------------*
 *       Content Handling        *
 *-------------------------------*/

string PubSub::next_id() {
	char buf[256];
	snprintf(buf, sizeof(buf), "PubSub_%d", rand());
	return string(buf);
}

vector<string> PubSub::node_discovery() {
	xmpp_stanza_t *iq;
	xmpp_stanza_t *query;
	xmpp_stanza_t *response;
	xmpp_stanza_t *seek;
	vector<string> nodes;


	/* <iq type="get" id="unique_id" to="this->pubsub_jid" /> */
	iq = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(iq, "iq");
	xmpp_stanza_set_type(iq, "get");
	xmpp_stanza_set_id(iq, this->next_id().c_str());
	xmpp_stanza_set_attribute(iq, "to", this->pubsub_jid);

	/* <query xmlns="http://jabber.org/protocol/disco#items" /> */
	query = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(query, "query");
	xmpp_stanza_set_ns(query, "http://jabber.org/protocol/disco#items");

	xmpp_stanza_add_child(iq, query);

	response = this->send_and_wait(iq);
	xmpp_stanza_release(iq);

	seek = xmpp_stanza_get_children(response);
	seek = xmpp_stanza_get_children(seek);

	do {
		if(strcmp(xmpp_stanza_get_name(seek), "item") == 0) {
			nodes.push_back(string(xmpp_stanza_get_attribute(seek, "node")));
		}
	} while(seek = xmpp_stanza_get_next(seek));

	xmpp_stanza_release(response);

	return nodes;
}


int PubSub::node_delete(const char *child_node) {
	xmpp_stanza_t *iq;
	xmpp_stanza_t *pubsub;
	xmpp_stanza_t *del;
	xmpp_stanza_t *response;
	xmpp_stanza_t *seek;

	/* <iq type="get" id="unique_id" to="this->pubsub_jid" /> */
	iq = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(iq, "iq");
	xmpp_stanza_set_type(iq, "set");
	xmpp_stanza_set_id(iq, this->next_id().c_str());
	xmpp_stanza_set_attribute(iq, "to", this->pubsub_jid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub#owner" /> */
	pubsub = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(pubsub, "pubsub");
	xmpp_stanza_set_ns(pubsub, "http://jabber.org/protocol/pubsub#owner");

	/* <delete node="child_node" /> */
	del = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(del, "delete");
	xmpp_stanza_set_attribute(del, "node", child_node);

	xmpp_stanza_add_child(pubsub, del);
	xmpp_stanza_add_child(iq, pubsub);

	this->report("node_delete: Deleting node %s\n", child_node);

	response = this->send_and_wait(iq);
	xmpp_stanza_release(iq);

	if (strcmp(xmpp_stanza_get_type(response), "result") != 0) {
		xmpp_stanza_release(response);
		//error
		return -1;
	}
	xmpp_stanza_release(response);
	return 0;
}

void PubSub::node_nuke() {
	vector<string> nodes = node_discovery();

	for(int i = 0; i < nodes.size(); i++) {
		this->report("node_nuke: Deleting node %s\n", nodes[i].c_str());
		this->node_delete(nodes[i].c_str());
	}
}

int PubSub::node_create(const char *child_node) {
	xmpp_stanza_t *iq;
	xmpp_stanza_t *pubsub;
	xmpp_stanza_t *create;
	xmpp_stanza_t *response;
	xmpp_stanza_t *seek;

	/* <iq type="set" id="unique_id" to="this->pubsub_jid" /> */
	iq = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(iq, "iq");
	xmpp_stanza_set_type(iq, "set");
	xmpp_stanza_set_id(iq, this->next_id().c_str());
	xmpp_stanza_set_attribute(iq, "to", this->pubsub_jid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub" /> */
	pubsub = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(pubsub, "pubsub");
	xmpp_stanza_set_ns(pubsub, "http://jabber.org/protocol/pubsub");

	/* <iq node="child_node" /> */
	create = xmpp_stanza_new(this->ctx);
	xmpp_stanza_set_name(create, "create");
	xmpp_stanza_set_attribute(create, "node", child_node);

	/* build tree */
	xmpp_stanza_add_child(pubsub, create);
	xmpp_stanza_add_child(iq, pubsub);

	response = this->send_and_wait(iq);
	xmpp_stanza_release(iq);

	if (strcmp(xmpp_stanza_get_type(response), "result") != 0) {
		xmpp_stanza_release(response);
		//error
		return -1;
	}

	/* subscribe to the node */
	this->subscribe(child_node);


	xmpp_stanza_release(response);
	return 0;
}

void PubSub::node_event(const char *child_node, node_event_handler_t handler) {
	event_handlers[string(child_node)] = handler;
}

//void PubSub::node_event(
//		const char *child_node,
//		const char *tag,
//		int (*handler)(void *userdata)) {
//
//	this->subscribe(child_node);
//	xmpp_handler_add(this->conn, Event::handler,
//
//}

/**
 * Publish an item with id \a item_id to the node \a child_node, who
 * is a child of \ref PubSub::collection_node .
 * @param child_node The node under \ref PubSub::collection_node to publish to
 * @param item_id The id of the item to publish
 * @param msg The content of the inner body tags
 *
 * @note The item published looks like the following:
 *       \code{.xml}
 *       <item id="item_id">
 *       	<body>
 *       		msg
 *       	</body>
 *       </item>
 *       \endcode
 */
void PubSub::publish(const char *child_node, const char *item_id, char *msg) {
	xmpp_stanza_t *iq;
	xmpp_stanza_t *pubsub;
	xmpp_stanza_t *publish;
	xmpp_stanza_t *item;
	xmpp_stanza_t *body, *body_text;

	/* <iq /> */
	iq = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(iq, "iq");
	xmpp_stanza_set_type(iq, "set");
	xmpp_stanza_set_attribute(iq, "from", jid);
	xmpp_stanza_set_attribute(iq, "to", "pubsub.192.168.1.108");

	/* <pubsub xmlns="" /> */
	pubsub = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(pubsub, "pubsub");
	xmpp_stanza_set_ns(pubsub, "http://jabber.org/protocol/pubsub");

	/* <publish node="child_node" /> */
	publish = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(publish, "publish");
	xmpp_stanza_set_attribute(publish, "node", child_node);

	/* <item id="item_id" /> */
	item = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(item, "item");
	xmpp_stanza_set_id(item, item_id);

	/* <body /> */
	body = xmpp_stanza_new(ctx);
	body_text = xmpp_stanza_new(ctx);
	xmpp_stanza_set_name(body, "body");
	xmpp_stanza_set_text(body_text, msg);

	/* build tree */

	//item
	xmpp_stanza_add_child(body, body_text);
	xmpp_stanza_add_child(item, body);
	//publish
	xmpp_stanza_add_child(publish, item);
	//pubsub
	xmpp_stanza_add_child(pubsub, publish);
	//iq
	xmpp_stanza_add_child(iq, pubsub);


	char *buf;
	size_t buf_size;
	xmpp_stanza_to_text(iq, &buf, &buf_size);
	this->report("Publishing: %s\n", buf);
	xmpp_free(ctx, buf);


	/* send */
	xmpp_send(conn, iq);

	/* release entire stanza tree */
	xmpp_stanza_release(iq);
}
