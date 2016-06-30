/**@file XMPPInterface.cpp
 *
 * @date Jul 14, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <iostream>
#include <cstdlib>
#include <cstddef> /* NULL */
#include <cstdio> /* snprintf, perror */
#include <cstring>
#include <cstdarg> /* print_info, print_dbg, and print_err */
#include <ctime>
#include <sys/time.h>
#include <signal.h>
#include <Stanza.h>
#include <Transaction.h>
#include "XMPPInterface.h"

#define RESPAWN_JID "respawn@sensor.andrew.cmu.edu"

using namespace std;

void sig_handler(int sig) {
	FILE *file = fopen("FORK_SIGPIPE.log", "a");
	if(!file) {
		fprintf(stderr, "# Error - File could not be opened\n");
		exit(12);
	}
	fprintf(file, "# SIG(%d): Oh no! We received a signal! Aborting!\n", sig);
	fflush(file);
	fclose(file);
	abort();
}

void sigpipe_handler(int sig) {
	FILE *file;
	char buf[40];
	struct tm d;
	const time_t t = time(NULL);

	localtime_r(&t, &d);
	strftime(buf, sizeof buf, "%c", &d);

	file = fopen("FORK_SIGPIPE.log", "a");
	if(!file) {
		fprintf(stderr, "# Error - File could not be opened\n");
		abort();
	}
	fprintf(file, "# SIGPIPE: Received a SIGPIPE at %s. -Ignoring\n", buf);
	fflush(file);
	fclose(file);
}

void callme_ontexit(int code, void *data) {
	FILE *file = fopen("FORK_SIGPIPE.log", "a");
	if(!file) {
		fprintf(stderr, "on_exit: code=%d\n", code);
	}
	fprintf(file, "on_exit: code=%d\n", code);
	fflush(file);
	fclose(file);
}

void setup_sighandlers() {
	signal(SIGPIPE, sigpipe_handler);
	/*
	signal(SIGHUP, sig_handler);
	//signal(SIGINT, sig_handler);
	signal(SIGQUIT, sig_handler);
	signal(SIGILL, sig_handler);
	signal(SIGSEGV, sig_handler);
	signal(SIGALRM, sig_handler);
	signal(SIGTERM, sig_handler);
	signal(SIGUSR1, sig_handler);
	signal(SIGUSR2, sig_handler);
	signal(SIGCHLD, sig_handler);
	signal(SIGCONT, sig_handler);
	signal(SIGSTOP, sig_handler);
	signal(SIGTSTP, sig_handler);

	on_exit(callme_ontexit, NULL);
	*/
}

/**
 * Only print when \a log_level is at least 1
 * @param fmt The format message to print
 */
void XMPPInterface::print_err(const char *fmt, ...) {
	va_list arguments;

	/* allow print if log level is at least 1 */
	if(log_level < 1) {
		return ;
	}

	va_start(arguments, fmt);
	std::string fmt_str("# ERR: ");
	fmt_str.append(fmt);

	vfprintf(stderr, fmt_str.c_str(), arguments);
	va_end(arguments);
}

/**
 * Only print when \a log_level is at least 2
 * @param fmt The format message to print
 */
void XMPPInterface::print_info(const char *fmt, ...) {
	va_list arguments;

	/* allow print if log level is at least 2 */
	if(log_level < 2) {
		return ;
	}

	va_start(arguments, fmt);
	std::string fmt_str("# INFO: ");
	fmt_str.append(fmt);

	vfprintf(stderr, fmt_str.c_str(), arguments);
	va_end(arguments);
}

/**
 * Only print when \a log_level is at least 3
 * @param fmt The format message to print
 */
void XMPPInterface::print_dbg(const char *fmt, ...) {
	va_list arguments;

	/* allow print if log level is at least 3 */
	if(log_level < 3) {
		return ;
	}

	va_start(arguments, fmt);
	std::string fmt_str("# DEBUG: ");
	fmt_str.append(fmt);

	vfprintf(stderr, fmt_str.c_str(), arguments);
	va_end(arguments);
}


/**
 *
 * @param jid The XMPP JID to login as
 * @param pass The XMPP password to login as
 * @param node The XMPP node to publish t
 * @param log_level Sets the logging verbosity. 0=silent, 1=errors, 2=info, 3=debug
 * @param altport The XMPP alternative port
 * @param altdomain The XMPP alternative domain
 */
XMPPInterface::XMPPInterface(	const char *jid,
								const char *pass,
								const char *node,
								const char *parent_node,
								bool enable_distributed_coordination,
								bool enable_actuation, 
								int log_level,
								unsigned short altport,
								const char * const altdomain) :
		jid(strdup(jid)), pass(strdup(pass)), node(strdup(node)), parent_node(strdup(parent_node)),
		log_level(log_level), altport(altport), altdomain(strdup(altdomain?altdomain:"")) {

	setup_sighandlers();

	if (this->jid == NULL) {
		perror("XMPPInterface could not make a local copy of the jid string\n");
	}

	if (this->pass == NULL) {
		perror("XMPPInterface could not make a local copy of the pass string\n");
	}

	if (this->node == NULL) {
		perror("XMPPInterface could not make a local copy of the node string\n");
	}

	if (this->altdomain == NULL) {
		perror("XMPPInterface could not make a local copy of the altdomain string\n");
	}

	xwork = new XMPPWorker(this->jid, this->pass, this->altport, strlen(this->altdomain)?this->altdomain:NULL, node, parent_node, enable_actuation, enable_distributed_coordination);

	if (xwork->start()) {
		print_err("XMPPInterface: Error - Failed to start the sender thread");
	}

	create();
}

XMPPInterface::~XMPPInterface() {
	while(xwork->isRunning() && (xwork->getPending() > 0)) ;
	if (xwork->stop()) {
		print_err("~XMPPInterface: Error - Failed to stop the sender thread\n");
	}

	free((void *) altdomain);
	free((void *) node);
	free((void *) parent_node);
	free((void *) pass);
	free((void *) jid);
	delete xwork;
}

int XMPPInterface::get_actuation_count()
{
	return xwork->get_actuation_count();
}

void XMPPInterface::sendMIOMessage(const char *node, const char *timestamp, int count)
{
	Transaction *trans;
	Stanza *iq, *pubsub, *publish, *item, *transducerData;
	const char *pjid = pubsub_jid(jid);

	/*---------------------*/
	/* Construct MIO Item  */
	/*---------------------*/
	
	trans = new Transaction();

	iq             = new Stanza(xwork->getCTX());
	pubsub         = new Stanza(xwork->getCTX());
	publish        = new Stanza(xwork->getCTX());
	item           = new Stanza(xwork->getCTX());
	transducerData = new Stanza(xwork->getCTX());

	/* <iq type="set"/> */
	iq->setName("iq");
	iq->setType("set");
	iq->setAttribute("to", pjid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub"/> */
	pubsub->setName("pubsub");
	pubsub->setNS("http://jabber.org/protocol/pubsub");

	/* <publish node="$node"/> */
	publish->setName("publish");
	publish->setAttribute("node", node);

	/* <item id="_Person Count"/> */
	item->setName("item");
	item->setID("_Person Count");

	char count_str[100];
	snprintf(count_str, sizeof(count_str), "%d", count);

	/* <transducerData value="$value" timestamp="2015-05-29T12:42:33.905137-0400" name="Person Count"/> */
	transducerData->setName("transducerData");
	transducerData->setAttribute("timestamp", timestamp);
	transducerData->setAttribute("value", count_str);
	transducerData->setAttribute("name", "Person Count");

	/* build stanza tree */
	item->addChild(transducerData);
	publish->addChild(item);
	pubsub->addChild(publish);
	iq->addChild(pubsub);

	/* build the transaction */
	trans->setRequest(iq);
	trans->setHandler(handler_publish, (void *)this);

	/* send */
	xwork->send(trans);

	delete iq;
	delete pubsub;
	delete publish;
	delete item;
	delete transducerData;

	delete[] pjid;
}

void XMPPInterface::sendJSONMessage(const char *node, const char *timestamp, int count, int offset)
{
	Transaction *trans;
	Stanza *iq, *pubsub, *publish, *item, *transducerData;
	Stanza *transducerDataJSON, *transducerDataJSONText;
	char json[4096];

	const char *pjid = pubsub_jid(jid);

	/*---------------------*/
	/* Construct JSON Item */
	/*---------------------*/
	trans = new Transaction();

	iq                     = new Stanza(xwork->getCTX());
	pubsub                 = new Stanza(xwork->getCTX());
	publish                = new Stanza(xwork->getCTX());
	item                   = new Stanza(xwork->getCTX());
	transducerDataJSON     = new Stanza(xwork->getCTX());
	transducerDataJSONText = new Stanza(xwork->getCTX());

	/* <iq type="set"/> */
	iq->setName("iq");
	iq->setType("set");
	iq->setAttribute("to", pjid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub"/> */
	pubsub->setName("pubsub");
	pubsub->setNS("http://jabber.org/protocol/pubsub");

	/* <publish node="$node"/> */
	publish->setName("publish");
	publish->setAttribute("node", node);

	/* <item id="_Summary"/> */
	item->setName("item");
	item->setID("_Summary");

	/* <transducerDataJSON /> */
	transducerDataJSON->setName("transducerDataJSON");

	char count_str[100];
	snprintf(count_str, sizeof(count_str), "%d", count);

	char occ_change[100], occ_change_amount[100], occ_change_dir[5];
	
	if(offset >= 0) {
		occ_change_dir[0] = '0', occ_change_dir[1] = '\0'; //OccChangeDir --> positive:0, negative:1
		snprintf(occ_change_amount, sizeof(occ_change_amount), "%d", offset);
		
		//occ_change[0] = '+';
		snprintf(occ_change, sizeof(occ_change) - 1, "%d", offset);
	}
	else {
		occ_change_dir[0] = '1', occ_change_dir[1] = '\0'; //OccChangeDir --> positive:0, negative:1
		snprintf(occ_change_amount, sizeof(occ_change_amount), "%d", -offset);

		occ_change[0] = '-';
		snprintf(occ_change + 1, sizeof(occ_change) - 1, "%d", -offset);
	}
	
	/* <transducerDataJSON>{ "id": "FORK003763250647", "type": "FORK", "timestamp":"2015-10-02T12:09:25.137168-0400", "payload": { "Person Count": 30, "OccChange":"+1", "OccChangeAmnt":1, "OccChangeDir":0 } }</transducerDataJSON> */


	json[0] = '\0';
	snprintf(json, sizeof(json), "{ \"id\": \"%s\", \"type\": \"FORK\", \"timestamp\": \"%s\", \"payload\": { \"Person Count\": %s, \"personCount\": %s, \"occChange\": %s, \"occChangeAmnt\": %s, \"occChangeDir\": %s} }", node, timestamp, count_str, count_str, occ_change, occ_change_amount, occ_change_dir);

	print_info("JSON: %s\n", json);
	transducerDataJSONText->setText(json);

	/* build stanza tree */
	transducerDataJSON->addChild(transducerDataJSONText);
	item->addChild(transducerDataJSON);
	publish->addChild(item);
	pubsub->addChild(publish);
	iq->addChild(pubsub);

	/* build the transaction */
	trans->setRequest(iq);
	trans->setHandler(handler_publish, (void *)this);

	/* send */
	xwork->send(trans);

	delete iq;
	delete pubsub;
	delete publish;
	delete item;
	delete transducerDataJSON;
	delete transducerDataJSONText;

	delete[] pjid;

}

void XMPPInterface::occupancyChange(int count, int offset = IGNORE) {
	const char *timestamp = create_timestamp();
	char count_str[100], offset_str[100];

	/* report if the connection is dead */
	if(!xwork->isRunning()) {
			print_err("XMPP is disconnected!\n");
			print_info("XMPP is trying to reconnect!\n");
			/* try to reconnect */
			
			bool enable_actuation = xwork->isActuationEnabled();
			bool enable_distributed_coordination = xwork->isDistributedCoordinationEnabled();			
			xwork->stop();
			delete xwork;

			xwork = new XMPPWorker(jid, pass, altport, strlen(altdomain)==0?NULL:altdomain, node, parent_node, enable_actuation, enable_distributed_coordination);
			
			if (xwork->start()) {
				print_err("XMPPInterface: Error - Failed to start the sender thread");
			}
	}

	
	/* send new count to FORK_NODE */ 
	sendMIOMessage(node, timestamp, count);
	sendJSONMessage(node, timestamp, count, offset);
	
	/* send offset to FORK_PARENT_NODE */
	if( offset != IGNORE && xwork->isDistributedCoordinationEnabled()) 
		sendJSONMessage(parent_node, timestamp, count, offset);
	
	
	delete[] timestamp;

	print_info("occupancyChange: Occupancy Changed to %d\n", count);

}

void XMPPInterface::publish_meta() {
	Transaction *trans;
	Stanza *iq, *pubsub, *publish, *item;
	Stanza *meta, *transducer, *property;
	const char *pjid = pubsub_jid(jid);
	const char *timestamp = create_timestamp();

	/*---------------------*/
	/* Construct MIO Item  */
	/*---------------------*/

	trans = new Transaction();

	iq         = new Stanza(xwork->getCTX());
	pubsub     = new Stanza(xwork->getCTX());
	publish    = new Stanza(xwork->getCTX());
	item       = new Stanza(xwork->getCTX());
	meta       = new Stanza(xwork->getCTX());
	transducer = new Stanza(xwork->getCTX());
	property   = new Stanza(xwork->getCTX());

	/* <iq type="set"/> */
	iq->setName("iq");
	iq->setID("publish_meta");
	iq->setType("set");
	iq->setAttribute("to", pjid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub"/> */
	pubsub->setName("pubsub");
	pubsub->setNS("http://jabber.org/protocol/pubsub");

	/* <publish node="$node"/> */
	publish->setName("publish");
	publish->setAttribute("node", node);

	/* <item id="meta"/> */
	item->setName("item");
	item->setID("meta");

	/* <meta xmlns="http://jabber.org/protocol/mio"
	 *       type="device"
	 *       timestamp="2015-06-23T12:28:23.896223-0400"
	 *       name="FORK <KINECT_SERIAL>"
	 *       info="Bosch FORK Node" /> */
	meta->setName("meta");
	meta->setNS("http://jabber.org/protocol/mio");
	meta->setType("device");
	meta->setAttribute("timestamp", timestamp);
	/* TODO: Using node for name results in "FORK<KINECT_SERIAL>"
	 * without space. We want it to be dynamic and have a space.
	 * This may require refactoring the XMPPInterface to accept
	 * a Kinect serial instead of a node.
	 */
	meta->setAttribute("name", node);
	meta->setAttribute("info", "Bosch FORK Node");

	/* <transducer unit="Quantity" type="doorway person count" name="Person Count"/> */
	transducer->setName("transducer");
	transducer->setAttribute("unit", "Quantity");
	transducer->setType("doorway person count");
	transducer->setAttribute("name", "Person Count");

	/* <property value="FORK" name="type"/> */
	property->setName("property");
	property->setAttribute("value", "FORK");
	property->setAttribute("name", "type");

	/* build stanza tree */
	meta->addChild(transducer);
	meta->addChild(property);
	item->addChild(meta);
	publish->addChild(item);
	pubsub->addChild(publish);
	iq->addChild(pubsub);

	/* build the transaction */
	trans->setRequest(iq);
	trans->setHandler(handler_publish, (void *)this);

	/* send */
	xwork->send(trans);

	delete iq;
	delete pubsub;
	delete publish;
	delete item;
	delete meta;
	delete transducer;
	delete property;
}

void XMPPInterface::invite_respawn() {
	Transaction *trans;
	Stanza *iq, *pubsub, *affiliations, *affiliation;
	const char *pjid = pubsub_jid(jid);

	trans = new Transaction();

	iq           = new Stanza(xwork->getCTX());
	pubsub       = new Stanza(xwork->getCTX());
	affiliations = new Stanza(xwork->getCTX());
	affiliation  = new Stanza(xwork->getCTX());

	/* <iq type="set"/> */
	iq->setName("iq");
	iq->setType("set");
	iq->setAttribute("to", pjid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub"/> */
	pubsub->setName("pubsub");
	pubsub->setNS("http://jabber.org/protocol/pubsub#owner");

	/* <affiliations node='FORK017584350747'> */
	affiliations->setName("affiliations");
	affiliations->setAttribute("node", node);

	/* <affiliation jid='respawn@sensor.andrew.cmu.edu' affiliation='publisher'/> */
	affiliation->setName("affiliation");
	affiliation->setAttribute("jid", RESPAWN_JID);
	affiliation->setAttribute("affiliation", "publisher");

	/* build stanza tree */
	affiliations->addChild(affiliation);
	pubsub->addChild(affiliations);
	iq->addChild(pubsub);

	/* build the transaction */
	trans->setRequest(iq);
	trans->setHandler(handler_invite, (void *)this);

	/* send */
	xwork->send(trans);

	delete iq;
	delete pubsub;
	delete affiliations;
	delete affiliation;
}

void XMPPInterface::create() {
	Stanza *iq, *pubsub, *create;
	Transaction *trans = new Transaction();
	const char *pjid = pubsub_jid(jid);

	iq     = new Stanza(xwork->getCTX());
	pubsub = new Stanza(xwork->getCTX());
	create = new Stanza(xwork->getCTX());

	/* <iq type="set" to="$pjid"/> */
	iq->setName("iq");
	iq->setType("set");
	iq->setAttribute("to", pjid);

	/* <pubsub xmlns="http://jabber.org/protocol/pubsub"/> */
	pubsub->setName("pubsub");
	pubsub->setNS("http://jabber.org/protocol/pubsub");

	/* <create node="$node"/> */
	create->setName("create");
	create->setAttribute("node", node);
	create->setAttribute("name", "FORK");

	/* build stanza tree */
	pubsub->addChild(create);
	iq->addChild(pubsub);

	/* build the transaction */
	trans->setRequest(iq);
	trans->setHandler(handler_create, (void *) this);

	/* send */
	xwork->send(trans);

	print_info("create: Sent node creation stanza for %s\n", node);

	delete iq;
	delete create;
	delete pubsub;
	delete[] pjid;
}

void XMPPInterface::handler_create(Transaction* trans, void* userdata) {
	XMPPInterface *xinterface = (XMPPInterface *) userdata;
	if(strcmp(trans->getResponse()->getType(), "result") != 0) {
		xinterface->print_err("XMPPInterface: Could not create the node %s (may already be created)\n", xinterface->node);
	} else {
		xinterface->print_info("XMPPInterface: node %s created successfully\n", xinterface->node);
		/* after the node is created, publish the meta data */
		xinterface->publish_meta();
		xinterface->print_info("handler_create: Sent meta publication stanza for %s\n", xinterface->node);
	}
	delete trans;
}

void XMPPInterface::handler_publish(Transaction* trans, void* userdata) {
	XMPPInterface *xinterface = (XMPPInterface *) userdata;

	if(strcmp(trans->getResponse()->getType(), "result") != 0) {
		xinterface->print_err("XMPPInterface: Error - could not publish to the node %s\n", xinterface->node);
	} else {
		xinterface->print_dbg("XMPPInterface: Received publish success\n");
		if(strcmp(trans->getResponse()->getID(), "publish_meta") == 0) {
			/* after publishing meta data, invite respawn */
			xinterface->invite_respawn();
			xinterface->print_info("handler_publish: Sent respawn invite for %s\n", xinterface->node);
		}
	}
	delete trans;
}

void XMPPInterface::handler_invite(Transaction* trans, void* userdata) {
	XMPPInterface *xinterface = (XMPPInterface *) userdata;
	if(strcmp(trans->getResponse()->getType(), "result") != 0) {
		xinterface->print_err("XMPPInterface: Failed to invite respawn as publisher to node %s\n", xinterface->node);
	} else {
		xinterface->print_info("XMPPInterface: Respawn successfully invited to node %s\n", xinterface->node);
	}
	delete trans;
}

/**
 * We return a new char* allocated with new
 * @param jid The jid that we will derive the pubsub jid from
 * @return A new allocated pubsub jid string buffer
 */
char* XMPPInterface::pubsub_jid(const char* jid) {
	/* determine pubsub_jid */
	// alloc/copy jid to pubsub_jid
	char *pubsub_jid = new char[strlen(jid) + 7 + 1];
	if (pubsub_jid == NULL) {
		print_err("Error - Could not determine pubsub jid\n");
		return NULL;
	}
	strcpy(pubsub_jid, "pubsub.");
	strcpy(&pubsub_jid[7], strchr(jid, '@') + 1);

	// terminate at optional resource specifier
	if (strchr(pubsub_jid, '/')) {
		*strchr(pubsub_jid, '/') = '\0';
	}
	return pubsub_jid;
}

/**
 * Gets the current time and date and returns them as a string.
 *
 * @returns Pointer to a string containing the current time and date. String needs to be freed when no longer in use.
 * @note This is taken from libMIO's mio_node.c file
 */
char* XMPPInterface::create_timestamp() {
	char fmt[64], buf[64];
	struct timeval tv;
	struct tm *tm;
	char *time_str = NULL;

	gettimeofday(&tv, NULL);
	if ((tm = localtime(&tv.tv_sec)) != NULL) {
		strftime(fmt, sizeof fmt, "%Y-%m-%dT%H:%M:%S.%%06u%z", tm);
		time_str = new char[strlen(fmt) + 7];
		snprintf(time_str, sizeof(buf), fmt, tv.tv_usec);
		return time_str;
	}
	return NULL;
}
