/**@file XMPPInterface.h
 *
 * @date Jul 14, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#ifndef XMPPINTERFACE_H_
#define XMPPINTERFACE_H_

#include <pthread.h>
#include <XMPPWorker.h>

#define IGNORE -50

class XMPPInterface {
public:
	XMPPInterface(	const char *jid,
					const char *pass,
					const char *node,
					const char *parent_node,
					bool enable_distributed_coordination = false,
					bool enable_actuation = false, 
					int log_level = 2,
					unsigned short altport = 0,
					const char * const altdomain = 0);
	void occupancyChange(int count, int offset);
	int get_actuation_count();
	virtual ~XMPPInterface();
private:
	const char *jid;
	const char *pass;
	const char *node;
	const char *parent_node;
	int log_level;
	unsigned short altport;
	const char * const altdomain;
	XMPPWorker *xwork;
	void print_err(const char *fmt, ...);
	void print_info(const char *fmt, ...);
	void print_dbg(const char *fmt, ...);
	char *pubsub_jid(const char *jid);
	char* create_timestamp();
	void create();
	void publish_meta();
	void invite_respawn();
	static void handler_create(Transaction *trans, void *userdata);
	static void handler_publish(Transaction *trans, void *userdata);
	static void handler_invite(Transaction *trans, void *userdata);
	void sendMIOMessage(const char *node, const char *timestamp, int count);
	void sendJSONMessage(const char *node, const char *timestamp, int count, int offset);
	
};

#endif /* XMPPINTERFACE_H_ */
