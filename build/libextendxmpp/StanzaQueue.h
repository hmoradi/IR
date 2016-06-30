/** @file StanzaQueue.h
 *
 *  @author Craig Hesling (craig@hesling.com)
 *  @date July 6, 2015
 */

#ifndef STANZAQUEUE_H_
#define STANZAQUEUE_H_

#include "Stanza.h"
#include <list>

class StanzaQueue {
public:
	StanzaQueue();
	virtual ~StanzaQueue();
	void enqueue(Stanza *stanza);
	Stanza *dequeue();
	Stanza *peek();
	bool remove(Stanza *stanza);
	size_t size();
	Stanza *findByAttribute(const char *key, const char *value);
	Stanza *findByName(const char *name);
	Stanza *findByID(const char *id);
	Stanza *findByType(const char *id);
private:
	std::list<Stanza *> queue;
};

#endif /* STANZAQUEUE_H_ */
