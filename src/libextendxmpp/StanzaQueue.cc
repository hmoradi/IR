/** @file StanzaQueue.cc
 *
 *  @author Craig Hesling (craig@hesling.com)
 *  @date July 6, 2015
 */

#include <cstring>
#include <cstddef> /* NULL */
#include "StanzaQueue.h"

StanzaQueue::StanzaQueue() {
}

StanzaQueue::~StanzaQueue() {
}

void StanzaQueue::enqueue(Stanza* stanza) {
	queue.push_back(stanza);
}

Stanza* StanzaQueue::dequeue() {
	Stanza *stanza = NULL;
	if(queue.size() > 0) {
		stanza = queue.front();
		queue.pop_front();
	}
	return stanza;
}

Stanza* StanzaQueue::peek() {
	return (queue.size() > 0) ? queue.front() : NULL;
}

/**
 * Remove the stanza pointed to by \a stanza from the queue
 * @param stanza The stanza to be removed
 * @return true upon successfully removing \a stanza,
 *         false if \a stanza cannot be found
 */
bool StanzaQueue::remove(Stanza* stanza) {
	std::list<Stanza *>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		if (*it == stanza) {
			queue.erase(it);
			return true;
		}
	}
	return false;
}

size_t StanzaQueue::size() {
	return queue.size();
}

Stanza* StanzaQueue::findByAttribute(const char* key, const char* value) {
	std::list<Stanza *>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		Stanza *stanza = *it;
		char *cmp;

		if (!stanza->isValid()) {
			continue;
		}

		cmp = stanza->getAttribute(key);
		if(cmp == NULL) {
			continue;
		}

		if (strcmp(cmp, value) == 0) {
			return stanza;
		}
	}
	return 0;
}

Stanza* StanzaQueue::findByName(const char* name) {
	std::list<Stanza *>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		Stanza *stanza = *it;
		char *cmp;

		if (!stanza->isValid()) {
			continue;
		}

		cmp = stanza->getName();
		if(cmp == NULL) {
			continue;
		}

		if (strcmp(cmp, name) == 0) {
			return stanza;
		}
	}
	return 0;
}

Stanza* StanzaQueue::findByID(const char* id) {
	std::list<Stanza *>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		Stanza *stanza = *it;
		char *cmp;

		if (!stanza->isValid()) {
			continue;
		}

		cmp = stanza->getID();
		if(cmp == NULL) {
			continue;
		}

		if (strcmp(cmp, id) == 0) {
			return stanza;
		}
	}
	return 0;
}

Stanza* StanzaQueue::findByType(const char* type) {
	std::list<Stanza *>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		Stanza *stanza = *it;
		char *cmp;

		if (!stanza->isValid()) {
			continue;
		}

		cmp = stanza->getType();
		if(cmp == NULL) {
			continue;
		}

		if (strcmp(cmp, type) == 0) {
			return stanza;
		}
	}
	return 0;
}
