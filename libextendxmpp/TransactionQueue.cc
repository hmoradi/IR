/**@file TransactionQueue.cc
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 */

#include <cstring>
#include <cassert>
#include <cstddef> /* NULL */
#include "TransactionQueue.h"

TransactionQueue::TransactionQueue() {
}

TransactionQueue::~TransactionQueue() {
}

void TransactionQueue::enqueue(Transaction* trans) {
	queue.push_back(trans);
}

Transaction* TransactionQueue::dequeue() {
	Transaction* trans = NULL;
	if(queue.size() > 0) {
		trans = queue.front();
		queue.pop_front();
	}
	return trans;
}

Transaction* TransactionQueue::peek() {
	return (queue.size() > 0) ? queue.front() : NULL;
}

/**
 * Remove the stanza pointed to by \a stanza from the queue
 * @param stanza The stanza to be removed
 * @return true upon successfully removing \a stanza,
 *         false if \a stanza cannot be found
 */
bool TransactionQueue::remove(Transaction* stanza) {
	std::list<Transaction*>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		if (*it == stanza) {
			queue.erase(it);
			return true;
		}
	}
	return false;
}

size_t TransactionQueue::size() {
	return queue.size();
}

Transaction* TransactionQueue::findByID(const char* id) {
	std::list<Transaction*>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++) {
		Transaction* trans = *it;
		char *cmp;

		if(!trans->getRequest()->isValid()) {
			continue;
		}

		cmp = trans->getRequest()->getID();
		if(cmp == NULL) {
			continue;
		}

		if (strcmp(cmp, id) == 0) {
			return trans;
		}
	}
	return 0;
}
