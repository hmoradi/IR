/**@file TransactionQueue.h
 *
 * @date Jul 15, 2015
 * @author Craig Hesling <craig@hesling.com>
 *
 */

#ifndef TRANSACTIONQUEUE_H_
#define TRANSACTIONQUEUE_H_

#include <list>
#include "Transaction.h"

/**
 * @note We try to do the C++ pointer star on the type identifier
 *       in this class declaration and definition.
 */
class TransactionQueue {
public:
	TransactionQueue();
	virtual ~TransactionQueue();

	void enqueue(Transaction* trans);
	Transaction* dequeue();
	Transaction* peek();
	bool remove(Transaction* trans);
	size_t size();
	Transaction* findByID(const char* id);
private:
	std::list<Transaction*> queue;
};

#endif /* TRANSACTIONQUEUE_H_ */
