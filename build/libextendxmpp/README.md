This project houses the C/C++ libraries that extend libstrophe.

Current Features:
- Stanza - A C++ binding for libstrophe stanzas
- StanzaQueue - A class representing a queue of Stanzas
- Transaction - A class facilitating sending a request to the xmpp server, receiving a response, and calling the registered handler.
- TransactionQueue - A class representing a queue of Transactions
- XMPPWorker - A multithreaded wrapper for libstrophe that connects and sends/handles Transactions
- PubSub - The old libstrophe PubSub library
