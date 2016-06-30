/** @file Stanza.h
 *
 *  @author Craig Hesling (craig@hesling.com)
 *  @date July 6, 2015
 */

#ifndef STANZA_H_
#define STANZA_H_

//namespace StropheCPP {

#include <string>
#include <strophe.h>

class Stanza {
public:
	Stanza(xmpp_ctx_t *ctx);
	Stanza(xmpp_stanza_t *stanza, bool clone=true);
	/** Create a new Stanza container from an existing Stanza */
	Stanza(Stanza *stanza, bool clone=true);
	~Stanza();

	xmpp_stanza_t *getStanza();

	void setStanza(xmpp_stanza_t *stanza, bool clone=true);

	/** Set the name of a stanza.
	 *
	 *  @param name a string with the name of the stanza
	 *
	 *  @return XMPP_EOK on success, a number less than 0 on failure (XMPP_EMEM,
	 *      XMPP_EINVOP)
	 */
	int setName(const char *name);

	/** Set an attribute for a stanza object.
	 *
	 *  @param key a string with the attribute name
	 *  @param value a string with the attribute value
	 *
	 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
	 */
	int setAttribute(const char * const key, const char * const value);

	/** Set the 'id' attribute of a stanza.
	 *
	 *  This is a convenience function for:
	 *  xmpp_stanza_set_attribute(stanza, 'id', id);
	 *
	 *  @param stanza a Strophe stanza object
	 *  @param id a string containing the 'id' value
	 *
	 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
	 */
	int setID(const char * const id);

	int setIDRandom();

	/** Set the 'type' attribute of a stanza.
	 *  This is a convenience function for:
	 *  xmpp_stanza_set_attribute(stanza, 'type', type);
	 *
	 *  @param stanza a Strophe stanza object
	 *  @param type a string containing the 'type' value
	 *
	 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
	 */
	int setType(const char * const type);

	/** Set the stanza namespace.
	 *  This is a convenience function equivalent to calling:
	 *  xmpp_stanza_set_attribute(stanza, "xmlns", ns);
	 *
	 *  @param stanza a Strophe stanza object
	 *  @param ns a string with the namespace
	 *
	 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
	 */
	int setNS(const char * const ns);

	/** Set the text data for a text stanza.
	 *  This function copies the text given and sets the stanza object's text to
	 *  it.  Attempting to use this function on a stanza that has a name will
	 *  fail with XMPP_EINVOP.  This function takes the text as a null-terminated
	 *  string.
	 *
	 *  @param stanza a Strophe stanza object
	 *  @param text a string with the text
	 *
	 *  @return XMPP_EOK (0) on success or a number less than zero on failure
	 */
	int setText(const char * const text);

	bool isNull();
	bool isValid();
	char *getName();
	char *getID();
	char *getType();
	char *getText();
	char *getAttribute(const char *key);
	Stanza *getChildByName(const char *name, bool clone=true);
	Stanza *getChildByNS(const char *ns, bool clone=true);
	Stanza *getChildren(bool clone=true);
	Stanza *getNext(bool clone=true);

	/**
	 * Sets the stanza to the new stanza requested by name.
	 * The clone reference setting set upon initialization
	 *  is maintained to new stanza.
	 * @param name The name of the child
	 */
	void gotoChildByName(const char *name);
	void gotoChildByNS(const char *ns);
	void gotoChildren();
	void gotoNext();

	int addChild(Stanza *stanza);

	Stanza *clone();
	std::string toText();

	/*
	 * These operators compare the inner stanza pointer
	 */

	inline bool operator==(const Stanza& rhs);
	inline bool operator==(const xmpp_stanza_t *rhs);
//	inline bool operator==(const xmpp_stanza_t *lhs, const Stanza& rhs);
	inline bool operator!=(const Stanza& rhs);
	inline bool operator!=(const xmpp_stanza_t *rhs);
//	inline bool operator!=(const xmpp_stanza_t *lhs, const Stanza& rhs);

protected:
	/** The real stanza object we represent */
	xmpp_stanza_t *stanza;
	/** Indicate whether the stanza needs to be released upon deleting */
	bool           stanza_release;
};

//}

#endif /* STANZA_H_ */
