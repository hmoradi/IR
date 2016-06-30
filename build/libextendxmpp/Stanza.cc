/** @file Stanza.cc
 *
 *  @author Craig Hesling (craig@hesling.com)
 *  @date July 6, 2015
 */

#include <cassert>
#include <string>
#include <strophe.h>
#include <time.h> /* only for setIDRandom */

#include "Stanza.h"

/*
 * This forward declaration is a hack to be able
 * to run xmpp_free(ctx, ptr) inside this class without
 * needing to provide the ctx for the toText method.
 * This is off that libstrophe provides the to_text
 * function that allocates memory for the string from
 * the embedded stanza->ctx, but then requires you
 * to call xmpp_free with the ctx as an argument.
 */
typedef enum {
    XMPP_STANZA_UNKNOWN,
    XMPP_STANZA_TEXT,
    XMPP_STANZA_TAG
} xmpp_stanza_type_t;

struct _xmpp_stanza_t {
    int ref;
    xmpp_ctx_t *ctx;

    xmpp_stanza_type_t type;

    xmpp_stanza_t *prev;
    xmpp_stanza_t *next;
    xmpp_stanza_t *children;
    xmpp_stanza_t *parent;

    char *data;

    //hash_t *attributes;
};


/** Unique value for random IDs */
static long int next_id = 0;

//namespace StropheCPP {

Stanza::Stanza(xmpp_ctx_t *ctx) {
	stanza_release = true;
	stanza = xmpp_stanza_new(ctx);
}

Stanza::Stanza(xmpp_stanza_t *stanza, bool clone/*=true*/) {
	this->stanza_release = clone;

	if (clone) {
		/* increase reference count */
		this->stanza = xmpp_stanza_clone(stanza);
	} else {
		/* do not increase ref count */
		this->stanza = stanza;
	}
}

/**
 * Create a new Stanza (container) from the \a xmpp_stanza_t of another Stanza
 * @param[in] stanza The Stanza to initialize from
 * @param[in] clone Whether to clone the inner stanza
 */
Stanza::Stanza(Stanza *stanza, bool clone/*=true*/) {
	this->stanza_release = clone;

	if (clone) {
		/* increase reference count */
		this->stanza = xmpp_stanza_clone(stanza->getStanza());
	} else {
		/* do not increase ref count */
		this->stanza = stanza->getStanza();
	}
}

Stanza::~Stanza() {
	if (stanza_release) {
		xmpp_stanza_release(stanza);
	}
}

xmpp_stanza_t *Stanza::getStanza() {
	return stanza;
}

void Stanza::setStanza(xmpp_stanza_t *stanza, bool clone/*=true*/) {
	/* release old stanza if needed */
	if (this->stanza) {
		if (stanza_release) {
			xmpp_stanza_release(this->stanza);
		}
	} else {
		/* sanity check that no one thought they needed to release a bad stanza */
		assert(stanza_release == false);
	}

	/* setup new stanza - clone if requested */
	if (clone) {
		// allow NULL stanzas
		if (stanza) {
			stanza_release = clone;
			this->stanza = xmpp_stanza_clone(stanza);
		} else {
			stanza_release = false;
			this->stanza = NULL;
		}
	} else {
		stanza_release = clone;
		this->stanza = stanza;
	}
}

/** Set the name of a stanza.
 *
 *  @param name a string with the name of the stanza
 *
 *  @return XMPP_EOK on success, a number less than 0 on failure (XMPP_EMEM,
 *      XMPP_EINVOP)
 */
int Stanza::setName(const char *name) {
	return xmpp_stanza_set_name(stanza, name);
}

/** Set an attribute for a stanza object.
 *
 *  @param key a string with the attribute name
 *  @param value a string with the attribute value
 *
 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
 */
int Stanza::setAttribute(const char * const key, const char * const value) {
	return xmpp_stanza_set_attribute(stanza, key, value);
}

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
int Stanza::setID(const char * const id) {
	return xmpp_stanza_set_id(stanza, id);
}

/**
 * Sets the id to a random string. Currently this is "strophe"
 * followed by the unix epoch time value.
 * Example: "strophe1436379908"
 * @return Same return as @ref Stanza::setID
 */
int Stanza::setIDRandom() {
	/* min size 7+20+1 */
	char id[100];
	time_t t = time((time_t *)0);
	assert(t != ((time_t) -1));

	snprintf(id, sizeof(id), "strophe%lu%lu", next_id++, (long int)t);
	return setID(id);
}

/** Set the 'type' attribute of a stanza.
 *  This is a convenience function for:
 *  xmpp_stanza_set_attribute(stanza, 'type', type);
 *
 *  @param stanza a Strophe stanza object
 *  @param type a string containing the 'type' value
 *
 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
 */
int Stanza::setType(const char * const type) {
	return xmpp_stanza_set_type(stanza, type);
}

/** Set the stanza namespace.
 *  This is a convenience function equivalent to calling:
 *  xmpp_stanza_set_attribute(stanza, "xmlns", ns);
 *
 *  @param stanza a Strophe stanza object
 *  @param ns a string with the namespace
 *
 *  @return XMPP_EOK (0) on success or a number less than 0 on failure
 */
int Stanza::setNS(const char * const ns) {
	return xmpp_stanza_set_ns(stanza, ns);
}

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
int Stanza::setText(const char * const text) {
	return xmpp_stanza_set_text(stanza, text);
}

bool Stanza::isNull() {
	return (this && stanza) ? false : true;
}

bool Stanza::isValid() {
	return !isNull();
}

char *Stanza::getName() {
	assert(stanza);
	return xmpp_stanza_get_name(stanza);
}

char *Stanza::getID() {
	assert(stanza);
	return xmpp_stanza_get_id(stanza);
}

char *Stanza::getType() {
	assert(stanza);
	return xmpp_stanza_get_type(stanza);
}

char *Stanza::getText() {
	assert(stanza);
	return xmpp_stanza_get_text(stanza);
}

char *Stanza::getAttribute(const char *key) {
	assert(stanza);
	return xmpp_stanza_get_attribute(stanza, key);
}

Stanza *Stanza::getChildByName(const char *name, bool clone/*=true*/) {
	assert(stanza);
	xmpp_stanza_t *s = xmpp_stanza_get_child_by_name(stanza, name);
	if (s) {
		return new Stanza(s, clone);
	} else {
		return NULL;
	}
}

Stanza *Stanza::getChildByNS(const char *ns, bool clone/*=true*/) {
	assert(stanza);
	xmpp_stanza_t *s = xmpp_stanza_get_child_by_ns(stanza, ns);
	if (s) {
		return new Stanza(s, clone);
	} else {
		return NULL;
	}
}

Stanza *Stanza::getChildren(bool clone/*=true*/) {
	assert(stanza);
	xmpp_stanza_t *s = xmpp_stanza_get_children(stanza);
	if (s) {
		return new Stanza(s, clone);
	} else {
		return NULL;
	}
}

Stanza *Stanza::getNext(bool clone/*=true*/) {
	assert(stanza);
	xmpp_stanza_t *s = xmpp_stanza_get_next(stanza);
	if (s) {
		return new Stanza(s, clone);
	} else {
		return NULL;
	}
}

/**
 * Sets the stanza to the new stanza requested by name.
 * The clone reference setting set upon initialization
 *  is maintained to new stanza.
 * @param name The name of the child
 */
void Stanza::gotoChildByName(const char *name) {
	setStanza(xmpp_stanza_get_child_by_name(stanza, name), stanza_release);
}

void Stanza::gotoChildByNS(const char *ns) {
	setStanza(xmpp_stanza_get_child_by_ns(stanza, ns), stanza_release);
}

void Stanza::gotoChildren() {
	setStanza(xmpp_stanza_get_children(stanza), stanza_release);
}

void Stanza::gotoNext() {
	setStanza(xmpp_stanza_get_next(stanza), stanza_release);
}

int Stanza::addChild(Stanza *stanza) {
	return xmpp_stanza_add_child(this->stanza, stanza->stanza);
}

Stanza *Stanza::clone() {
	return new Stanza(stanza, true);
}

std::string Stanza::toText() {
	std::string output;
	char *buf;
	size_t buf_len;

	xmpp_stanza_to_text(stanza, &buf, &buf_len);
	output = std::string(buf);
	xmpp_free((const xmpp_ctx_t *)stanza->ctx, buf);
	return output;
}

/*
 * These operators compare the inner stanza pointer
 */

inline bool Stanza::operator==(const Stanza& rhs) {
	return stanza == rhs.stanza;
}
inline bool Stanza::operator==(const xmpp_stanza_t *rhs) {
	return stanza == rhs;
}
//	inline bool Stanza::operator==(const xmpp_stanza_t *lhs, const Stanza& rhs) {
//		return lhs==rhs.stanza;
//	}
inline bool Stanza::operator!=(const Stanza& rhs) {
	return !(*this == rhs);
}
inline bool Stanza::operator!=(const xmpp_stanza_t *rhs) {
	return !(*this == rhs);
}
//	inline bool Stanza::operator!=(const xmpp_stanza_t *lhs, const Stanza& rhs) {
//		return !(lhs == rhs);
//	}

//}
