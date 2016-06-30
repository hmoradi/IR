# Author: Craig Hesling <craig@hesling.com>
# Date: Jun 21, 2015

cmake_minimum_required (VERSION 2.6)
project(libextendxmpp)

set(LIBEXTENDXMPP_SOURCES
		#PubSub.cc
		Stanza.cc
		StanzaQueue.cc
		Transaction.cc
		TransactionQueue.cc
		XMPPWorker.cc
		XMPPUtils.cc
	)

set(LIBEXTENDXMPP_HEADERS
		#PubSub.h
		Stanza.h
		StanzaQueue.h
		Transaction.h
		TransactionQueue.h
		XMPPWorker.h
		XMPPUtils.h
	)

add_compile_options(-static)

include_directories(.)
add_library(extendxmpp STATIC ${LIBEXTENDXMPP_SOURCES} ${LIBEXTENDXMPP_HEADERS})
# Link to libstrophe and pthread
target_link_libraries(extendxmpp strophe)
target_link_libraries(extendxmpp pthread)
#find_package(pthread)

add_executable(TransactionQueue_test ${PROJECT_SOURCE_DIR}/TransactionQueue_test.cc)
add_executable(XMPPWorker_test       ${PROJECT_SOURCE_DIR}/XMPPWorker_test.cc)
add_executable(XMPPUtils_test        ${PROJECT_SOURCE_DIR}/XMPPUtils_test.cc)

target_link_libraries(TransactionQueue_test extendxmpp)
target_link_libraries(XMPPWorker_test       extendxmpp)
target_link_libraries(XMPPUtils_test        extendxmpp)
