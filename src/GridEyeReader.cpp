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
#include "GridEyeReader.h"

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

