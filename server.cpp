/*
 *  This file is part of btcontrol
 *
 *  Copyright (C) Christian Ferbar
 *
 *  btcontrol is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  btcontrol is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with btcontrol.  If not, see <http://www.gnu.org/licenses/>.
 *
 * TCP - server
 */
#include <sys/select.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <cxxabi.h>

#include "server.h"
#include "utils.h"
#include "clientthread.h"

#include <stdexcept>

bool Server::isrunning=false;
bool Server::exit=false;
std::map <int,pthread_t> Server::clients;

Server::Server(int port)
: clientID_counter(1)
{

	// init socket
	int on=1;
	int rc;

	struct sockaddr_in address;
	bzero((char *)&address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	this->tcp_so = socket(AF_INET, SOCK_STREAM, 0);
	if(this->tcp_so < 0) {
		perror("Error in socket ");
		abort();
	}
	setsockopt(this->tcp_so, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on));

	// potentialy not threadsafe !
	// struct hostent *hp=gethostbyname("localhost");
	struct hostent *hp=gethostbyname("0.0.0.0");
	if(! hp) {
		perror("invalid IP");
		abort();
	}

	memcpy((char *)&address.sin_addr,(char *)hp->h_addr,(size_t)hp->h_length);

	rc = bind(this->tcp_so, (struct sockaddr *)&address, sizeof(address));

	if (rc!=0)
	{
		perror("bind error");
		abort();
	}
	rc = ::listen(this->tcp_so, 20);
	if (rc!=0)
	{
		perror("Error in Listen");
		abort();
	}
}
 
int Server::accept()
{
	// assert(this->bt_so > 0); - keine BT hardware -> this->bt_so < 0
	assert(this->tcp_so > 0); // FD_SET mit -1 macht was hin

	fd_set fd;
	FD_ZERO(&fd);
#ifdef INCL_BT	
	if(this->bt_so > 0) {
		FD_SET(this->bt_so,&fd);
	}
#endif
	FD_SET(this->tcp_so,&fd);
	int rc=select(FD_SETSIZE, &fd, NULL, NULL, NULL);
	printf("Server::accept - select rc=%d errno:%s\n",rc,strerror(errno));
	if( rc == -1 ) {
		perror("select/accept error");
		return -1;
	}
#ifdef INCL_BT
	if(this->bt_so > 0 && FD_ISSET(this->bt_so,&fd)) {
		printf("bt connection\n");
		return BTServer::accept();
	} else
#endif
	if(FD_ISSET(this->tcp_so,&fd)) {
		printf("tcp connection (%d)\n",this->tcp_so);

		struct sockaddr_in addr2;
		socklen_t siz;
		siz = (socklen_t)sizeof(addr2);
		int csock = ::accept(this->tcp_so, (struct sockaddr *)&addr2, &siz);
		if(csock < 0) {
			perror("so->accept error:");
			throw std::runtime_error("so->accept error");
		}
		printf("socket: %d\n",csock);
		return csock;
	} else {
		printf("Puit?\n");
		abort();
	}
	/*
	   struct linger ling;
	   ling.l_onoff=1;
	   ling.l_linger=10;
	   setsockopt(csock, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling)); */

	perror("select/accept error");
	return -1;
}

/**
 * unregister handler
 */
static void unregisterPhoneClient(void *data)
{
	startupdata_t *startupData=(startupdata_t *)data;
	printf("%d:unregisterPhoneClient\n",startupData->clientID);
	Server::clientDone(startupData->clientID);
	free(startupData);
}

/**
 * für jedes handy ein eigener thread...
 */
static void *phoneClient(void *data)
{
	startupdata_t *startupData=(startupdata_t *)data;
	printf("%d:new client\n",startupData->clientID);
	pthread_cleanup_push(unregisterPhoneClient,data);

	try {
#ifdef INCL_X11
		if(cfg_X11) {
			ClientThreadX11 client(startupData->clientID, startupData->so);
			client.run();
		} else 
#endif
		{
			ClientThread client(startupData->clientID, startupData->so);
			client.run();
		}
	} catch(const char *e) {
		printf(ANSI_RED "%d: exception %s - client thread killed\n" ANSI_DEFAULT, startupData->clientID,e);
	} catch(std::RuntimeExceptionWithBacktrace &e) {
		printf(ANSI_RED "%d: Runtime Exception %s - client thread killed\n" ANSI_DEFAULT, startupData->clientID,e.what());
	} catch(std::exception &e) {
		printf(ANSI_RED "%d: exception %s - client thread killed\n" ANSI_DEFAULT, startupData->clientID,e.what());
	} catch (abi::__forced_unwind&) { // http://gcc.gnu.org/bugzilla/show_bug.cgi?id=28145
		printf(ANSI_RED "%d: forced unwind exception - client thread killed\n" ANSI_DEFAULT, startupData->clientID);
		// copy &paste:
		// printf("%d:client exit\n",startupData->clientID);
		// pthread_cleanup_pop(true);
		throw; // rethrow exeption bis zum pthread_create, dort isses dann aus
	}
	printf("%d:client exit\n",startupData->clientID);

	pthread_cleanup_pop(true);
	return NULL;
}

void Server::run()
{
	assert(Server::isrunning==false); // darf nur einmal gestartet wern
	Server::isrunning=true;
	while(1) {
		int nsk = this->accept();
		if(nsk < 0) { // fehler, kann auch z.b. interrupted system call sein
			if(this->exit) {
				Server::isrunning=false;
				break;
			}
			continue;
		}
	// client thread vorbereiten + starten
		startupdata_t *startupData=(startupdata_t*) calloc(sizeof(startupdata_t),1);
		startupData->clientID=this->clientID_counter++;
		startupData->so=nsk;
		pthread_t &newThread=Server::clients[startupData->clientID];
		bzero(&newThread,sizeof(newThread));
#ifdef NO_THREADS
		phoneClient((void *)startupData);
		printf("k8055 client func done: %lx\n",newThread);
#else
		if(int rc=pthread_create(&newThread, NULL, phoneClient, (void *)startupData) != 0) {
			printf("error creating new thread rc=%d\n",rc);
			perror("error creating new thread ");
		}
		printf("new Thread: %lx\n",newThread);
		pthread_detach(newThread);
#endif
	}
}

/**
 * setzt nur das exit - flag!
 */
void Server::setExit() {
	Server::exit=true;
	for (std::map<int,pthread_t>::iterator it=Server::clients.begin(); it!=Server::clients.end(); ++it) {
		if(pthread_cancel(it->second) != 0) {
			perror("error sending pthread cancel");
		}
	}
}

/**
 * wartet drauf dass alle threads weg sind:
 * mainThread geht nur weg wenna ein INTR bekommt (vom ctrl+c z.b.)
 */
void Server::waitExit() {
	printf("waiting for main thread exit\n");  // 
	while(Server::isrunning){
		printf(".");
		sleep(1);
	}
	// lock ??
	printf("waiting for client threads\n");
	while(true) {
		if(Server::clients.empty()) {
			break;
		}
		for (std::map<int,pthread_t>::iterator it=Server::clients.begin(); it!=Server::clients.end(); ++it) {
			printf("%d:%lu\n",it->first,it->second);
		}
		sleep(1);
	}
	printf("done\n");
}

void Server::clientDone(int clientID) {
	Server::clients.erase(clientID);
}

