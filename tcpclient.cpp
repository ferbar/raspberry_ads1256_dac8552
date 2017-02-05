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
 *
 * client - thread - part (für jeden client ein thread)
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <assert.h>
#include <stdexcept>

// für setsockopt
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include <errno.h>

#include "utils.h"
#include "server.h"
#include "tcpclient.h"

int TCPClient::numClients=0;

int cfg_tcpTimeout=100000;

void TCPClient::prepareMessage()
{
	int flag = 1;
	setsockopt(this->so, IPPROTO_TCP, TCP_CORK, (char *)&flag, sizeof(flag) ); // prepare message, stopsel rein
}

void TCPClient::flushMessage()
{
	int flag=0;
	setsockopt(this->so, IPPROTO_TCP, TCP_CORK, (char *)&flag, sizeof(flag) ); // message fertig, senden
}

/**
 * wartet bis daten daherkommen, macht exception wenn keine innerhalb von timeout gekommen sind
 */
void TCPClient::readSelect()
{
	struct timeval timeout;
	fd_set set;
	timeout.tv_sec=cfg_tcpTimeout; timeout.tv_usec=0;
	FD_ZERO(&set); FD_SET(this->so,&set);
	int rc;
	if((rc=select(this->so+1, &set, NULL, NULL, &timeout)) <= 0) {
		if(rc != 0) {
			throw std::runtime_error("error select");
		}
		printf("ClientThread::readSelect error in select(%d) %s\n", this->so, strerror(errno));
		throw std::runtime_error("timeout reading cmd");
	}
}


/**
 * destruktor
 */
TCPClient::~TCPClient()
{
	close(this->so);
}
