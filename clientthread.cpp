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
#include "clientthread.h"


void ClientThread::sendMessage(const std::string &msg)
{
	int msgsize=msg.size();
	printf("%d:  sendMessage size: %zu+4 %s\n", this->clientID, msg.size(), msg.c_str());
	this->prepareMessage();
	write(this->so, &msgsize, 4);
	write(this->so, msg.data(), msg.size());
	this->flushMessage();
}

/**
 * für jedes handy ein eigener thread...
 */
void ClientThread::run()
{

	printf("%d:socket accepted sending welcome msg\n",this->clientID);
	std::string heloReply="raspi adc\n";
	sendMessage(heloReply);

	printf("%d:hello done, enter main loop\n",this->clientID);
	while(1) {

		int msgsize=0;
		int rc;
		this->readSelect(); // auf daten warten, macht exception wenn innerhalb vom timeout nix kommt
		if((rc=read(this->so, &msgsize, 4)) != 4) {
			throw std::runtime_error("error reading cmd: " + rc);
		}
		// printf("%d:reading msg.size: %d bytes\n",this->clientID,msgsize);
		if(msgsize < 0 || msgsize > MAX_MESSAGE_SIZE) {
			throw std::runtime_error("invalid size msgsize 2big");
		}
		char buffer[msgsize];
		this->readSelect();
		if((rc=read(this->so, buffer, msgsize)) != msgsize) {
			throw std::runtime_error("error reading cmd.data: " + rc );
		}
		std::string message(buffer,msgsize);
		
		if(cfg_debug) {
			printf("%d: msg %s\n", this->clientID, message.c_str());
		}
		switch (message.at(0)) {
			case 's': { 
				break; }
			case 'g': { 
				break; }
			default: {
				throw std::runtime_error("invalid command ("+message+")");
				break; }
		}


	}
	printf("%d:client exit\n",this->clientID);
}

/**
 * destruktor, schaut ob er der letzte clientThread war, wenn ja dann alle loks notstoppen
 */
ClientThread::~ClientThread()
{
	printf("%d:~ClientThread numClientd=%d\n",this->clientID, this->numClients);
	if(this->numClients == 0) {
	}
}
