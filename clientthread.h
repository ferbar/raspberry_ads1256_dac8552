#ifndef CLIENTTHREAD_H
#define CLIENTTHREAD_H

#include <string>
#include "tcpclient.h"

#define MAX_MESSAGE_SIZE 10000

class ClientThread : public TCPClient {
public:
	ClientThread(int id, int so) : TCPClient(id, so) {
	};
	virtual ~ClientThread();
	virtual void run();
	void sendMessage(const std::string &message);

};

#endif
