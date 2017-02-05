#ifndef SERVER_H
#define SERVER_H

#include <pthread.h>
#include <map>
#include <string>

class Server
{
public:
	Server(int port);
	virtual ~Server() {};
	virtual int accept();
	void run();
	static void setExit();
	static void waitExit();
	static void clientDone(int clientID);
	static bool isrunning;
private:
	int tcp_so;
	// für IDs
	int clientID_counter;
	// mapping clientID => pthreadID
	static bool exit;
	static std::map<int,pthread_t> clients;
};

struct startupdata_t {
	int clientID;
	int so;
};

#endif
