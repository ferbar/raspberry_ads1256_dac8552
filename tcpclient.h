#ifndef TCPCLIENT_H
#define TCPCLIENT_H

class TCPClient {
public:
	TCPClient(int id, int so) : so(so), clientID(id) {
		numClients++; // sollte atomic sein
	};
	virtual ~TCPClient();
	virtual void run()=0;
	void readSelect();
	void prepareMessage();
	void flushMessage();

	int so;
	// ID vom client
	int clientID;
	// anzahl clients die gerade laufen
	static int numClients;
};

#endif
