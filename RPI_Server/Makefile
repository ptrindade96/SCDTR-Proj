all: server clean_o

clean_o: main.o Luminaires.o Connection.o
	rm *.o

server: server.hpp main.o Luminaires.o Connection.o Read_I2C.o
	g++ -Wall -std=c++11 main.o Luminaires.o Connection.o Read_I2C.o -lpthread -lrt -lpigpio -o server -lboost_system

Luminaires.o: Luminaires.cpp
	g++ -Wall -std=c++11 -c Luminaires.cpp

Connection.o: Connection.cpp
	g++ -Wall -std=c++11 -c Connection.cpp

Read_I2C.o: Read_I2C.cpp
	g++ -Wall -std=c++11 -c Read_I2C.cpp

main.o: main.cpp
	g++ -Wall -std=c++11 -c main.cpp

clean:
	rm server
