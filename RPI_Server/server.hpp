#ifndef SERVER_HPP
#define SERVER_HPP

// C++ standard library
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <queue>
#include <string>
#include <algorithm>
#include <unordered_map>

// Boost library
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>

// Includes for PIGPIO library
#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <pigpio.h>

// Defines
#define RASPBERRY_ADDR 0
#define STD_MSG_LENGHT  4
#define LISTENING_PORT 17000
#define MAX_REC_LENGTH 20
#define REC_LENGTH 6
#define MAX_D 255
#define BUFF_TIME_SEC 60
#define BUFF_EST_LENGTH 60*100*15
#define TS 0.01

// Some namespaces
using namespace std;
using namespace boost::asio;
using namespace std::chrono;
using ip::tcp;

///////////////////////////////////////////////////////////////////////////////
//  Data structures
///////////////////////////////////////////////////////////////////////////////
// timed_values - To hold timed values in the buffer
typedef struct timed_values{
    steady_clock::time_point time;
    float value;
}timed_values;
// i2c_msg - To keep keep the message to be inserted in the queue
typedef struct i2c_msg{
    char bytes[STD_MSG_LENGHT];
    steady_clock::time_point t;
}i2c_msg;


///////////////////////////////////////////////////////////////////////////////
//  Class: Read_I2C - Implements an higher abstraction level to the I2C slave
// implementation for monitoring the bus. Constructor initializes GPIO
// and setup the I2C communication.
///////////////////////////////////////////////////////////////////////////////
class Read_I2C{
private:
    static int obj_count;
    bool valid;
    bsc_xfer_t xfer;
    void init_slave(int addr);
    void close_slave(void);
public:
    Read_I2C() = delete;
    Read_I2C(int addr);
    ~Read_I2C();
    bool read_message(char msg[]);
    bool const is_valid(){return valid;};
};

///////////////////////////////////////////////////////////////////////////////
//  Class: Luminaires - Use to implement data storage. Safely keeps data and
// provides functions that regulate the access to data.
///////////////////////////////////////////////////////////////////////////////
class Luminaires{
private:
    steady_clock::time_point start;
    unordered_map<std::string,float> data;
    unordered_map<std::string,std::deque<timed_values>> last_min;
    bool valid = false;
    void remove_older_than_1min(std::deque<timed_values> *l);
    static bool is_available(int id);
public:
    bool is_valid();
    void reset(i2c_msg msg1,i2c_msg msg2);
    void update_values(i2c_msg m);
    bool request_is_valid(std::string request);
    std::string request_response(std::string request);
};

///////////////////////////////////////////////////////////////////////////////
//  Class: Connection - Each instance of this class represents a TCP client
// connection. It owns a socket associated with the connection, and handles
// client requests.
///////////////////////////////////////////////////////////////////////////////
class Connection : public boost::enable_shared_from_this<Connection>{
private:
    static int count;
    int id;
    tcp::socket sock;
    char request[MAX_REC_LENGTH-1];
    std::string previous_request;
    bool flag_stream = false;
public:
    typedef boost::shared_ptr<Connection> pointer;
    Connection(io_service &io):sock(io){ count++; id=count; };
    void start_read();
    void read();
    void handle_request(std::string request);
    void send_message(string msg);
    tcp::socket& socket();
    static pointer create_new(io_service &io){ return pointer(new Connection(io)); }
};

///////////////////////////////////////////////////////////////////////////////
//  Class: Server - This class implements the server, waiting for client
// connections. It owns an acceptor, which asynchronously accepts connections
// and creates a new instance of class Connection. (Note: This is fully
// defined in this header)
///////////////////////////////////////////////////////////////////////////////
class Server{
private:
    tcp::acceptor acceptor;
    void do_accept(){
        Connection::pointer connection = Connection::create_new(acceptor.get_io_service());
        acceptor.async_accept(connection->socket(),
            [this,connection](boost::system::error_code ec){
                if(!ec)
                    connection->start_read();
                do_accept();
            });
    }
public:
    Server(io_service &io,int port_):acceptor(io,tcp::endpoint(tcp::v4(),port_)){
        do_accept();
    }
};

///////////////////////////////////////////////////////////////////////////////
// Global variable declaration
///////////////////////////////////////////////////////////////////////////////
extern Luminaires Data;
extern mutex mut[2];
extern mutex rw[2];
extern condition_variable cv;
extern bool avail[2];


#endif
