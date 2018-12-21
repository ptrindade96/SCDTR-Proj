#ifndef SERVER_HPP
#define SERVER_HPP

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
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <pigpio.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>

#define RASPBERRY_ADDR 0
#define STD_MSG_LENGHT  4
#define LISTENING_PORT 17000
#define MAX_REC_LENGTH 20
#define REC_LENGTH 6
#define MAX_D 255
#define BUFF_TIME_SEC 60
#define BUFF_EST_LENGTH 60*100*15
#define TS 0.1

using namespace std;
using namespace boost::asio;
using namespace std::chrono;
using ip::tcp;

///////////////////////////////////////////////////////////////////////////////
// Structure: Data structures
///////////////////////////////////////////////////////////////////////////////
typedef struct timed_values{
    steady_clock::time_point time;
    float value;
}timed_values;

typedef struct i2c_msg{
    char bytes[STD_MSG_LENGHT];
    steady_clock::time_point t;
}i2c_msg;

///////////////////////////////////////////////////////////////////////////////
// Class: Read_I2C
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
// Class: Luminaires
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
// Class: Connection
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
    static pointer create_new(io_service &io){  return pointer(new Connection(io)); }
};

///////////////////////////////////////////////////////////////////////////////
// Class: Server
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
// Global Variables
///////////////////////////////////////////////////////////////////////////////
extern Luminaires Data;

extern mutex mut[2];
extern mutex rw[2];

extern condition_variable cv;
extern bool avail[2];

#endif
