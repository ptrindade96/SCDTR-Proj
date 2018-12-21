#include "server.hpp"

// Global Class initialization
Luminaires Data;

// Syncronization Variables
bool avail[2] = {false,false};
mutex mut[2];
condition_variable cv;
mutex rw[2];

// Queue to temporarily hold the messages
std::queue<i2c_msg> queue_i2c;

///////////////////////////////////////////////////////////////////////////////
//  thread_server() - This thread runs the server
///////////////////////////////////////////////////////////////////////////////
void thread_server(){
    boost::asio::io_service io;
    Server server(io,LISTENING_PORT);
    io.run();
}

///////////////////////////////////////////////////////////////////////////////
//  thread_read_i2c() - This thread actively waits for new messages and
//  puts them on a queue, for further processing
///////////////////////////////////////////////////////////////////////////////
void thread_read_i2c(){
	Read_I2C rpi_i2c(RASPBERRY_ADDR);
	i2c_msg msg;

    std::cout << "Raspberry slave address = " << RASPBERRY_ADDR << '\n';

	while(rpi_i2c.is_valid())
		if(rpi_i2c.read_message(msg.bytes)){
            msg.t = steady_clock::now();
            queue_i2c.push(msg);
        }

    std::cout << "Error in I2C communications" << '\n';
}

///////////////////////////////////////////////////////////////////////////////
//  thread_treat_i2c() - This thread reads new values from the queue and
//  updates the data structure.
///////////////////////////////////////////////////////////////////////////////
void thread_treat_i2c(){
	i2c_msg msg1;
    i2c_msg msg2;
	while(true)
		while(!queue_i2c.empty()){
			msg1 = queue_i2c.front();
			queue_i2c.pop();
            if(msg1.bytes[0]&0b00000100 and Data.is_valid())
                Data.update_values(msg1);
            if(!(msg1.bytes[0]&0b00000100)){
                while(queue_i2c.empty()){}
                msg2 = queue_i2c.front();
                queue_i2c.pop();
                Data.reset(msg1,msg2);
                std::cout << "Reset Detected" << '\n';
            }
		}
}

///////////////////////////////////////////////////////////////////////////////
//  main()
///////////////////////////////////////////////////////////////////////////////
using namespace std;
int main(int argc, char const *argv[]){
    cout << "--- Starting Server ---" << '\n';

    thread read_i2c{thread_read_i2c};
    thread treat_i2c{thread_treat_i2c};
    thread server{thread_server};

    read_i2c.join();
    treat_i2c.join();
    server.join();

    cout << "Server is exiting...\nGoodbye :)\n";

    return 0;
}
