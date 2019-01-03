#include "server.hpp"

using namespace boost::asio;
using ip::tcp;

int Connection::count = 0;
typedef boost::shared_ptr<Connection> pointer;

///////////////////////////////////////////////////////////////////////////////
//  Connection::socket() - Returns the socket associated to the connection
///////////////////////////////////////////////////////////////////////////////
tcp::socket& Connection::socket(){
    return sock;
}

///////////////////////////////////////////////////////////////////////////////
//  Connection::start_read() - Starts the connection
///////////////////////////////////////////////////////////////////////////////
void Connection::start_read(){
    std::cout << "Connected to client " << id << '\n';
    read();
}

///////////////////////////////////////////////////////////////////////////////
//  Connection::read() - Makes the async_read() and defines the function
//  (lambda) associated to the async_read() (read callback).
///////////////////////////////////////////////////////////////////////////////
void Connection::read(){
    auto self(shared_from_this());
    sock.async_read_some(buffer(request,MAX_REC_LENGTH),
        [this,self](boost::system::error_code err,size_t bt){
            if(err){
                std::cout << "Error: " << err.message() << std::endl;
                std::cout << "Close connection to " << id << '\n';
                sock.close();
            }else{
                request[bt] = '\0';
                std::string rqt_str(request);
                std::cout << rqt_str;
                if(Data.request_is_valid(rqt_str)){
                    if(flag_stream){
                        if(rqt_str==previous_request){
                            flag_stream = false;
                            send_message("ack\n"); }
                    }else{
                        if(rqt_str[0]=='s'){
                            flag_stream = true;
                            previous_request = rqt_str; }
                        handle_request(rqt_str);
                    }
                }else send_message("Invalid request\n");
                read();
            }
        });
}

///////////////////////////////////////////////////////////////////////////////
//  Connection::handle_request() - This function receives the request from
//  the client, in the form of a string, and acts accordingly. Namely,
//  computes the response and passes it to an async_write(). If a stream is
//  enabled, the "callback" associated to async_write trigger another write
///////////////////////////////////////////////////////////////////////////////
void Connection::handle_request(std::string req){  // TODO - Giant code!
    if(!sock.is_open()) return;

    auto self(shared_from_this());
    std::string response = Data.request_response(req);
    if(response == "Error\n") flag_stream = false;
    sock.async_write_some(buffer(std::move(response)),
        [this,self,req](boost::system::error_code err,size_t bt){
            if(err){
                std::cout << "Error: " << err.message() << std::endl;
                std::cout << "Close connection to " << id << '\n';
                sock.close();   }
            else{
                if(flag_stream){
                    handle_request(req);
                }
                std::cout << "Sent response to client " << id << std::endl;
            }
        });
}

///////////////////////////////////////////////////////////////////////////////
//  Connection::send_message() - Receives a string to be sent to the client.
///////////////////////////////////////////////////////////////////////////////
void Connection::send_message(string msg){
    auto self(shared_from_this());
    sock.async_write_some(buffer(msg),
        [this,self](boost::system::error_code err,size_t bt){
            if(err){
                std::cout << "Error: " << err.message() << std::endl;
                std::cout << "Close connection to " << id << '\n';
                sock.close();   }
            else{ std::cout << "Sent response to client " << id << std::endl; }
        });
}
