#include "server.hpp"

using namespace std;
using namespace std::chrono;

///////////////////////////////////////////////////////////////////////////////
//  is_available() - predicate to return flag
///////////////////////////////////////////////////////////////////////////////
bool Luminaires::is_available(int id){
    return avail[id];
}

///////////////////////////////////////////////////////////////////////////////
//  is_valid() - Checks if the data stucture is valid
///////////////////////////////////////////////////////////////////////////////
bool Luminaires::is_valid(){
    return valid;
}

///////////////////////////////////////////////////////////////////////////////
//  reset() - Resets the luminaires data
///////////////////////////////////////////////////////////////////////////////
void Luminaires::reset(i2c_msg msg1,i2c_msg msg2){
    char id_1 = (msg1.bytes[0]&0b00000001) + '0';
    char id_2 = (msg2.bytes[0]&0b00000001) + '0';
    string id1;
    string id2;
    id1.push_back(id_1);
    id2.push_back(id_2);

    start = msg1.t;
    last_min.clear();
    data.clear();

    data["N 0"] = 0;
    data["N 1"] = 0;
    data["Lh "+id1] = msg1.bytes[1];
    data["Lb "+id1] = msg1.bytes[2];
    data["o "+id1] = msg1.bytes[3];
    data["Lh "+id2] = msg2.bytes[1];
    data["Lb "+id2] = msg2.bytes[2];
    data["o "+id2] = msg2.bytes[3];

    valid = true;
}

///////////////////////////////////////////////////////////////////////////////
//  update_value() - Receives a new message and updates the internal data
///////////////////////////////////////////////////////////////////////////////
void Luminaires::update_values(i2c_msg m){
    char id_ = (m.bytes[0] & 0b00000001);
    bool s_ = m.bytes[0] & 0b00000010;
    char d_ = m.bytes[1];
    char l_ = m.bytes[2];
    char r_ = m.bytes[3];

    string id;
    id.push_back(id_+'0');
    std::chrono::duration<double,std::ratio<1,1>> elapsed = m.t - start;

    timed_values l,d;
    l.value = l_;
    l.time = m.t;
    d.value = d_*100.0/MAX_D;
    d.time = m.t;

    // Lock thread here...
    rw[id_].lock();

    last_min["l "+id].push_back(l);
    remove_older_than_1min(&last_min["l "+id]);
    last_min["d "+id].push_back(d);
    remove_older_than_1min(&last_min["d "+id]);

    data["l "+id] = l.value;
    if(s_) data["L "+id] = data["Lh "+id];
    else data["L "+id] = data["Lb "+id];

    data["N "+id]+=1;
    data["caux "+id] += max((float) 0, data["L "+id]-data["l "+id]);

    if(data["N "+id] > 3.0){
        if(((data["l "+id] - data["l-1 "+id])*(data["l-1 "+id] - data["l-2 "+id])) < 0)
            data["vaux "+id] += (abs(data["l "+id] - data["l-1 "+id]) + abs(data["l-1 "+id] - data["l-2 "+id]))/(2*TS);
        data["l-2 "+id] = data["l-1 "+id];
        data["l-1 "+id] = data["l "+id];
    }

    data["e "+id] += data["d "+id]*0.01*0.01; /*data["d 0"] last iteration*/
    data["e T"] = data["e 0"] + data["e 1"];

    data["c "+id] = data["caux "+id]/data["N "+id];
    data["c T"] = (data["caux 0"] + data["caux 1"])/(data["N 0"] + data["N 1"]);

    data["v "+id] = data["vaux "+id]/data["N "+id];
    data["v T"] = (data["vaux 0"] + data["vaux 1"])/(data["N 0"] + data["N 1"]);

    data["d "+id] = d.value;
    data["r "+id] = r_;
    data["s "+id] = s_;

    data["p "+id] = data["d "+id]*0.01; /*data["d 0"] after update*/
    data["p T"] = data["p 0"] + data["p 1"];

    data["t "+id] = elapsed.count();

    rw[id_].unlock();

    // Signal threads that a new value is available
    unique_lock<std::mutex> lck(mut[id_]);
    avail[id_] = true;
    cv.notify_all();


    return;
}

///////////////////////////////////////////////////////////////////////////////
//  request_response() - Transform a request string into a response string
///////////////////////////////////////////////////////////////////////////////
string Luminaires::request_response(string request){
    string index = request.substr(2,3);
    string response("Error");
    request.pop_back();
    int id = request[4] - '0';

    if(request[0]=='g'){
        rw[id].lock();
        if(data.find(index)!=data.end())
            response = index + " " + to_string(data[index]);
        else
            response = index + " is empty";
        rw[id].unlock();    }

    if(request[0]=='b'){
        rw[id].lock();
        if(last_min.find(index)!=last_min.end()){
            response = request + " ";
            remove_older_than_1min(&last_min[index]);
            if(last_min[index].empty())
                response = request + " is empty ";
            response.reserve(BUFF_EST_LENGTH);
            for(auto it = last_min[index].cbegin();it!=last_min[index].cend();++it)
                response = response + to_string(it->value) + ",";
            response.pop_back();    }
        else
            response = request + " is empty";
        rw[id].unlock();
    }

    if(request[0]=='s')
        if(data.find(index)!=data.end()){
            unique_lock<std::mutex> lck(mut[id]);
            while(!avail[id])
                if(cv.wait_for(lck,seconds(1))==cv_status::timeout)
                    return std::move("Error\n");
            avail[id] = false;

            string val = to_string(data[index]);
            duration<double,std::milli> elapsed = steady_clock::now() - start;
            response = request + " " + val + "\t" + to_string(elapsed.count());
        }

    return std::move(response + "\n");
}

///////////////////////////////////////////////////////////////////////////////
//  request_is_valid - Checks whether a given request is valid
///////////////////////////////////////////////////////////////////////////////
bool Luminaires::request_is_valid(string request){
    if(request.length() == REC_LENGTH){
        const string starters("gbs");
        const string specifiers("ldsLorptecv");
        const string enders("01T");
        if(starters.find(request[0])!=string::npos)
            if(specifiers.find(request[2])!=string::npos)
                if(enders.find(request[4])!=string::npos)
                    if(request[1]==' ' and request[3] == ' ')
                        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////
//  remove_older_than_1min - removes older elements from list
///////////////////////////////////////////////////////////////////////////////
void Luminaires::remove_older_than_1min(std::deque<timed_values> *l){
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double,std::ratio<1,1>> elapsed = now - l->front().time;
    while(elapsed.count()>BUFF_TIME_SEC and !l->empty()){
        l->pop_front();
        elapsed = now - l->front().time;
    }
}
