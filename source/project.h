#pragma once
#include <string>
#include <cstring>
#include <Arduino.h>
#include "nvs.h"
#include <WebSocketsServer.h>


using namespace std;

class Parameter {
public:
    uint32_t default_value;
    string readable_name;
    void set_value(string s, uint32_t d) {
        readable_name = s;
        default_value = d;
    };
    Parameter(string s, uint32_t d){
        readable_name = s;
        default_value = d;
    }
    Parameter(){
        readable_name = "";
        default_value = 0;
    }
};

// ------------------------------------------- Ring class -------------------------------------------------
template <typename T>
class Ring {
    public:
        T average(int n);
//        int fullness();
        int fullness() {
            return (buffer_size + in_index - out_index) % buffer_size;
        }
        void reset() {
            in_index = 0;
            out_index = 0;
            memset(buffer, 0, sizeof(T) * buffer_size);
        }
        void add(T item);
        Ring(int size);
        int get_last_n(int num, T * items);
    private:
        int in_index;
        int out_index;
        T * buffer;
        int buffer_size;
};

template <typename T>
Ring<T>::Ring(int size) {  // create a ring buffer of size size
    buffer_size = size;
    in_index = 0;
    out_index = 0;
    buffer = new T[size];
    memset(buffer, 0, sizeof(T) * size);
}

template <typename T>
void Ring<T>::add(T item) {
    buffer[in_index] = item;
    in_index = (in_index + 1) % buffer_size;
    if (in_index == out_index)
        out_index = (out_index + 1) % buffer_size;
    // we can fill this queue, even though we only look at most recent ones for average
}

template <typename T>
T Ring<T>::average(int m) { // return the average of the last m values entered
    // N = running_average_depth
    T sum;
    int index, n;
    int f = fullness();
    // start with last entry
    sum = 0;
    n = (f >= m) ? m : f;
    for (int i = 0; i < n; i++) {
        index = (buffer_size + in_index - i - 1) % buffer_size;
        sum += buffer[index];
    }
    if (n == 0) return 0;
    return sum / n;
}

template <typename T>
int Ring<T>::get_last_n(int num, T * items){
    int i;
    if (num >= buffer_size) return 0;
    int f = fullness();
    int iptr = (in_index + buffer_size - 1) % buffer_size;  // address of last one entered
    for (i = 0; i < num; i++) {
        if (i >= f) break;
        items[i] = buffer[iptr];
        if (iptr == 0) 
            iptr = buffer_size - 1;
        else
            iptr--;
    }
    return i + 1;
}


void idle(int idle_ms);
void initialize_project();
bool save_params();
void restore_parameters(Parameter * p, int count);
void maybe_save_params();
string int_to_string(int i);
int str_to_int(string s);
void printHexArray(uint8_t *arr, uint8_t len);
void initalize_arduino();
void new_telnet_client();
void send_parameters(uint8_t num);

#define PARAM_COUNT 30  /* This is the maximum number of parameters */
typedef  uint32_t Params[PARAM_COUNT];



