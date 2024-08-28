#ifndef _TIMER_
#define _TIMER_
#pragma once
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include "01.Slam2ref/RosParamServer.h"
#include "0.Utils/ParamServer.h"


class Timer {
public:
    Timer(const std::string& name) : name(name), start(std::chrono::high_resolution_clock::now())    {
        std::cout << BLUE_BOLD <<"     Starting: " << name << "..." << DEFAULT << std::endl;
        params = &ParamServer::getInstance();
    }


    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        auto duration_in_mins = duration_in_ms / 1000.0 / 60.0;   // Convert milliseconds to minutes.

        std::cout << WHITE <<"\n       TIMER:  " << name << " took " << duration_in_ms << " ms (" << duration_in_mins
        << " mins) \n" << DEFAULT << std::endl;

        std::ofstream log_file(params->save_directory_ + "time_log.txt", std::ios_base::app | std::ios_base::out);
        log_file << name << "; took ;" << duration_in_ms << "; ms; " << duration_in_mins << "; mins" << std::endl;  // For csv.
    }

private:
    ParamServer* params;

    std::string name;
    std::chrono::high_resolution_clock::time_point start;
    const std::string BLUE_BOLD = "\033[1;34m";
    const std::string ORANGE = "\033[38;5;202m";
    const std::string LIGHT_BLUE = "\033[1;96m";
    const std::string DEFAULT = "\033[0m";
    const std::string WHITE = "\033[1;97m";
};


#endif //_TIMER_