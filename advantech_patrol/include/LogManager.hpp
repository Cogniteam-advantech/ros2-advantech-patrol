/*
 * LogManager.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 *  Converted to ROS2 Humble
 */

#ifndef INCLUDE_LOG_MANAGER_HPP
#define INCLUDE_LOG_MANAGER_HPP

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ctime>
#include <memory>

using namespace std;

class LogManager {

public:
    LogManager(string logPath = "");
    virtual ~LogManager();

    void setLogPath(const string& logPath);
    void writeToLog(const string& line);
    void closeFile();

private:
    string getCurrentTime();

private:
    string log_path_ = "";
    ofstream log_file_;
};

#endif /* INCLUDE_LOG_MANAGER_HPP */