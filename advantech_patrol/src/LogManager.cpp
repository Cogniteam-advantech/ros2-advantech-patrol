/*
 * LogManager.cpp
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 *  Converted to ROS2 Humble
 */

#include "LogManager.hpp"

LogManager::LogManager(string logPath) {
    log_path_ = logPath;
}

LogManager::~LogManager() {
    closeFile();
}

void LogManager::setLogPath(const string& logPath) {
    log_path_ = logPath;
    
    string currTime = getCurrentTime();
    string full_path = log_path_ + "/" + currTime + ".txt";
    
    std::cout << "Log path is: " << full_path << std::endl;
    
    // Close existing file if open
    if (log_file_.is_open()) {
        log_file_.close();
    }
    
    log_file_.open(full_path);
    
    if (!log_file_.is_open()) {
        std::cerr << "Failed to open log file: " << full_path << std::endl;
    }
}

void LogManager::writeToLog(const string& line) {
    if (!log_file_.is_open()) {
        std::cerr << "Log file is not open, cannot write: " << line << std::endl;
        return;
    }
    
    string currTime = getCurrentTime();
    log_file_ << currTime << ": " << line << std::endl;
    log_file_.flush(); // Ensure immediate write to file
}

void LogManager::closeFile() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

string LogManager::getCurrentTime() {
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];
    
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    strftime(buffer, 80, "%F/%H_%M_%S", timeinfo);
    string curr_time_str = string(buffer);
    
    // Replace '/' and '-' with '_'
    std::replace(curr_time_str.begin(), curr_time_str.end(), '/', '_');
    std::replace(curr_time_str.begin(), curr_time_str.end(), '-', '_');
    
    return curr_time_str;
}