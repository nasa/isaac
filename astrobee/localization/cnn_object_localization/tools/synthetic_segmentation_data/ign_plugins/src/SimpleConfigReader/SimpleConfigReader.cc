/* Copyright (c) 2017, United States Government, as represented by the
* Administrator of the National Aeronautics and Space Administration.
* 
* All rights reserved.
* 
* The Astrobee platform is licensed under the Apache License, Version 2.0
* (the "License"); you may not use this file except in compliance with the
* License. You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governiing permissions and limitations
* under the License.
*/

#include "SimpleConfigReader.hh"
#include <sstream>

SimpleConfigReader::SimpleConfigReader(const std::string& filename) {
    load(filename);
}

bool SimpleConfigReader::load(const std::string& filename) {
    std::ifstream configFile(filename);
    if (!configFile.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(configFile, line)) {
        parseLine(line);
    }

    configFile.close();
    return true;
}

void SimpleConfigReader::parseLine(const std::string& line) {
    std::istringstream is_line(line);
    std::string key;
    if (std::getline(is_line, key, '=')) {
        std::string value;
        if (std::getline(is_line, value)) {
            configData[key] = value;
        }
    }
}

std::string SimpleConfigReader::getValue(const std::string& key) const {
    auto it = configData.find(key);
    if (it != configData.end()) {
        return it->second;
    } else {
        return "";
    }
}
