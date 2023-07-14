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

#include "CSVReader.hh"

vector<string> CSVReader::split(const string& str, char delim) {
    vector<string> tokens;
    stringstream ss(str);
    string token;
    while (getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

int CSVReader::findIndex(const vector<string>& vec, const string& item) {
    auto it = find(vec.begin(), vec.end(), item);
    if (it != vec.end())
        return distance(vec.begin(), it);
    else
        throw std::runtime_error("Column not found");
}

vector<vector<string>> CSVReader::readCSV(string filepath, vector<string> columnLabels) {
    ifstream file(filepath);
    if (!file.is_open())
        throw std::runtime_error("Could not open file");

    string line;
    getline(file, line);
    vector<string> header = split(line, ',');
    vector<int> indices;
    for (auto& label : columnLabels) {
        indices.push_back(findIndex(header, label));
    }

    vector<vector<string>> data;
    while (getline(file, line)) {
        vector<string> row = split(line, ',');
        vector<string> selectedData;
        for (auto index : indices) {
            selectedData.push_back(row[index]);
        }
        data.push_back(selectedData);
    }

    file.close();
    return data;
}
