#ifndef CSV_READER_H
#define CSV_READER_H

#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdexcept>

using std::string;
using std::vector;
using std::ifstream;
using std::getline;
using std::stringstream;

class CSVReader {
public:
    CSVReader() {}

    vector<vector<string>> readCSV(string filepath, vector<string> columnLabels);

private:
    vector<string> split(const string& str, char delim);
    int findIndex(const vector<string>& vec, const string& item);
};

#endif
