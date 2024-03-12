#include <string>
#include <map>
#include <fstream>

class SimpleConfigReader {
public:
    SimpleConfigReader() = default;
    explicit SimpleConfigReader(const std::string& filename);
    bool load(const std::string& filename);
    std::string getValue(const std::string& key) const;

private:
    std::map<std::string, std::string> configData;
    void parseLine(const std::string& line);
};
