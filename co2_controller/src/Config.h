#ifndef CONFIG_H
#define CONFIG_H

#include <unordered_map>
#include <string>

class Config
{
public:
    Config();

    bool exists(const std::string& key);
    const std::string& get(const std::string& key);
    void set(const std::string& key, const std::string& value);

private:
    std::unordered_map <std::string, std::string> data;
    
    //  Manual says that top 64 bytes of the EEPROM are reserved and may not be used
    const static unsigned offset = 65;

    //  How many bytes of the EEPROM are we going to use
    const static unsigned range = 256;  
};

#endif