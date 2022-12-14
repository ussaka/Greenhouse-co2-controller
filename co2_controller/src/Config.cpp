#include "Config.h"
#include "Disablers.h"

#include "eeprom.h"
#include "board.h"
#include "FreeRTOS.h"

void Config::read()
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);
    Chip_SYSCTL_PeriphReset(RESET_EEPROM);

    //  Let's read the EEPROM first so it can be cached
    uint8_t buffer[range] { 0 };
    uint8_t res = Chip_EEPROM_Read(offset, buffer, range);

    //  Where in the buffer do the key and value start
    uint8_t* keyStart = nullptr;
	uint8_t* valueStart = nullptr;

    //  We don't where meaningful data ends so we have to loop through the entire buffer
	for(unsigned i = 0; i < range; i++)
	{
        //  Is the current byte "Start of heading"
		if(buffer[i] == 1)
		{
            //  Null the byte so that std::string knows where a value ends
			buffer[i] = 0;

            //  If the value has already been found, save the previous key value pair
			if(valueStart)
			{
                /*  Because we've nulled bytes that mark where keys and values start
                 *  we can just give std::string a pointer to the byte that comes after
                 *  said mark. It's guaranteed that there will be a null terminator at some
                 *  point which means that std::string knows where a key or a value ends */
				std::string key((char*)keyStart);
				data[key] = std::string((char*)valueStart);

                DEBUGSTR(std::string("Key is '" + key + "'\r\n").c_str());
                DEBUGSTR(std::string("Value is '" + data[key] + "'\r\n").c_str());

                //  The previous value start is irrelevant now
				valueStart = nullptr;
			}

            //  The next key starts on the next byte
			keyStart = &buffer[i] + 1;
		}

		else if(buffer[i] == 2)
		{
            //  Null the byte so that std::string knows where a key ends
			buffer[i] = 0;

            //  The value starts on the next byte
			valueStart = &buffer[i] + 1;
		}
	}
}

bool Config::exists(const std::string& key)
{
    //  Is the given key found in data
    auto it = data.find(key);
    return it != data.end();
}

const std::string& Config::get(const std::string& key)
{
    static std::string empty;
    auto it = data.find(key);

    //  If the given key isn't present in data, return an empty string
    if(it == data.end())
        return empty;

    //  Return the found value
    return it->second;
}

void Config::set(const std::string& key, const std::string& value)
{
    data[key] = value;
    uint8_t buffer[range] { 0 };
    size_t index = 0;

    //  Loop through each key value pair
    for(auto& it : data)
    {
        //  Set a mark that indicates that a key starts here
        buffer[index] = 1;

        //  Write the key to the buffer
        for(size_t i = 0; i < it.first.size(); i++)
            buffer[++index] = it.first[i];

        //  Set a mark that indicates that a value starts here
        buffer[++index] = 2;

        //  Write the value to the buffer
        for(size_t i = 0; i < it.second.size(); i++)
            buffer[++index] = it.second[i];

        index++;
    }

    //  Because of the way the config parser works, there needs to be a key marker at the very end
    buffer[index] = 1;

    //  Disable the scheduler and write the buffer to the EEPROM
    DisableScheduler d;
    uint8_t res = Chip_EEPROM_Write(offset, buffer, range);

    DEBUGSTR(std::string("eeprom_write returned " + std::to_string(res) + "\r\n").c_str());
}