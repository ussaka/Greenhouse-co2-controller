#ifndef TEXT_PROPERTY_H
#define TEXT_PROPERTY_H

#include "Menu.h"

#include <string>

class TextProperty: public Property {
public:
	TextProperty(const std::string &name, const std::string& initialValue, bool constant = false);

	std::string getValue() override;
	std::string getRange() override;

	std::string getRealValue();

	void stopEdit(bool discard) override;
	bool exitOnConfirm() override;
	void input(bool up) override;

private:
	std::string value;
	std::string oldValue;

    const char* ascii = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    unsigned selected = 1;

    unsigned currentlyEditing = 0;
};


#endif