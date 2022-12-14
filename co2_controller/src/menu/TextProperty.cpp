#include "TextProperty.h"

TextProperty::TextProperty(const std::string &name, const std::string& initialValue, bool constant)
    : Property(name, constant), value(initialValue), oldValue(initialValue)
{
}

std::string TextProperty::getValue()
{
    return value;
}

std::string TextProperty::getRange()
{
    return "ASCII";
}

std::string TextProperty::getRealValue()
{
    return oldValue;
}

void TextProperty::stopEdit(bool discard)
{
    if (discard)
        value = oldValue;

    else oldValue = value;
}

void TextProperty::input(bool up)
{
    const unsigned asciiLength = 95;
    if(currentlyEditing >= value.size())
        value += ascii[selected];

    if(up)
    {
        if(++selected >= asciiLength)
            selected = 0;
    }

    else
    {
        if(--selected >= asciiLength)
            selected = asciiLength - 1;
    }

    value.back() = ascii[selected];
}

bool TextProperty::exitOnConfirm()
{
    //  TODO Use something else than '!'
    if(ascii[selected] == '!')
    {
        //  If the user didn't reach the last character, cut the string
        if(currentlyEditing < value.size() - 1)
            value.erase(value.begin() + currentlyEditing, value.end());

        currentlyEditing = 0;
        return true;
    }

    //  If we're not exiting, move on to the next character
    currentlyEditing++;
    return false;
}