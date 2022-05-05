#pragma once

struct Color
{
    unsigned char r;
    unsigned char g;
    unsigned char b;

    static Color CreateColor(int r, int g, int b)
    {
        return {(unsigned char)r, (unsigned char)g, (unsigned char)b};
    }
};