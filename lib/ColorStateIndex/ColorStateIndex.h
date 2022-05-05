#pragma once

struct ColorStateIndex
{
    unsigned int Solid;
    unsigned int Animated;

    void SetValue(unsigned int index, unsigned int value)
    {
        switch (index)
        {
        case 0:
            Solid = value;
            break;
        case 1:
            Animated = value;
            break;
        default:
            break;
        }
    }

    int GetValue(unsigned int index)
    {
        switch (index)
        {
        case 0:
            return Solid;
        case 1:
            return Animated;
        default:
            return 0;
        }
    }

    void Reset(unsigned int index)
    {
        SetValue(index, 0);
    }

    void Increment(unsigned int index)
    {
        unsigned int value = GetValue(index);
        SetValue(index, value + 1);
    }
};