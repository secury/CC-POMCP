#ifndef COORD_H
#define COORD_H

#include <stdlib.h>
#include <assert.h>
#include <ostream>
#include <math.h>

struct COORD
{
    int X, Y;
    
    COORD() { }
    COORD(int x, int y)
    :   X(x), Y(y) { }

    bool Valid() const
    {
        return X >= 0 && Y >= 0;
    }

    bool operator==(COORD rhs) const
    {
        return X == rhs.X && Y == rhs.Y;
    }
    
    bool operator!=(COORD rhs) const
    {
        return X != rhs.X || Y != rhs.Y;
    }

    void operator+=(COORD offset)
    {
        X += offset.X;
        Y += offset.Y;
    }

    COORD operator+(COORD rhs) const
    {
        return COORD(X + rhs.X, Y + rhs.Y);
    }
    
    COORD operator*(int mul) const
    {
        return COORD(X * mul, Y * mul);
    }

    enum
    {
        E_NORTH,
        E_EAST,
        E_SOUTH,
        E_WEST,
        E_NORTHEAST,
        E_SOUTHEAST,
        E_SOUTHWEST,
        E_NORTHWEST
    };
    
    static double EuclideanDistance(COORD lhs, COORD rhs);
    static int ManhattanDistance(COORD lhs, COORD rhs);
    static int DirectionalDistance(COORD lhs, COORD rhs, int direction);
    
    static const COORD Null;
    static const COORD North, East, South, West;
    static const COORD NorthEast, SouthEast, SouthWest, NorthWest;
    static const COORD Compass[8];
    static const char* CompassString[8];
    static int Clockwise(int dir) { return (dir + 1) % 4; }
    static int Opposite(int dir) { return (dir + 2) % 4; }
    static int Anticlockwise(int dir) { return (dir + 3) % 4; }

    static void UnitTest();
};

inline double COORD::EuclideanDistance(COORD lhs, COORD rhs)
{
    return sqrt((lhs.X - rhs.X) * (lhs.X - rhs.X) 
              + (lhs.Y - rhs.Y) * (lhs.Y - rhs.Y));
}

inline int COORD::ManhattanDistance(COORD lhs, COORD rhs)
{
    return abs(lhs.X - rhs.X) + abs(lhs.Y - rhs.Y);
}

inline int COORD::DirectionalDistance(COORD lhs, COORD rhs, int direction)
{
    switch (direction)
    {
        case E_NORTH: return rhs.Y - lhs.Y;
        case E_EAST: return rhs.X - lhs.X;
        case E_SOUTH: return lhs.Y - rhs.Y;
        case E_WEST: return lhs.X - rhs.X;
        default: assert(false);
    }
}

inline std::ostream& operator<<(std::ostream& ostr, COORD& coord)
{
    ostr << "(" << coord.X << ", " << coord.Y << ")";
    return ostr;
}

#endif // COORD_H
