#include "coord.h"
#include "utils.h"

const COORD COORD::Null(-1, -1);
const COORD COORD::North(0, 1);
const COORD COORD::East(1, 0);
const COORD COORD::South(0, -1);
const COORD COORD::West(-1, 0);
const COORD COORD::NorthEast(1, 1);
const COORD COORD::SouthEast(1, -1);
const COORD COORD::SouthWest(-1, -1);
const COORD COORD::NorthWest(-1, 1);

const COORD COORD::Compass[8] = 
{ 
    North, 
    East, 
    South, 
    West, 
    NorthEast,
    SouthEast,
    SouthWest,
    NorthWest
};

const char* COORD::CompassString[8] = 
{ 
    "N", 
    "E",
    "S",
    "W",
    "NE",
    "SE",
    "SW",
    "NW"
};

void COORD::UnitTest()
{
    assert(COORD(3, 3) + COORD(2, 2) == COORD(5, 5));
    COORD coord(5, 2);
    coord += COORD(2, 5);
    assert(coord == COORD(7, 7));
    assert(COORD(2, 2) + North == COORD(2, 3));
    assert(COORD(2, 2) + East == COORD(3, 2));
    assert(COORD(2, 2) + South == COORD(2, 1));
    assert(COORD(2, 2) + West == COORD(1, 2));
    assert(Compass[E_NORTH] == North);
    assert(Compass[E_EAST] == East);
    assert(Compass[E_WEST] == West);
    assert(Compass[E_SOUTH] == South);
    assert(Clockwise(E_NORTH) == E_EAST);
    assert(Clockwise(E_EAST) == E_SOUTH);
    assert(Clockwise(E_SOUTH) == E_WEST);
    assert(Clockwise(E_WEST) == E_NORTH);
    assert(Opposite(E_NORTH) == E_SOUTH);
    assert(Opposite(E_EAST) == E_WEST);
    assert(Opposite(E_SOUTH) == E_NORTH);
    assert(Opposite(E_WEST) == E_EAST);
    assert(Anticlockwise(E_NORTH) == E_WEST);
    assert(Anticlockwise(E_EAST) == E_NORTH);
    assert(Anticlockwise(E_SOUTH) == E_EAST);
    assert(Anticlockwise(E_WEST) == E_SOUTH);
    assert(ManhattanDistance(COORD(3, 2), COORD(-4, -7)) == 16);
    assert(DirectionalDistance(COORD(3, 2), COORD(-4, -7), E_NORTH) == -9);
    assert(DirectionalDistance(COORD(3, 2), COORD(-4, -7), E_EAST) == -7);
    assert(DirectionalDistance(COORD(3, 2), COORD(-4, -7), E_SOUTH) == 9);
    assert(DirectionalDistance(COORD(3, 2), COORD(-4, -7), E_WEST) == 7);
}
