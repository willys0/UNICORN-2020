#include <thetastar_planner/types.h>

Types::Point2f::Point2f(const Types::Point& p)
{
    this->x = (float)p.x;
    this->y = (float)p.y;
}

///////////////////
// Point2f
///////////////////
void Types::Point2f::operator=(const Types::Point& rhs)
{
    this->x = (float)rhs.x;
    this->y = (float)rhs.y;
}

Types::Point2f Types::Point2f::operator+(const Types::Point2f& rhs)
{
    return Types::Point2f(this->x + rhs.x, this->y + rhs.y);
}

Types::Point2f Types::Point2f::operator-(const Types::Point2f& rhs)
{
    return Types::Point2f(this->x - rhs.x, this->y - rhs.y);
}

Types::Point2f Types::Point2f::operator-()
{
    return Types::Point2f(-this->x, -this->y);
}

Types::Point2f Types::Point2f::operator+=(const Types::Point2f& rhs) {
    this->x += rhs.x;
    this->y += rhs.y;

    return *this;
}

Types::Point2f operator-(const Types::Point2f& rhs) {
    return Types::Point2f(-rhs.x, -rhs.y);
}

Types::Point2f Types::operator*(float f, const Types::Point2f& c)
{
    return Types::Point2f(f * c.x, f * c.y);
}

Types::Point2f Types::operator*(const Types::Point2f& c, float f)
{
    return Types::Point2f(f * c.x, f * c.y);
}

// Point
void Types::Point::operator=(const Types::Point& rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
}

void Types::Point::operator=(const Types::Point2f& rhs)
{
    this->x = (int)rhs.x;
    this->y = (int)rhs.y;
}
