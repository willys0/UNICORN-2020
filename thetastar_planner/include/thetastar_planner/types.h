#pragma once

// Custom types to replace OpenCV

namespace Types {
    struct Point2f;

    struct Point {
        int x;
        int y;

        void operator=(const Types::Point& lhc);
        void operator=(const Types::Point2f& lhc);

        Point() { x = 0; y = 0; };
        Point(int _x, int _y) { x = _x; y = _y; }
    };

    struct Point2f {
        float x;
        float y;

        Point2f() { x = 0; y = 0; };
        Point2f(float _x, float _y) { x = _x; y = _y; }
        Point2f(const Types::Point& p);

        void operator=(const Types::Point& lhc);
        Types::Point2f operator+(const Types::Point2f& lhc);
        Types::Point2f operator-(const Types::Point2f& lhc);
        Types::Point2f operator-();
        Types::Point2f operator+=(const Types::Point2f& rhs);
        friend Types::Point2f operator * (float f, const Types::Point2f& c);
        friend Types::Point2f operator * (const Types::Point2f& c, float f);
    };

    struct Point2d {
		double x;
		double y;

        Point2d() { x = 0; y = 0; };
		Point2d(double _x, double _y) { x = _x, y = _y; };
    };
}
