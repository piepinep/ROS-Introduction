#ifndef POLYGON_H
#define POLYGON_H

namespace polygon_ns{

    class PolygonBase
    {
    protected:
        PolygonBase(){}

    public:
        virtual void set_side_length(double length) = 0;
        virtual double get_perimeter() = 0;
    };
};

#endif