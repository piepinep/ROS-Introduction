#ifndef POLYGONS_H
#define POLYGONS_H

#include "pluginlib_use/polygon_base.h"
#include "pluginlib/class_list_macros.h"

namespace polygons_ns{

    class Triangle : public polygon_ns::PolygonBase{
    public:
        Triangle():side_length(0){}
        void set_side_length(double edge);
        double get_perimeter();

    private :
        double side_length;
    };

    class Square : public polygon_ns::PolygonBase
    {
    public:
        Square() : side_length(0) {}
        void set_side_length(double edge);
        double get_perimeter();

    private:
        double side_length;
    };
};


#endif