#include "pluginlib_use/polygons.h"
#include "pluginlib/class_list_macros.h"

namespace polygons_ns
{
    void Triangle::set_side_length(double edge)
    {
        side_length = edge;
    }

    double Triangle::get_perimeter()
    {
        return 3 * side_length;
    }

    void Square::set_side_length(double edge)
    {
        side_length = edge;
    }

    double Square::get_perimeter()
    {
        return 4 * side_length;
    }
} // namespace polygons_ns


PLUGINLIB_EXPORT_CLASS(polygons_ns::Triangle, polygon_ns::PolygonBase)
PLUGINLIB_EXPORT_CLASS(polygons_ns::Square, polygon_ns::PolygonBase)