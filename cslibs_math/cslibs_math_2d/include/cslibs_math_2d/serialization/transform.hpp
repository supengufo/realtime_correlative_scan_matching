#ifndef CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP
#define CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP

#include <cslibs_math/serialization/vector.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<typename T>
struct convert<cslibs_math_2d::Transform2<T>>
{
    static Node encode(const cslibs_math_2d::Transform2<T> &rhs)
    {
        Node n;
        n["t"] = rhs.translation();
        n["r"] = rhs.yaw();
        return n;
    }
    static bool decode(const Node& n, cslibs_math_2d::Transform2<T> &rhs)
    {
        if(!n.IsMap())
            return false;
        rhs.translation() = n["t"].as<cslibs_math_2d::Vector2<T>>();
        rhs.setYaw(n["r"].as<T>());
        return true;
    }
};
}

#endif // CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP
