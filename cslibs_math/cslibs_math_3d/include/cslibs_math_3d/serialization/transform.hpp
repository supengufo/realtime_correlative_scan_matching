#ifndef CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP
#define CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP

#include <cslibs_math/serialization/vector.hpp>
#include <cslibs_math_3d/serialization/quaternion.hpp>
#include <cslibs_math_3d/linear/quaternion.hpp>
#include <cslibs_math_3d/linear/transform.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<typename T>
struct convert<cslibs_math_3d::Transform3<T>>
{
    static Node encode(const cslibs_math_3d::Transform3<T> &rhs)
    {
        Node n;
        n["t"] = rhs.translation();
        n["r"] = rhs.rotation();
        return n;
    }
    static bool decode(const Node& n, cslibs_math_3d::Transform3<T> &rhs)
    {
        if(!n.IsMap())
            return false;
        rhs.translation() = n["t"].as<cslibs_math_3d::Vector3<T>>();
        rhs.rotation()    = n["r"].as<cslibs_math_3d::Quaternion<T>>();
        return true;
    }
};
}

#endif // CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP
