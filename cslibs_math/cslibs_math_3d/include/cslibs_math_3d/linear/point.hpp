#ifndef CSLIBS_MATH_3D_POINT_3D_HPP
#define CSLIBS_MATH_3D_POINT_3D_HPP

#include <cslibs_math/color/color.hpp>
#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
template <typename T>
using Point3 = Vector3<T>;
using Point3d = Point3<double>;
using Point3f = Point3<float>;

template <typename T>
class EIGEN_ALIGN16 PointRGB3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PointRGB3<T>>;

    using point_t = Point3<T>;
    using color_t = cslibs_math::color::Color<T>;
    using type_t  = T;

    inline PointRGB3():
        a_(1.0)
    {
    }

    inline PointRGB3(const PointRGB3 &other) :
        point_(other.point_),
        a_(other.a_),
        color_(other.color_)
    {
    }

    inline PointRGB3(PointRGB3 &&other) :
        point_(std::move(other.point_)),
        a_(std::move(other.a_)),
        color_(std::move(other.color_))
    {
    }

    inline PointRGB3& operator=(const PointRGB3 &other)
    {
        point_ = other.point_;
        a_ = other.a_;
        color_ = other.color_;
        return *this;
    }

    inline PointRGB3(const point_t &pos) :
        point_(pos),
        a_(1.0)
    {
    }

    inline PointRGB3(const double& v) :
        PointRGB3(point_t(v,v,v))
    {
    }

    inline PointRGB3(const point_t &pos, const T& a, const color_t& c) :
        point_(pos),
        a_(a),
        color_(c)
    {
    }

    virtual ~PointRGB3() = default;

    inline point_t getPoint() const
    {
        return point_;
    }

    inline float getAlpha() const
    {
        return a_;
    }

    inline color_t getColor() const
    {
        return color_;
    }

    inline void setPoint(const point_t& point)
    {
        point_ = point;
    }

    inline void setAlpha(T a)
    {
        a_ = a;
    }

    inline void getColor(const color_t& c)
    {
        color_ = c;
    }

    inline PointRGB3& min(const PointRGB3 &other)
    {
        setPoint(point_.min(other.point_));
        return *this;
    }

    inline PointRGB3& max(const PointRGB3 &other)
    {
        setPoint(point_.max(other.point_));
        return *this;
    }

    inline static PointRGB3 inf()
    {
        return PointRGB3(point_t::inf());
    }

    inline static PointRGB3 max()
    {
        return PointRGB3(point_t::max());
    }

    inline static PointRGB3 min()
    {
        return PointRGB3(point_t::min());
    }

private:
    point_t point_;
    T       a_;
    color_t color_;
};

using PointRGB3d = PointRGB3<double>;
using PointRGB3f = PointRGB3<float>;
}

#endif // CSLIBS_MATH_2D_POINT_3D_HPP
