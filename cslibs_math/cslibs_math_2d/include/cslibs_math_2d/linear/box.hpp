#ifndef CSLIBS_MATH_2D_BOX_2D_HPP
#define CSLIBS_MATH_2D_BOX_2D_HPP

#include <cslibs_math_2d/linear/line.hpp>
#include <cslibs_math/common/equal.hpp>

#include <limits>
#include <set>

namespace cslibs_math_2d {
template <typename T>
class EIGEN_ALIGN16 Box2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t     = Eigen::aligned_allocator<Box2<T>>;

    using point_t        = Point2<T>;
    using line_t         = Line2<T>;
    using point_set_t    = std::set<point_t, typename point_t::allocator_t>;
    using coefficients_t = std::array<T, 2>;

    inline Box2() :
        min_(std::numeric_limits<T>::lowest(),
             std::numeric_limits<T>::lowest()),
        max_(std::numeric_limits<T>::max(),
             std::numeric_limits<T>::max())
    {
    }

    inline Box2(const T min_x, const T min_y,
                 const T max_x, const T max_y) :
        min_(min_x, min_y),
        max_(max_x, max_y)
    {
    }

    inline Box2(const point_t &min,
                 const point_t &max) :
        min_(min),
        max_(max)
    {
    }

    inline Box2(const Box2 &other) :
        min_(other.min_),
        max_(other.max_)
    {
    }

    inline Box2(Box2 &&other) :
        min_(std::move(other.min_)),
        max_(std::move(other.max_))
    {
    }

    inline void setMin(const point_t &min)
    {
        min_ = min;
    }

    inline void setMax(const point_t &max)
    {
        max_ = max;
    }

    inline point_t const & getMin() const
    {
        return min_;
    }

    inline point_t const & getMax() const
    {
        return max_;
    }

    inline point_t lu() const
    {
        return point_t(min_(0), max_(1));
    }

    inline point_t ll() const
    {
        return min_;
    }

    inline point_t ru() const
    {
        return max_;
    }

    inline point_t rl() const
    {
        return point_t(max_(0), min_(1));
    }

    inline bool intersects(const line_t &line) const
    {
        //// LIANG BARSKY
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        T t0 = T();
        T t1 = 1.0;

        auto clip = [] (const T p, const T q,
                        T &t0, T &t1)
        {
            if(cslibs_math::common::eq(p, T()) && q < T())
                return false;

            const T r = q / p;
            if(p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if(p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d(0), -(min_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(d(0), (max_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(-d(1), -(min_(1)-p0(1)), t0, t1))
                return false;

        if(!clip(d(1), (max_(1)-p0(1)), t0, t1))
                return false;
        return true;
    }

    inline bool intersection(const line_t &line,
                             line_t &clipped)
    {
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        T t0 = T();
        T t1 = 1.0;

        auto clip = [] (const T p, const T q,
                        T &t0, T &t1)
        {
            if(cslibs_math::common::eq(p, T()) && q < T())
                return false;

            const T r = q / p;
            if (p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if (p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d(0), -(min_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(d(0), (max_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(-d(1), -(min_(1)-p0(1)), t0, t1))
                return false;

        if(!clip(d(1), (max_(1)-p0(1)), t0, t1))
                return false;

        clipped[0](0) = p0(0) + t0*d(0);
        clipped[0](1) = p0(1) + t0*d(1);
        clipped[1](0) = p0(0) + t1*d(0);
        clipped[1](1) = p0(1) + t1*d(1);

        return true;
    }

private:
    point_t min_;
    point_t max_;
};

using Box2d = Box2<double>;
using Box2f = Box2<float>;
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Box2<T> &b)
{
    out << "[" << b.getMin() << "," << b.getMax() << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_BOX_2D_HPP
