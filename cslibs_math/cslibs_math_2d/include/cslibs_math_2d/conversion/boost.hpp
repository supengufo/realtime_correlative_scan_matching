#ifndef CSLIBS_MATH_2D_BOOST_HPP
#define CSLIBS_MATH_2D_BOOST_HPP

#include <cslibs_math_2d/linear/line.hpp>
#include <cslibs_math_2d/linear/point.hpp>

#include <cslibs_boost_geometry/types.hpp>

namespace cslibs_math_2d {
namespace conversion {
template <typename T>
using boost_point_t = boost::geometry::model::d2::point_xy<T>;
template <typename T>
using boost_line_t = typename cslibs_boost_geometry::types::Line<boost_point_t<T>>::type;
template <typename T>
using boost_pointset_t = typename cslibs_boost_geometry::types::PointSet<boost_point_t<T>>::type;
template <typename T>
using boost_lineset_t = typename cslibs_boost_geometry::types::LineSet<boost_point_t<T>>::type;

template <typename T>
inline Point2<T> from(const boost_point_t<T> &p)
{
    return Point2<T>(p.x(), p.y());
}

template <typename T>
inline Line2<T> from(const boost_line_t<T> &l)
{
    return {{from(l.first), from(l.second)}};
}

template <typename T>
inline boost_point_t from(const Point2<T> &p)
{
    return boost_point_t(p.x(), p.y());
}

template <typename T>
inline boost_line_t from(const Line2<T> &l)
{
    return {from(l[0]), from(l[1])};
}

template <typename T>
inline void from(const boost_pointset_t<T> &src,
                 std::vector<Point2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const boost_point_t<T> &p){return from(p);});
}

template <typename T>
inline void from(const boost_lineset_t<T> &src,
                 std::vector<Line2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const boost_line_t<T> &l){return from(l);});
}

template <typename T>
inline void from(const std::vector<Point2<T>> &src,
                 boost_pointset_t<T> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const Point2<T> &p){return from(p);});
}

template <typename T>
inline void from(const std::vector<Line2<T>> &src,
                 boost_lineset_t<T> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const Line2<T> &l){return from(l);});
}
}
}

#endif // CSLIBS_MATH_2D_BOOST_HPP
