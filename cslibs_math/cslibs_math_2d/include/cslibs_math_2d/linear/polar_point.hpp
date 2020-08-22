#ifndef CSLIBS_MATH_2D_POLAR_HPP
#define CSLIBS_MATH_2D_POLAR_HPP

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
template <typename T>
class EIGEN_ALIGN16 PolarPoint2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PolarPoint2<T>>;

    using point_t = Point2<T>;

    inline PolarPoint2() :
        cartesian_(std::numeric_limits<T>::infinity()),
        theta_(0.0),
        rho_(std::numeric_limits<T>::infinity())
    {
    }

    inline PolarPoint2(const T theta,
                       const T rho) :
        cartesian_(std::cos(theta) * rho,
                   std::sin(theta) * rho),
        theta_(theta),
        rho_(rho)
    {
    }

    inline PolarPoint2(const point_t &cartesian) :
        cartesian_(cartesian),
        theta_(cslibs_math_2d::angle(cartesian)),
        rho_(cartesian.length())
    {
    }

    inline PolarPoint2(const point_t &cartesian,
                       const T theta,
                       const T rho) :
        cartesian_(cartesian),
        theta_(theta),
        rho_(rho)
    {
    }

    inline PolarPoint2(const PolarPoint2 &other) :
        cartesian_(other.cartesian_),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    inline PolarPoint2(PolarPoint2 &&other) :
        cartesian_(std::move(other.cartesian_)),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    virtual ~PolarPoint2() = default;

    inline bool isNormal() const
    {
        return std::isnormal(rho_) && !std::isinf(rho_);
    }

    inline void setRho(const T rho)
    {
        rho_ = rho;
        updateCartesian();
    }

    inline void setTheta(const T theta)
    {
        theta_ = theta;
        updateCartesian();
    }

    inline T getRho() const
    {
        return rho_;
    }

    inline T getTheta() const
    {
        return theta_;
    }

    inline void setCartesian(const point_t &cartesian)
    {
        cartesian_ = cartesian;
        updatePolar();
    }

    inline point_t const & getCartesian() const
    {
        return cartesian_;
    }

    inline PolarPoint2 min(const PolarPoint2 &other) const
    {
        return PolarPoint2(cslibs_math::linear::min(cartesian_, other.cartesian_),
                           std::min(theta_, other.theta_),
                           std::min(rho_, other.rho_));
    }

    inline PolarPoint2 max(const PolarPoint2 &other) const
    {
        return PolarPoint2(cslibs_math::linear::max(cartesian_, other.cartesian_),
                           std::max(theta_, other.theta_),
                           std::max(rho_, other.rho_));
    }

    inline static PolarPoint2 inf()
    {
        return PolarPoint2(std::numeric_limits<T>::infinity(),
                           std::numeric_limits<T>::infinity());
    }

    inline static PolarPoint2 max()
    {
        return PolarPoint2(std::numeric_limits<T>::max(),
                           std::numeric_limits<T>::max());
    }

    inline static PolarPoint2 min()
    {
        return PolarPoint2(std::numeric_limits<T>::lowest(),
                           std::numeric_limits<T>::lowest());
    }

private:
    point_t cartesian_;
    T       theta_;
    T       rho_;

    inline void updateCartesian()
    {
        cartesian_(0) = std::cos(theta_) * rho_;
        cartesian_(1) = std::sin(theta_) * rho_;
    }

    inline void updatePolar()
    {
        theta_ = std::atan2(cartesian_(1), cartesian_(0));
        rho_   = cartesian_.length();
    }
};

using PolarPoint2d = PolarPoint2<double>;
using PolarPoint2f = PolarPoint2<float>;
}

#endif // CSLIBS_MATH_2D_POLAR_HPP
