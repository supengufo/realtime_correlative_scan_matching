#ifndef CSLIBS_MATH_2D_TRANSFORM_2D_HPP
#define CSLIBS_MATH_2D_TRANSFORM_2D_HPP

#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math_2d
{
    template <typename T>
    class Transform2;
}

template <typename T>
inline cslibs_math_2d::Vector2<T> operator * (const cslibs_math_2d::Transform2<T> &t,
                                              const cslibs_math_2d::Vector2<T> &v);

namespace cslibs_math_2d {
template <typename T>
class EIGEN_ALIGN16 Transform2 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Transform2<T>>;

    inline Transform2() :
        translation_(T(), T()),
        yaw_(T()),
        sin_(T()),
        cos_(T(1.0))
    {
    }

    inline Transform2(const Vector2<T> &translation,
                       const T            yaw,
                       const T            sin,
                       const T            cos) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin),
        cos_(cos)
    {
    }

    inline explicit Transform2(const Eigen::Matrix<T,3,1> &eigen){
        translation_(eigen(0,2),eigen(1,2));
        Eigen::Matrix<T,3,1> eulerAngle = eigen.eulerAngles(2, 1, 0);

    }

    static inline Transform2 identity()
    {
        return Transform2(T(), T());
    }

    inline Transform2(const T x,
                      const T y) :
        translation_(x, y),
        yaw_(T()),
        sin_(T()),
        cos_(T(1.0))
    {
    }

    inline Transform2(const Vector2<T> &translation) :
        translation_(translation),
        yaw_(T()),
        sin_(T()),
        cos_(T(1.0))
    {
    }

    inline Transform2(const T yaw) :
        Transform2(T(), T(), yaw)
    {
    }

    inline Transform2(const T x,
                      const T y,
                      const T yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2(const Vector2<T> &translation,
                      const T yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2(const Transform2 &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2(Transform2 &&other) :
        translation_(std::move(other.translation_)),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline static Transform2 random()
    {
        const Eigen::Matrix<T,3,1> r = Eigen::Matrix<T,3,1>::Random();
        return Transform2(r(0),r(1),r(2));
    }

    inline Transform2 & operator *= (const Transform2 &other)
    {
        if(yaw_ == T()) {
            translation_ += other.translation_;
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
        } else if (other.yaw_ == T()) {
            translation_ = (*this) * other.translation_;
        } else {
            translation_ = (*this) * other.translation_;
            yaw_ = cslibs_math::common::angle::normalize(yaw_ + other.yaw_);
            const T s = sin_ * other.cos_ + cos_ * other.sin_;
            const T c = cos_ * other.cos_ - sin_ * other.sin_;
            sin_ = s;
            cos_ = c;
        }
        return *this;
    }

    inline Transform2& operator = (const Transform2 &other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform2& operator = (const Eigen::Matrix<T,3,1> &eigen)
    {
        translation_ = eigen.block<2,1>(0,0);
        setYaw(eigen(2));
        return *this;
    }

    inline Transform2& operator = (Transform2 &&other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform2 inverse() const
    {
        return Transform2(Vector2<T>(-(cos_ *  translation_(0) + sin_ * translation_(1)),
                                     -(-sin_ * translation_(0) + cos_ * translation_(1))),
                           -yaw_,
                           -sin_,
                           cos_);
    }

    inline Transform2 operator -() const
    {
        return inverse();
    }

    inline T & tx()
    {
        return translation_(0);
    }

    inline T tx() const
    {
        return translation_(0);
    }

    inline T & ty()
    {
        return translation_(1);
    }

    inline T ty() const
    {
        return translation_(1);
    }

    inline Vector2<T> & translation()
    {
        return translation_;
    }

    inline Vector2<T> const & translation() const
    {
        return translation_;
    }

    inline void setYaw(const T yaw)
    {
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline T yaw() const
    {
        return yaw_;
    }

    inline T sin() const
    {
        return sin_;
    }

    inline T cos() const
    {
        return cos_;
    }

    inline Eigen::Matrix<T,3,1> toEigen() const
    {
        return Eigen::Matrix<T,3,1>(translation_(0), translation_(1), yaw_);
    }

    inline Eigen::Matrix<T,2,2> rotation() const
    {
        Eigen::Matrix<T,2,2> rot;
        rot(0,0) =  cos_;
        rot(0,1) = -sin_;
        rot(1,0) =  sin_;
        rot(1,1) =  cos_;
        return rot;
    }

    inline void setFrom(const Eigen::Matrix<T,3,1> &eigen)
    {
        translation_(0) = eigen(0);
        translation_(1) = eigen(1);
        setYaw(eigen(2));
    }

    inline void setFrom(const T x,
                        const T y,
                        const T yaw)
    {
        translation_(0) = x;
        translation_(1) = y;
        setYaw(yaw);
    }

    inline Transform2 interpolate(const Transform2 &other,
                                   const T ratio) const
    {
        assert(ratio >= 0.0);
        assert(ratio <= 1.0);
        if (ratio == T())
            return *this;

        if (ratio == T(1.0))
            return other;

        const T ratio_inverse = T(1.0) - ratio;
        const Vector2<T> translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const T yaw = cslibs_math::common::angle::normalize(yaw_ + ratio * cslibs_math::common::angle::normalize(other.yaw_ - yaw_));
        return Transform2(translation, yaw);
    }

    inline std::string toString() const
    {
        return "[" + std::to_string(tx()) + "," +
                     std::to_string(ty()) + "," +
                     std::to_string(yaw()) + "]";
    }

private:
    Vector2<T> translation_;
    T           yaw_;
    T           sin_;
    T           cos_;
};
using Transform2d = Transform2<double>;
using Transform2f = Transform2<float>;
}

template <typename T>
inline cslibs_math_2d::Vector2<T> operator * (const cslibs_math_2d::Transform2<T> &t,
                                              const cslibs_math_2d::Vector2<T> &v)
{
    return t.yaw() == T() ? v + t.translation()
                          : cslibs_math_2d::Vector2<T>(t.cos() * v(0) - t.sin() * v(1) + t.translation()(0),
                                                       t.sin() * v(0) + t.cos() * v(1) + t.translation()(1));
}

template <typename T>
inline cslibs_math_2d::Transform2<T> operator * (const cslibs_math_2d::Transform2<T> &a,
                                                 const cslibs_math_2d::Transform2<T> &b)
{
    return a.yaw() == T() ? cslibs_math_2d::Transform2<T>(b.translation() + a.translation(),
                                                          b.yaw(), b.sin(), b.cos())
                          : b.yaw() == T() ? cslibs_math_2d::Transform2<T>(a * b.translation(),
                                                                           a.yaw(), a.sin(), a.cos())
                                           : cslibs_math_2d::Transform2<T>(a * b.translation(),
                                                                           cslibs_math::common::angle::normalize(a.yaw() + b.yaw()),
                                                                           a.sin() * b.cos() + a.cos() * b.sin(),
                                                                           a.cos() * b.cos() - a.sin() * b.sin());
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Transform2<T> &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_TRANSFORM_2D_HPP
