#ifndef CSLIBS_MATH_2D_ICP_HPP
#define CSLIBS_MATH_2D_ICP_HPP

#include <cslibs_math_2d/linear/pointcloud.hpp>

namespace cslibs_math_2d {
namespace algorithms {
namespace icp {
template <typename T>
class EIGEN_ALIGN16 Result {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum Termination {Eps, Iteration};

    using covariance_t = Eigen::Matrix<T,2,2>;
    using transform_t  = Transform2<T>;

    inline Result(const std::size_t iterations = 100,
                  const Termination termination = Iteration,
                  const covariance_t covariance = covariance_t::zero(),
                  const transform_t &transform = transform_t()) :
        iterations_(iterations),
        termination_(termination),
        covariance_(covariance),
        transform_(transform)
    {
    }

    inline std::size_t iterations() const
    {
        return iterations_;
    }

    inline std::size_t& iterations()
    {
        return iterations_;
    }

    inline Termination termination() const
    {
        return termination_;
    }

    inline Termination& termination()
    {
        return termination_;
    }

    inline covariance_t covariance() const
    {
        return covariance_;
    }

    inline covariance_t& covariance() const
    {
        return covariance_;
    }

    inline const transform_t &transform() const
    {
        return transform_;
    }

    inline transform_t &transform()
    {
        return transform_;
    }

private:
    std::size_t iterations_;
    Termination termination_;
    covariance_t covariance_;
    transform_t transform_;
};

template <typename T>
class EIGEN_ALIGN16 Parameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using transform_t = Transform2<T>;

    inline Parameters(const std::size_t max_iterations = 100,
                      const T trans_eps = 1e-4,
                      const T rot_eps = 1e-4,
                      const T max_distance = 0.5,
                      const transform_t &transform = transform_t()) :
        max_iterations_(max_iterations),
        trans_eps_(trans_eps),
        rot_eps_(rot_eps),
        max_distance_(max_distance),
        transform_(transform)
    {
    }

    inline T transEps() const
    {
        return trans_eps_;
    }

    inline T &transEps()
    {
        return trans_eps_;
    }

    inline T rotEps() const
    {
        return rot_eps_;
    }

    inline T &rotEps()
    {
        return rot_eps_;
    }

    inline T maxDistance() const
    {
        return max_distance_;
    }

    inline T &maxDistance()
    {
        return max_distance_;
    }

    inline std::size_t maxIterations() const
    {
        return max_iterations_;
    }

    inline std::size_t & maxIterations()
    {
        return max_iterations_;
    }

    inline const transform_t &transform() const
    {
        return transform_;
    }

    inline transform_t &transform()
    {
        return transform_;
    }

private:
    std::size_t max_iterations_;
    T trans_eps_;
    T rot_eps_;
    T max_distance_;
    transform_t transform_;
};

template <typename T>
inline void apply(const Pointcloud2<T>::ConstPtr &src,
                  const Pointcloud2<T>::ConstPtr &dst,
                  const Parameters<T> &params,
                  Result<T> &r)
{
    const Pointcloud2<T>::points_t &src_points = src->getPoints();
    const Pointcloud2<T>::points_t &dst_points = dst->getPoints();
    const std::size_t src_size = src_points.size();
    const std::size_t dst_size = dst_points.size();

    auto sq = [](const T x) {return x * x;};

    const T trans_eps = sq(params.transEps());
    const T rot_eps = sq(params.rotEps());
    const T max_distance = sq(params.maxDistance());

    Transform2<T> &transform = r.transform();
    transform = params.transform();
    Pointcloud2<T>::points_t src_points_transformed(src_size);

    std::vector<std::size_t> indices(src_size, std::numeric_limits<std::size_t>::max());

    auto is_assigned = [](const std::size_t index)
    {
        return index < std::numeric_limits<std::size_t>::max();
    };

    Point2<T> dst_mean;
    for(const Point2<T> &p : dst_points) {
        src_mean += p;
    }
    dst_mean /= static_cast<T>(dst_size);


    Eigen::Matrix<T,2,2> &S = r.covariance();

    for(std::size_t i = 0 ; i < max_iterations_ ; ++i) {
        std::fill(indices.begin(), indices.end(), std::numeric_limits<std::size_t>::max());

        Point2<T> src_mean;

        /// associate
        for(std::size_t s ; s < src_size ; ++s) {
            Point2<T> &sp = src_points_transformed[s];
            sp = transform * src_points[s];
            std::size_t &index = indices[s];
            src_mean += sp;

            T min_distance = std::numeric_limits<T>::max();
            for(std::size_t d = 0 ; d < dst_size ; ++d) {
                const Point2<T> &dp = dst_points[d];
                const T dist = distance2(dp, sp);
                if(dist < min_distance &&
                        dist < max_distance) {
                    index = d;
                    min_distance = dist;
                }
            }
        }
        src_mean /= static_cast<T>(src_size);

        for(std::size_t s = 0 ; s < src_size ; ++s) {
            const Point2<T> &sp = src_points_transformed[s];
            const std::size_t index = indices[s];
            if(is_assigned(index)) {
                const Point2<T> &dp = dst_points[index];
                S += (sp - src_mean).data() * (dp - dst_mean).data().transpose();
            }

        }

        const T dyaw = std::atan2(S(0,1) - S(1,0), S(0,0) + S(1,1));
        Transform2<T> dt(dyaw);
        dt.translation() = dst_mean - dt * src_mean;
        transform *= dt;

        if(dt.translation().length2() < trans_eps ||
                sq(dyaw) < rot_eps) {
            r.iterations_   = i;
            r.termination() = Result<T>::Eps;
            return;
        }
    }

    r.iterations() = params.maxIterations();
    r.termination = Result<T>::Iteration;
}
}
}

#endif // CSLIBS_MATH_2D_ICP_HPP
