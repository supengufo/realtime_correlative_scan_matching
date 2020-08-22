#ifndef CSLIBS_MATH_3D_ICP_HPP
#define CSLIBS_MATH_3D_ICP_HPP

#include <cslibs_math_3d/linear/pointcloud.hpp>

namespace cslibs_math_3d {
namespace algorithms {
namespace icp {
template <typename T>
class EIGEN_ALIGN16 Result {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Termination {EPS, ITERATIONS};

    using covariance_t = Eigen::Matrix<T,3,3>;
    using transform_t  = Transform3<T>;

    inline Result(const std::size_t iterations = 100,
                  const Termination termination = ITERATIONS,
                  const covariance_t covariance = covariance_t::Zero(),
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

    inline const covariance_t& covariance() const
    {
        return covariance_;
    }

    inline covariance_t& covariance()
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

    using transform_t = Transform3<T>;

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
    T           trans_eps_;
    T           rot_eps_;
    T           max_distance_;
    transform_t transform_;
};

template<typename T, typename src_iterator_t, typename dst_iterator_t>
inline void apply(const src_iterator_t &src_begin,
                  const src_iterator_t &src_end,
                  const dst_iterator_t &dst_begin,
                  const dst_iterator_t &dst_end,
                  const Parameters<T> &params,
                  Result<T> &r)
{
    const std::size_t src_size = std::distance(src_begin, src_end);
    const std::size_t dst_size = std::distance(dst_begin, dst_end);

    auto sq = [](const T x) {return x * x;};

    const T trans_eps = sq(params.transEps());
    const T rot_eps = sq(params.rotEps());
    const T max_distance = sq(params.maxDistance());
    const std::size_t max_iterations = params.maxIterations();

    Transform3<T> &transform = r.transform();
    transform = params.transform();
    typename Pointcloud3<T>::points_t src_points_transformed(src_size);

    std::vector<std::size_t> indices(src_size, std::numeric_limits<std::size_t>::max());

    auto is_assigned = [](const std::size_t index)
    {
        return index < std::numeric_limits<std::size_t>::max();
    };

    Point3<T> dst_mean;
    for(auto itr = dst_begin; itr != dst_end; ++itr) {
        dst_mean += *itr;
    }
    dst_mean /= static_cast<T>(dst_size);


    Eigen::Matrix<T, 3, 3> &S = r.covariance();

    for(std::size_t i = 0 ; i < max_iterations ; ++i) {
        std::fill(indices.begin(), indices.end(), std::numeric_limits<std::size_t>::max());

        Point3<T> src_mean;

        /// associate
        for(std::size_t s = 0 ; s < src_size ; ++s) {
            Point3<T> &sp = src_points_transformed[s];
            sp = transform * *std::next(src_begin, s);
            std::size_t &index = indices[s];
            src_mean += sp;

            T min_distance = std::numeric_limits<T>::max();
            for(std::size_t d = 0 ; d < dst_size ; ++d) {
                const Point3<T> &dp = *std::next(dst_begin, d);
                const T dist = distance2(dp, sp);
                if(dist < min_distance &&
                        dist < max_distance) {
                    index = d;
                    min_distance = dist;
                }
            }
        }
        src_mean /= static_cast<T>(src_size);

        S = Eigen::Matrix<T, 3, 3>::Zero();
        for(std::size_t s = 0 ; s < src_size ; ++s) {
            const Point3<T> &sp = src_points_transformed[s];
            const std::size_t index = indices[s];
            if(is_assigned(index)) {
                const Point3<T> &dp = *std::next(dst_begin, index);
                S += (sp - src_mean).data() * (dp - dst_mean).data().transpose();
            }

        }

        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3> > svd (S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 3> R =(svd.matrixU() * svd.matrixV().transpose()).transpose();
        // Eigen::Matrix<T, 3, 1> T = dst_mean.data() - R * src_mean.data();
        Eigen::Quaternion<T> qe(R);

        Quaterniond   q(qe.x(), qe.y(), qe.z(), qe.w());
        Transform3<T> dt(dst_mean - q * src_mean,
                         q);
        transform *= dt;

        if(dt.translation().length2() < trans_eps ||
                sq(q.angle(Quaterniond())) < rot_eps) {
            r.iterations()  = i;
            r.termination() = Result<T>::EPS;
            return;
        }
    }

    r.iterations() = params.maxIterations();
    r.termination() = Result<T>::ITERATIONS;
}

template <typename T>
inline void apply(const typename Pointcloud3<T>::ConstPtr &src,
                  const typename Pointcloud3<T>::ConstPtr &dst,
                  const Parameters<T> &params,
                  Result<T> &r)
{
    apply<T>(src->begin(), src->end(), dst->begin(), dst->end(), params, r);
}

}
}
}

#endif // CSLIBS_MATH_3D_ICP_HPP
