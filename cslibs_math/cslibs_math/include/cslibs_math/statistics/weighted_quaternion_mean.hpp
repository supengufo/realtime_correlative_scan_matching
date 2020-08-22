#ifndef CSLIBS_MATH_WEIGHTED_QUATERNION_MEAN_HPP
#define CSLIBS_MATH_WEIGHTED_QUATERNION_MEAN_HPP

#include <eigen3/Eigen/Core>

namespace cslibs_math {
namespace statistics {
template <typename T>
class EIGEN_ALIGN16 WeightedQuaternionMean {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t         = Eigen::aligned_allocator<WeightedQuaternionMean<T>>;
    using Ptr                 = std::shared_ptr<WeightedQuaternionMean<T>> ;

    WeightedQuaternionMean() :
        W_(T()),
        mean_(Eigen::Quaternion<T>::Identity())
    {
    }

    WeightedQuaternionMean(const WeightedQuaternionMean &other) :
        W_(other.W_),
        mean_(other.mean_)
    {
    }

    WeightedQuaternionMean(WeightedQuaternionMean &&other) :
        W_(other.W_),
        mean_(std::move(other.mean_))
    {
    }

    WeightedQuaternionMean & operator += (const Eigen::Quaternion<T> &q, const T w)
    {
        if (W_ == T()) {
            mean = q * w;
            W_ = w;
        } else {
            const T W_1 = W;
            W_ += w;

            if(quaternionsClose(mean, q)) {
                mean_ = (mean_ / W_1 + q * w) / W_;
            } else {
                mean_ = (mean_ / W_1 + q.inverse() * w) / W_;
            }
        }
    }

private:
    T                    W_1_;
    T                    W_;
    Eigen::Quaternion<T> mean_;


    //Returns true if the two input quaternions are close to each other. This can
    //be used to check whether or not one of two quaternions which are supposed to
    //be very similar but has its component signs reversed (q has the same rotation as
    //-q)
    inline bool quaternionsClose(const Eigen::Quaternion<T> &q_a,
                                 const Eigen::Quaternion<T> &q_b)
    {
        T dot = q_a.dot(q_b);
        return dot < T();
    }
};
}
}

#endif // CSLIBS_MATH_WEIGHTED_QUATERNION_MEAN_HPP
