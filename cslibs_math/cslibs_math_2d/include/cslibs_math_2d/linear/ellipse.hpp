#ifndef CSLIBS_MATH_ELLIPSE_HPP
#define CSLIBS_MATH_ELLIPSE_HPP

#include <cslibs_math/linear/vector.hpp>
#include <cslibs_math/linear/matrix.hpp>
#include <cslibs_math/common/equal.hpp>

#include <eigen3/Eigen/Jacobi>
#include <vector>
#include <limits>
#include <set>

namespace cslibs_math_2d {
template <typename T>
struct EIGEN_ALIGN16 Ellipse
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Ellipse() = default;

    inline Ellipse(T l1, T l2, T cx, T cy, T alpha):
        alpha(alpha),
        axis(std::fabs(l1),std::fabs(l2)),
        center(cx,cy)
    {
        axis_1(0) =  std::cos(alpha);
        axis_1(1) =  std::sin(alpha);
        axis_2(0) = -axis_1(1);
        axis_2(1) =  axis_1(0);

        axis_1 *= axis(0);
        axis_2 *= axis(1);
    }

    inline cslibs_math::linear::Vector<T,2> getPoint(T angle)
    {
        cslibs_math::linear::Vector<T,2> res;
        T x = axis(0) * std::cos(angle);
        T y = axis(1) * std::sin(angle);
        res(0) = std::cos(alpha) * x - std::sin(alpha) *y;
        res(1) = std::sin(alpha) * x + std::cos(alpha) *y;
        res += center;
        return res;
    }

    inline bool equals(const Ellipse<T>& other, T eps = 1e-3)
    {
        bool axis_a = (axis_1 - other.axis_1).length() < eps &&
                      (axis_2 - other.axis_2).length() < eps;

        bool axis_b = (axis_1 + other.axis_1).length() < eps &&
                      (axis_2 + other.axis_2).length() < eps;

        bool axis_c = (axis_1 - other.axis_2).length() < eps &&
                      (axis_2 + other.axis_1).length() < eps;

        bool axis_d = (axis_1 + other.axis_2).length() < eps &&
                      (axis_2 - other.axis_1).length() < eps;

        bool pos  = (center - other.center).length() < eps;
        return pos && (axis_a || axis_b || axis_c || axis_d);
    }

    inline cslibs_math::linear::Matrix<T,2,2> getRotationMatrix()
    {
        cslibs_math::linear::Matrix<T,2,2> r;
        r(0,0) =  std::cos(alpha);
        r(0,1) = -std::sin(alpha);
        r(1,0) =  r(0,1);
        r(1,1) =  r(0,0);
        return r;
    }

    T alpha;
    cslibs_math::linear::Vector<T,2> axis;
    cslibs_math::linear::Vector<T,2> axis_1;    /// 1st axis of the ellipse
    cslibs_math::linear::Vector<T,2> axis_2;    /// 2nd axis of the ellipse
    cslibs_math::linear::Vector<T,2> center;
};


template <typename T>
class EIGEN_ALIGN16 EllipseFit
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<T,Eigen::Dynamic,6> RegressionMatrix;
    using point_t = cslibs_math::linear::Vector<T,2>;
    using points_t = std::vector<point_t, typename point_t::allocator_t>;

    inline  EllipseFit() = default;

    T                                   cost;
    Ellipse<T>                          solution;
    cslibs_math::linear::Matrix<T,6,1>  hyper_params;

    inline bool fit(const points_t& points)
    {
        if (points.size() < 6){
            std::cerr << "At least 6 points are required for an ellipse fit. Given are " + std::to_string(points.size()) + " points." << "\n";
            return false;
        }
        RegressionMatrix reg_mat = getRegressionMatrix(points);
        Eigen::FullPivLU<Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>> lu(reg_mat);
        hyper_params.data() = lu.kernel();
        Eigen::Matrix<T,Eigen::Dynamic, 1> test = reg_mat * hyper_params.data();

        cost = test.norm();

        return ellipseFromHyper();
    }

protected:
    inline RegressionMatrix getRegressionMatrix(const points_t& points)
    {
        RegressionMatrix result = RegressionMatrix::Zero(points.size(),6);
        std::size_t row = 0;
        for(auto p : points){
            result(row,0) = p(0) * p (0);
            result(row,1) = p(1) * p (1);
            result(row,2) = p(0) * p (1);
            result(row,3) = p(0);
            result(row,4) = p(1);
            result(row,5) = 1.0;
            ++row;
        }
        return result;
    }

    inline Eigen::Matrix<T,6,6> getMatrixM(const points_t& points)
    {
        Eigen::Matrix<T,6,6> res = Eigen::Matrix<T,6,6>::Zero(6,6);
        for (auto p : points){
            Eigen::Matrix<T,6,1> xi;
            xi(0,0) = p(0) * p(0);
            xi(1,0) = p(1) * p(1);
            xi(2,0) = p(0) * p(1);
            xi(3,0) = p(0);
            xi(4,0) = p(1);
            xi(5,0) = 1.0;
            res += xi * xi.transpose();
        }
        res /= points.size();
        return res;
    }


    inline bool ellipseFromHyper()
    {
        const T& A = hyper_params(0,0);
        const T& C = hyper_params(1,0);
        const T& B = hyper_params(2,0);
        const T& D = hyper_params(3,0);
        const T& E = hyper_params(4,0);
        const T& F = hyper_params(5,0);
        if (A == 0  && B == 0 && C == 0 && D == 0 && E == 0 && F==0) {
            std::cerr << "Did not find non trivial solution. Ellipse fit not possible." << "\n";
            return false;
        }
        Eigen::Matrix<T,3,3> M0;
        M0(0,0) = F;
        M0(0,1) = 0.5*D;
        M0(0,2) = 0.5*E;
        M0(1,0) = 0.5*D;
        M0(1,1) = A;
        M0(1,2) = 0.5*B;
        M0(2,0) = 0.5*E;
        M0(2,1) = 0.5*B;
        M0(2,2) = C;

        Eigen::Matrix<T,2,2> M;
        M(0,0) = A;
        M(0,1) = 0.5*B;
        M(1,0) = 0.5*B;
        M(1,1) = C;

        T detM0 = M0.determinant();
        T detM = M.determinant();
        T l1,l2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T,2,2> > eigensolver(M);
        if(eigensolver.info() == Eigen::Success){
            Eigen::Matrix<T,2,1> ev = eigensolver.eigenvalues();
            if(std::abs(ev(0,0) -A) <= std::abs(ev(0,0) -C) ){
                l1 = ev(0,0);
                l2 = ev(1,0);
                //        std::cout << "l2 case" << std::endl;
            } else{
                l1 = ev(1,0);
                l2 = ev(0,0);
                //        std::cout << "l1 case" << std::endl;
            }

            T a = std::sqrt(-detM0 /(detM * l1));
            T b = std::sqrt(-detM0 /(detM * l2));
            T cx = (B*E -2*C*D)/(4*A*C -B*B);
            T cy = (B*D -2*A*E)/(4*A*C -B*B);
            T t = (A-C)/B;
            T alpha = std::atan(1/t);
            alpha *= 0.5;

            solution = Ellipse<T>(a,b,cx,cy,alpha);

        } else {
            std::cerr << "Cannot solve parametrization!" << std::endl;
            return false;
        }

        return true;
    }
};
}
#endif // CSLIBS_MATH_2D_ELLIPSE_HPP
