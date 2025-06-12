#pragma once
#include <Eigen/Dense>

/**
 * @brief A class representing a equality or inequality Task_qp expressed.
 * If equality Task_qp, it expressed as A * x = b.
 * If inequality Task_qp, it expressed as A * x <= b.
 */
class Task_qp
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Task_qp() = default;

    template <typename MatrixType, typename VectorType>
    Task_qp(MatrixType &&A, VectorType &&b)
        : A_(std::forward<MatrixType>(A)), b_(std::forward<VectorType>(b))
    {
        assert(A_.rows() == b_.rows() &&
               "Matrix A and vector b must have the same number of rows");
    }

    Task_qp operator+(const Task_qp &rhs) const
    {
        return Task_qp(concatenateMat(A_, rhs.A_),
                    concatenateVec(b_, rhs.b_));
    }

    Task_qp &operator+=(const Task_qp &rhs)
    {
        A_ = concatenateMat(A_, rhs.A_);
        b_ = concatenateVec(b_, rhs.b_);
        return *this;
    }

    Task_qp operator*(double rhs) const
    {
        return Task_qp(A_.size() > 0 ? rhs * A_ : A_,
                    b_.size() > 0 ? rhs * b_ : b_);
    }

    Task_qp &operator*=(double rhs)
    {
        if (A_.size() > 0)
            A_ *= rhs;
        if (b_.size() > 0)
            b_ *= rhs;
        return *this;
    }

    const Eigen::MatrixXd &A() const { return A_; }
    const Eigen::VectorXd &b() const { return b_; }

private:
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;

    static Eigen::MatrixXd concatenateMat(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2)
    {
        if (m1.size() == 0)
            return m2;
        if (m2.size() == 0)
            return m1;

        assert(m1.cols() == m2.cols() && "Matrix column count must match for concatenation");

        Eigen::MatrixXd res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;
        return res;
    }

    static Eigen::VectorXd concatenateVec(const Eigen::VectorXd &v1, const Eigen::VectorXd &v2)
    {
        if (v1.size() == 0)
            return v2;
        if (v2.size() == 0)
            return v1;

        Eigen::VectorXd res(v1.size() + v2.size());
        res << v1, v2;
        return res;
    }
};
