#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include "NumericalIntegration.h"

namespace kalmanfilter {

template <typename Dynamics, typename Observation, IntegrationMode mode = EULER, int integration_steps = 1>
class ExtendedKalmanFilter {
public:
    using Scalar = typename Dynamics::State::Scalar;
    using State = typename Dynamics::State;
    using Control = typename Dynamics::Control;
    using Measurement = typename Observation::Measurement;
    using StateCov = Eigen::Matrix<Scalar, State::RowsAtCompileTime, State::RowsAtCompileTime>;
    enum {
        nx = State::RowsAtCompileTime,
        nz = Measurement::RowsAtCompileTime,
    };

    NumericalIntegration<Dynamics, mode> f;
    Observation h;
    State x; // state vector
    StateCov P; // state covariance
    Eigen::Matrix<Scalar, nz, nz> S; // innovation covariance
    Eigen::Matrix<Scalar, nx, nz> K; // Kalman gain
    Eigen::Matrix<Scalar, nx, nx> I; // identity

    ExtendedKalmanFilter(State x0, StateCov P0): x(x0), P(P0)
    {
        I.setIdentity();
    }

    void predict(const Control &u, Scalar delta_t)
    {
        Eigen::Matrix<Scalar, nx, nx> A;

        A = I + delta_t * f.jacobian(x, u);

        x = f.integrate(delta_t, delta_t / integration_steps, x, u);
        P = A * P * A.transpose() + f.Q;
    }

    void correct(const Measurement &z)
    {
        Measurement y;                      // innovation
        Eigen::Matrix<Scalar, nz, nx> H;    // jacobian of h
        Eigen::Matrix<Scalar, nx, nx> IKH;  // temporary matrix

        H = h.jacobian(x);
        S = H * P * H.transpose() + h.R;

        // efficiently compute: K = P * H.transpose() * S.inverse();
        K = S.llt().solve(H*P).transpose();

        y = z - h(x);
        IKH = (I - K * H);

        // measurement update
        x = x + K * y;
        P = IKH * P * IKH.transpose() + K * h.R * K.transpose();
    }

    State update(const Control &u, const Measurement &z, Scalar delta_t)
    {
        predict(u, delta_t);
        correct(z);
        return x;
    }
};

} // end namespace kalmanfilter

#endif /* EXTENDED_KALMAN_FILTER_H */
