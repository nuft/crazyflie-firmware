#ifndef CAR_MODEL_H
#define CAR_MODEL_H

#include <math.h>
#include <Eigen/Dense>

struct Control {
    float v;
    float phi;
};

struct Dynamics {
public:
    // mandatory fields for ExtendedKalmanFilter class
    using Scalar = float;
    using State = Eigen::Matrix<float, 4, 1>; // x,y,v,theta
    using Control = struct Control;

    // helper types
    using Jacobian = Eigen::Matrix<float, 4, 4>;
    using ProcessNoiseCov = Eigen::Matrix<float, 4, 4>;

    ProcessNoiseCov Q;
    const float B =  0.1f; // [m] base distance between front and back wheel

    Dynamics()
    {
        Q.setZero();
        Q.diagonal() << 0.01f, 0.01f, 0.01f, 0.01f;
    }

    State operator()(const State &x, const Control &u)
    {
        const float theta = x[3];
        State dx(
            u.v * cosf(theta),
            u.v * sinf(theta),
            0,
            u.v / B * tanf(u.phi)
            );
        return dx;
    }

    Jacobian jacobian(const State &x, const Control &u)
    {
        const float theta = x[3];
        const float ct = cosf(theta);
        const float st = sinf(theta);
        Jacobian J;
        J.setZero();
        J.coeffRef(0, 2) = ct;
        J.coeffRef(0, 3) = -u.v * st;
        J.coeffRef(1, 2) = st;
        J.coeffRef(1, 3) = u.v * ct;
        J.coeffRef(3, 2) = tanf(u.phi) / B;
        return J;
    }
};

struct Observation {
    // mandatory fields for ExtendedKalmanFilter class
    using Measurement = Eigen::Matrix<float, 4, 1>;

    // helper types
    using State = Eigen::Matrix<float, 4, 1>;
    using MeasurementNoiseCov = Eigen::Matrix<float, 4, 4>;
    using Jacobian = Eigen::Matrix<float, 4, 4>;

    MeasurementNoiseCov R;

    Observation()
    {
        R.setZero();
        R.diagonal() << 2e-6f, 2e-6f, 2e-4f, 5e-5f;
    }

    Measurement operator()(const State &x)
    {
        return x;
    }

    Jacobian jacobian(const State &x)
    {
        Jacobian H;
        H.setIdentity();
        return H;
    }

    static Measurement from_imu(State x, float acc_x, float acc_y, float gyro_z, float dt)
    {
        Measurement z;
        const float v = x[2];
        const float theta = x[3];
        const float ct = cosf(theta);
        const float st = sinf(theta);
        z << (0.5f * dt * dt * acc_x + v) * ct + 0.5f * dt * dt * st * acc_y,
        (0.5f * dt * dt * acc_x + v) * st - 0.5f * dt * dt * ct * acc_y,
            dt * acc_x,
            dt * gyro_z;
        return z + x;
    }
};

#endif /* CAR_MODEL_H */
