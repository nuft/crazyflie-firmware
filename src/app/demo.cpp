/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
#include "config.h"
#include "led.h"
#include "debug.h"
#include "sensors.h"
#include "system.h"
#include "log.h"
}

#define eigen_assert(x)
#define EIGEN_NO_MALLOC

#include <Eigen/Dense>
#include "../../../Kalman/ExtendedKalmanFilter.h"
#include "car_model.h"

// #define BENCHMARK
#ifdef BENCHMARK
#include "benchmark.h"
#endif

float log_x = 0;
float log_y = 0;
float log_v = 0;
float log_theta = 0;

namespace demo {

#define ESTIMATOR_UPDATE_RATE 250.0f // [Hz]
#define STANDARD_GRAVITY 9.80665f // [m/s^2]
#define DEG2RAD ((float)M_PI / 180.f)

using EKF = kalmanfilter::ExtendedKalmanFilter<Dynamics, Observation, kalmanfilter::EULER>;

void main(void *param)
{
    (void) param;
    DEBUG_PRINT("DEMO\n");

    TickType_t last_update;

    // Wait for the system to be fully started to start stabilization loop
    systemWaitStart();

    // Wait for sensors to be calibrated
    last_update = xTaskGetTickCount();
    while (!sensorsAreCalibrated()) {
        vTaskDelayUntil(&last_update, F2T(ESTIMATOR_UPDATE_RATE));
    }

    const float Ts = 1.0f / ESTIMATOR_UPDATE_RATE;

    static EKF::Control u;
    static EKF::Measurement z;
    static EKF::State x;
    static EKF::StateCov P;

    // initial state and covariance
    x << 0, 0, 0, 0;
    P.setZero();
    P.diagonal() << 0.1f, 0.1f, 0.1f, 0.1f;

    static EKF ekf(x, P);

    DEBUG_PRINT("Start EKF\n");
    last_update = xTaskGetTickCount();
    while (1) {
        ledSet(LED_GREEN_L, 1); // for time measurement & debugging

        Axis3f g, a;
        sensorsReadGyroCopy(&g);
        sensorsReadAccCopy(&a);
        // DEBUG_PRINT("%f %f %f %f\n", (double)a.x, (double)a.y, (double)a.z, (double)g.z);

        u.v = 0;
        u.phi = 0;

        z = Observation::from_imu(x,
                                  STANDARD_GRAVITY * a.x,
                                  STANDARD_GRAVITY * a.y,
                                  DEG2RAD * g.z,
                                  Ts);
#ifdef BENCHMARK
        uint32_t count;
        // Prevent the RTOS kernel swapping out the task.
        // vTaskSuspendAll();
        taskENTER_CRITICAL();
        cycle_counter_reset();
#endif

        x = ekf.update(u, z, Ts);


#ifdef BENCHMARK
        count = cycle_counter_get();
        // Resume the RTOS kernel.
        // xTaskResumeAll();
        taskEXIT_CRITICAL();
        DEBUG_PRINT("EKF update nb cycles: %lu\n", count);
#endif

        ledSet(LED_GREEN_L, 0);

        log_x = x(0);
        log_y = x(1);
        log_v = x(2);
        log_theta = x(3);


        // static int i = 0;
        // if (i++ > (int)(ESTIMATOR_UPDATE_RATE/10)) {
        //     i = 0;
        //     DEBUG_PRINT("%f %f %f %f\n", (double)x[0], (double)x[1], (double)x[2], (double)x[3]);
        // }

        vTaskDelayUntil(&last_update, F2T(ESTIMATOR_UPDATE_RATE));
    }
}

} // namespace demo

extern "C" {

void demoInit(void)
{
    xTaskCreate(demo::main, DEMO_TASK_NAME, DEMO_TASK_STACKSIZE, NULL,
                DEMO_TASK_PRI, NULL);
}

LOG_GROUP_START(car_states)
LOG_ADD(LOG_FLOAT, x, &log_x)
LOG_ADD(LOG_FLOAT, y, &log_y)
LOG_ADD(LOG_FLOAT, v, &log_v)
LOG_ADD(LOG_FLOAT, theta, &log_theta)
LOG_GROUP_STOP(car_states)

}
