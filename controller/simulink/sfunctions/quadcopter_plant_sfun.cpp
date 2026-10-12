#define S_FUNCTION_NAME quadcopter_plant_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include "quadcopter/motor_dynamics.hpp"
#include "quadcopter/quadcopter_dynamics.hpp"
#include "quadcopter/quadcopter_state.hpp"
#include "quadcopter/rigid_body_dynamics.hpp"
#include "quadcopter/rk4_integrator.hpp"
#include "quadcopter/rotor_dynamics.hpp"

#include <cmath>
#include <exception>
#include <new>

using namespace quadcopter;

namespace {

constexpr int STATE_COUNT = 17;
constexpr int INPUT_COUNT = 4;
constexpr int OUTPUT_COUNT = 9;
constexpr double SAMPLE_TIME = 0.001;

struct PlantContext {
    RigidBodyDynamics rigid_body;
    RotorDynamics rotor;
    MotorDynamics motor;
    QuadcopterDynamics dynamics;
    Rk4Integrator integrator;

    PlantContext()
        : rigid_body(
              RigidBodyParameters{
                  1.2,
                  9.80665,
                  {0.015, 0.015, 0.025}
              }),
          rotor(
              RotorParameters{
                  8.0e-6,
                  1.5e-7,
                  1200.0,
                  0.141421356,
                  {1.0, -1.0, 1.0, -1.0}
              }),
          motor(MotorParameters{0.04, 1200.0}),
          dynamics(rigid_body, rotor, motor),
          integrator(dynamics) {}
};

QuadcopterState read_state(const real_T* x)
{
    QuadcopterState s{};

    for (int i = 0; i < 3; ++i) {
        s.rigid_body.position_ned_m[i] = x[i];
        s.rigid_body.velocity_body_m_s[i] = x[i + 3];
        s.rigid_body.angular_velocity_body_rad_s[i] = x[i + 10];
    }

    for (int i = 0; i < 4; ++i) {
        s.rigid_body.quaternion_nb[i] = x[i + 6];
        s.rotor_speeds_rad_s[i] = x[i + 13];
    }

    return s;
}

void write_state(real_T* x, const QuadcopterState& s)
{
    for (int i = 0; i < 3; ++i) {
        x[i] = s.rigid_body.position_ned_m[i];
        x[i + 3] = s.rigid_body.velocity_body_m_s[i];
        x[i + 10] = s.rigid_body.angular_velocity_body_rad_s[i];
    }

    for (int i = 0; i < 4; ++i) {
        x[i + 6] = s.rigid_body.quaternion_nb[i];
        x[i + 13] = s.rotor_speeds_rad_s[i];
    }
}

} // namespace

static void mdlInitializeSizes(SimStruct* S)
{
    ssSetNumSFcnParams(S, 0);

    if (!ssSetNumInputPorts(S, 1)) {
        return;
    }

    ssSetInputPortWidth(S, 0, INPUT_COUNT);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) {
        return;
    }

    ssSetOutputPortWidth(S, 0, OUTPUT_COUNT);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetNumDiscStates(S, STATE_COUNT);
    ssSetNumPWork(S, 1);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct* S)
{
    PlantContext* context = new (std::nothrow) PlantContext();

    if (context == nullptr) {
        ssSetErrorStatus(
            S, "Failed to allocate the quadcopter plant context.");
        return;
    }

    ssSetPWorkValue(S, 0, context);

    real_T* x = ssGetRealDiscStates(S);

    for (int i = 0; i < STATE_COUNT; ++i) {
        x[i] = 0.0;
    }

    // Scalar-first identity quaternion [qw, qx, qy, qz].
    x[6] = 1.0;

    // Initialize actual rotor speeds at hover.
    const double hover_speed = std::sqrt(
        (1.2 * 9.80665) / (4.0 * 8.0e-6));

    for (int i = 0; i < 4; ++i) {
        x[13 + i] = hover_speed;
    }
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
    UNUSED_ARG(tid);

    const real_T* x = ssGetRealDiscStates(S);
    real_T* y = ssGetOutputPortRealSignal(S, 0);

    const QuadcopterState state = read_state(x);
    const auto& q = state.rigid_body.quaternion_nb;

    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];

    // Outputs 1-2: roll and pitch angles in radians.
    y[0] = std::atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy));

    double sin_pitch = 2.0 * (qw * qy - qz * qx);

    if (sin_pitch > 1.0) {
        sin_pitch = 1.0;
    } else if (sin_pitch < -1.0) {
        sin_pitch = -1.0;
    }

    y[1] = std::asin(sin_pitch);

    // Outputs 3-5: measured body rates p, q, r in rad/s.
    y[2] = x[10];
    y[3] = x[11];
    y[4] = x[12];

    // Outputs 6-9: actual rotor speeds in rad/s.
    for (int i = 0; i < 4; ++i) {
        y[5 + i] = x[13 + i];
    }
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct* S, int_T tid)
{
    UNUSED_ARG(tid);

    auto* context = static_cast<PlantContext*>(
        ssGetPWorkValue(S, 0));

    if (context == nullptr) {
        ssSetErrorStatus(
            S, "Quadcopter plant context is unavailable.");
        return;
    }

    const real_T* u = ssGetInputPortRealSignal(S, 0);
    real_T* x = ssGetRealDiscStates(S);

    RotorSpeeds commands{};

    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(u[i]) ||
            u[i] < 0.0 ||
            u[i] > 1200.0) {

            ssSetErrorStatus(
                S,
                "Motor commands must be finite and between 0 and 1200 rad/s.");
            return;
        }

        commands[i] = u[i];
    }

    try {
        const QuadcopterState current = read_state(x);

        const QuadcopterState next = context->integrator.step(
            current, commands, SAMPLE_TIME);

        write_state(x, next);
    }
    catch (const std::exception& e) {
        ssSetErrorStatus(S, e.what());
    }
    catch (...) {
        ssSetErrorStatus(
            S, "Unknown error during quadcopter plant integration.");
    }
}

static void mdlTerminate(SimStruct* S)
{
    auto* context = static_cast<PlantContext*>(
        ssGetPWorkValue(S, 0));

    delete context;
    ssSetPWorkValue(S, 0, nullptr);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif