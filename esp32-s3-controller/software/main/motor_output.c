#include <math.h>
#include "leg_config.h"
#include "motor_output.h"

float motor_rev_voltage_4010(float rpm)
{
    const float speed = fabsf(rpm) * (float)M_PI / 30.0f;
    const float voltage = 0.00008f * speed * speed * speed -
                          0.0035f * speed * speed + 0.2322f * speed;
    return copysignf(voltage, rpm);
}

float motor_torque_voltage(unsigned int id, float torque, float rpm)
{
    // Original Motor_InitAll coefficients, mapped to the user's current IDs.
    // shortcut: reference estimates, not calibration of these six motors.
    // ID1 left wheel; ID2/3 left leg phi4/phi1;
    // ID4/5 right leg phi1/phi4; ID6 right wheel.
    static const float torque_per_volt[] = {0.0101f, 0.0299f, 0.0321f,
                                            0.0317f, 0.0316f, 0.0096f};
    if (id < 1 || id > 6 || !isfinite(torque) || !isfinite(rpm)) return NAN;
    float emf;
    if (id == WHEEL_LEFT_ID || id == WHEEL_RIGHT_ID) {
        const float speed = fabsf(rpm) * (float)M_PI / 30.0f;
        emf = copysignf(0.000004f * speed * speed * speed -
                        0.0003f * speed * speed + 0.0266f * speed, rpm);
    } else {
        emf = motor_rev_voltage_4010(rpm);
    }
    // S3 has no battery ADC configuration. Do not use a fabricated voltage
    // in the original empirical motorOutRatio formula; torque scale stays 1.
    const float voltage = torque / torque_per_volt[id - 1] + emf;
    if (!isfinite(voltage)) return NAN;
    return limit_motor_voltage(id, voltage);
}
