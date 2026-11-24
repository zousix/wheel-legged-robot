#pragma once

// Raw motor coordinates: torque in N m, speed in rpm. No direction flip here.
// Returns final voltage, or NaN on invalid input/unsupported motor ID.
float motor_torque_voltage(unsigned int id, float torque, float rpm);
float motor_rev_voltage_4010(float rpm);
