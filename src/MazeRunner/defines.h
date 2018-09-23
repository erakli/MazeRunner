/* This file is contains defines for conditional compilation. For example, it's
 * used for configuring things like debug output.
 */

#ifndef DEFINES_H
#define DEFINES_H

#define DEBUG true

#if DEBUG
#define MOTOR_DEBUG false
#define ORIENTATION_DEBUG false
#define PID_DEBUG false
#endif

#endif