/**
 * \file functions.hpp
 *
 * Declarations for the shared helper functions in src/functions.cpp:
 * raw drive helpers, in-house PID drive/turn routines, and timed
 * subsystem helpers for the intake and catapult.
 */

#ifndef _FUNCTIONS_HPP_
#define _FUNCTIONS_HPP_

// --- Brake mode helpers ---
void BrakeOn();   ///< set all drive motors to HOLD
void BrakeOff();  ///< set all drive motors to COAST

// --- Raw drive helpers ---
void Powerdrive(int powerforward, int powerturning); ///< arcade drive
void PowerdriveSide(int leftPower, int rightPower);  ///< tank drive (left/right halves)
void timedmove(int time);                            ///< fixed-power drive for a duration
void millisdrive(int time, int power);               ///< scaled-power drive for a duration
void turning(int time, int turnp);                   ///< timed in-place turn
void turn(int target);                               ///< bang-bang turn to a heading (deg)
void drive(int target);                              ///< bang-bang drive to an encoder target

// --- Subsystem helpers ---
void timedintake(int time, int power); ///< run intake for a duration
void timedcata(int time, int power);   ///< run catapult for a duration (skills volleys)
void resetcata();                      ///< re-cock the catapult via the rotation sensor

// --- Unit conversion ---
double InchtoTicks(double distance); ///< inches -> drive encoder ticks

// --- In-house PID routines (pre-LemLib) ---
void PIDturn(int degrees, double kP, double kI, double kD, int timer);          ///< PID turn to a heading
int SpeedCap(int speed);                                                        ///< clamp power to +/-127
int SpeedCapLimit(int speed, int limit);                                        ///< clamp power to +/-limit
void PIDdrive(int inches, double kP, double kI, double kD, int time);           ///< straight PID drive w/ heading hold
void PIDdriverightArc(int inches, double kP, double kI, double kD, int time);   ///< PID arc drive, curving right
void PIDdriveleftArc(int inches, double kP, double kI, double kD, int time);    ///< PID arc drive, curving left
void PIDfdriverightArc(int inches, double kP, double kI, double kD, int time);  ///< reversed PID arc drive, right
void resetSens();                                                               ///< zero encoders + IMU rotation

#endif // _FUNCTIONS_HPP_
