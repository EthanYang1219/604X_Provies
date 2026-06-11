/**
 * \file autons.hpp
 *
 * Declarations for every autonomous routine available to team 604X.
 * Each routine is implemented in src/Autonomous.cpp; the one that runs
 * during a match is selected inside autonomous() in src/main.cpp.
 */

#ifndef _AUTONS_HPP_
#define _AUTONS_HPP_

#include "lemlib/api.hpp"

// The LemLib chassis is constructed in src/main.cpp and shared with the
// autonomous routines through this declaration.
extern lemlib::Chassis chassis;

// ----- Competition routines (BC Provincial Championship) -----

/// Safe far-side (offensive zone) routine — the route run at Provincials.
/// Scores the corner ball and rams two triballs into the side of the goal.
void farSideAuton();

/// Win-point (AWP) routine: scores the alliance triball, descores the
/// corner ball with the back wings, and touches the hang bar.
void awpAuton();

/// Skills-style routine: opens the back wings for a long catapult volley
/// of match loads, then crosses the field and pushes triballs into the
/// far goal with the wings.
void skillsAuton();

/// One-minute programming-skills routine that uses the hang piston as a
/// plow while pushing triballs into the far goal.
void oneMinSkillsAuton();

// ----- Legacy / practice routines (kept for reference and testing) -----

/// Older far-side route from before the Provincials revision.
void oldFarSideAuton();

/// Older AWP route that used the hang piston for the corner descore.
void oldAwpAuton();

/// AWP variant that scores the alliance ball with the wings open, then
/// descores the corner with the back wings and touches the bar.
void backWingAwpAuton();

/// Conservative AWP: intake the alliance ball, descore the corner from a
/// nearly stationary position, then drive to touch the hang bar.
void safeAwpAuton();

/// Goal-side pushing sequence fragment used while practicing skills.
void goalPushFragment();

#endif // _AUTONS_HPP_
