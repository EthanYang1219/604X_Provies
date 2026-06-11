/**
 * \file Autonomous.cpp
 *
 * All of the autonomous routines for team 604X at the BC Provincial
 * Championship (VEX Over Under, 2023-2024 season).
 *
 * Every route lives in its own named function so routines can be swapped
 * with a single call in autonomous() (see src/main.cpp). Field coordinates
 * are in inches and headings in degrees, relative to the pose given to
 * chassis.setPose() at the start of each routine. Motor power values range
 * from -127 to 127.
 *
 * Routine index
 *   Competition:
 *     farSideAuton()      - safe far-side route (ran at Provincials)
 *     awpAuton()          - win-point route
 *     skillsAuton()       - catapult volley + field crossing skills route
 *     oneMinSkillsAuton() - one-minute skills variant
 *   Legacy / practice:
 *     oldFarSideAuton(), oldAwpAuton(), backWingAwpAuton(),
 *     safeAwpAuton(), goalPushFragment()
 */

#include "main.h"
#include "autons.hpp"

// ===========================================================================
// COMPETITION ROUTINES
// ===========================================================================

/**
 * Safe far-side (offensive zone) routine — the route run at Provincials.
 *
 * Sequence: intake the ball in front, back into the corner to sweep the
 * corner triball out with the back wings, ram it into the side of the goal
 * (twice for a sure score), then collect the two middle-field triballs and
 * push them in with the front wings.
 */
void farSideAuton() {
    chassis.setPose(0, 0, -90);                       // start against the far-side wall, facing the ball
    intake = -127;                                    // start the intake before moving
    chassis.moveToPoint(-7.5, 0, 2000, true, 45);     // creep forward and intake the ball in front
    delay(250);
    chassis.waitUntil(2);
    chassis.waitUntilDone();

    // Drive backwards into the corner and line up on the corner triball
    chassis.moveToPose(27, 0, -90, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 40});   // back toward the corner
    chassis.waitUntilDone();
    chassis.moveToPose(27, 0, -135, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 40});  // rotate 45 deg to face it
    chassis.waitUntilDone();
    chassis.moveToPose(37.4, 8.4, -135, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 40}); // approach the corner ball
    backwings.set_value(true);                        // deploy back wings to sweep the ball out of the corner
    // chassis.moveToPose(39.4,10.4,-160,1500,{.forwards = false, .lead = 0.1}); // (alternate corner approach)
    chassis.waitUntil(2);
    chassis.waitUntilDone();

    // First ram: drive backwards into the goal with the corner ball
    chassis.moveToPose(44, 33, -181, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 60});
    chassis.waitUntil(4);
    backwings.set_value(false);                       // retract wings once the ball is committed
    chassis.waitUntil(6);
    chassis.waitUntilDone();

    // Swing out, turn around, and ram the goal again front-first
    chassis.moveToPose(31.7, 4.7, -139.8, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 50}); // pull away to build momentum
    chassis.waitUntil(6);
    chassis.waitUntilDone();
    chassis.moveToPose(31.7, 3.7, 40, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 50});     // turn ~180 deg
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    chassis.moveToPose(44, 32, 0, 1000, {.forwards = true, .lead = 0.1, .minSpeed = 50});         // close distance to the goal
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    chassis.moveToPose(44, 32, -0, 1000, {.forwards = true, .lead = 0.1});                        // second ram, outtaking the held ball
    intake = 110;
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    intake = 0;

    // Optional second ram cycle (disabled — cost too much time in testing):
    // chassis.moveToPose(31.7,4.7,-139.8,2000,{.forwards = false, .lead = 0.1, .minSpeed =40}); // pull back for momentum
    // chassis.waitUntilDone();
    // chassis.moveToPose(44,32,1,1000,{.forwards = true, .lead = 0.1, .minSpeed =40});          // approach goal
    // chassis.waitUntilDone();
    // chassis.moveToPose(44,32,1,1000,{.forwards = true, .lead = 0.1, .minSpeed =40});          // ram goal
    // chassis.waitUntil(3);
    // chassis.waitUntilDone();

    // Collect the left-middle triball and bring it toward the goal side
    chassis.moveToPose(35.3, 15.9, -327, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 40}); // back off the goal
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    chassis.moveToPose(-4.5, 33.2, -651, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 40});  // drive to the left middle triball
    chassis.waitUntil(10);
    intake = -127;                                    // intake it on approach
    chassis.waitUntilDone();
    chassis.moveToPose(-4.5, 36.2, -290, 2000, {.forwards = true, .lead = 0.01, .minSpeed = 40}); // turn around
    chassis.waitUntil(10);
    intake = 127;                                     // spit it out toward the goal
    chassis.waitUntilDone();

    // Collect the center triball, then push everything in with the wings
    chassis.moveToPose(-8.8, 57.2, 0, 2000, {.forwards = true, .lead = 0.01, .minSpeed = 40});    // drive to the middle ball
    chassis.waitUntil(2);
    intake = -127;
    chassis.waitUntilDone();
    chassis.moveToPose(-8.8, 57.2, 90, 2000, {.forwards = true, .lead = 0.01, .minSpeed = 40});   // face the goal
    chassis.waitUntil(2);
    chassis.waitUntilDone();
    chassis.moveToPose(28, 57.2, 90, 2000, {.forwards = true, .lead = 0.01, .minSpeed = 127});    // full-speed push into the goal
    intake = 127;
    wings.set_value(true);                            // wings out to carry both triballs
    chassis.waitUntil(2);
    chassis.waitUntilDone();
    wings.set_value(false);
    intake = 0;
}

/**
 * Win-point (AWP) routine.
 *
 * Scores the alliance triball into the near goal, descores the corner
 * triball with the back wings, then drives across to touch the hang bar.
 */
void awpAuton() {
    chassis.setPose(0, 1, 0);

    // Score the alliance triball while flicking match loads away with the back wings
    chassis.moveToPose(9.1, 50.99, 18.54, 4000, {.minSpeed = 80}); // long drive to the goal
    intake = 110;                                     // brief outward pulse to drop the preload into position
    chassis.waitUntil(3);
    intake = 0;
    chassis.waitUntil(3);
    intake = -127;                                    // hold the triball while driving
    chassis.waitUntil(40);
    backwings.set_value(true);                        // wing flick on the way past
    chassis.waitUntil(48);
    backwings.set_value(false);
    chassis.waitUntilDone();

    // Multi-step reverse path to line up on the corner triball
    chassis.moveToPose(-4.18, 21.70, 57.07, 2000, {.forwards = false, .lead = 0.4});  // begin backing toward the corner
    chassis.waitUntil(40);
    intake = 0;
    chassis.waitUntilDone();
    chassis.moveToPose(-24.42, 10.01, 78.15, 2000, {.forwards = false, .lead = 0.4}); // swing wide
    chassis.moveToPose(-20.0, 17.39, -30.3, 2000, {.forwards = false, .lead = 0.6});  // square up on the corner

    // Descore the corner triball with a back-wing sweep
    chassis.moveToPose(-13.30, 0.6, -39.15, 1500, {.forwards = false, .lead = 0.1, .minSpeed = 20});
    backwings.set_value(true);
    chassis.moveToPose(-13.30, 0.6, -20.15, 1500, {.forwards = false, .lead = 0.1, .minSpeed = 20}); // sweep across
    delay(500);
    chassis.moveToPose(-13.30, 0.6, -39.15, 1500, {.forwards = false, .lead = 0.1, .minSpeed = 20}); // sweep back
    backwings.set_value(false);
    chassis.waitUntilDone();

    // Reposition, then drive to the hang bar for the AWP touch
    chassis.moveToPose(-21.17, 14.5, -29.15, 1500, {.forwards = true, .lead = 0.4});  // back off the corner
    chassis.moveToPose(-21.17, 14.5, -227, 1500, {.forwards = true, .lead = 0.4});    // turn in place toward the bar
    chassis.moveToPose(20.0, -4.5, 95, 4000, {.lead = 0.8});                          // drive to the hang bar
    chassis.waitUntil(20);
    intake = 127;                                     // outtake so we finish without possession
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.moveToPose(15.0, -4.5, 95, 2000, {.lead = 0.4});                          // nudge to guarantee bar contact
    chassis.moveToPose(26.0, -4.5, 95, 2000, {.lead = 0.4});
}

/**
 * Skills-style routine ("last chance" build).
 *
 * Fires the catapult at the match-load bar for 30 seconds, then crosses
 * to the far side and repeatedly pushes triballs into the far goal using
 * the front wings.
 */
void skillsAuton() {
    // Drive to the match-load bar and volley with the catapult
    chassis.moveToPose(10.46, 9.66, -70, 2000);
    backwings.set_value(true);                        // brace against the bar while firing
    timedcata(30000, 127);                            // 30-second catapult volley
    backwings.set_value(false);

    // Cross the field under the alley
    chassis.moveToPose(-12, -8, -88, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 80});
    chassis.waitUntil(1);
    resetcata();                                      // re-cock the catapult while driving
    chassis.waitUntilDone();
    chassis.moveToPoint(-86, -9, 3000, true, 95);     // long drive down the alley
    intake = 127;
    chassis.waitUntilDone();

    // First push into the far goal from the front
    chassis.moveToPose(-107.5, 26.9, 361, 2000, {.forwards = true, .minSpeed = 100});
    intake = 90;
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(-105, 33, 1000, true, 127);   // ram
    chassis.waitUntilDone();
    chassis.moveToPoint(-105, 10, 1000, false, 127);  // back out
    chassis.waitUntilDone();
    chassis.moveToPoint(-105, 33, 1000, true, 127);   // ram again
    chassis.waitUntilDone();

    // Reposition to the middle of the field for side pushes
    chassis.moveToPoint(-105, 6, 1500, false, 127);
    chassis.waitUntilDone();
    chassis.moveToPose(-61, 33.8, 52.8, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 80});
    chassis.waitUntilDone();
    // chassis.moveToPoint(-54,20.5,2000,false,100); // (older approach)

    // Wing pushes into the goal from the middle barrier side
    chassis.moveToPose(-105, 42, -91, 1500, {.forwards = true, .lead = 0.1, .minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPose(-60, 48, -91, 1500, {.forwards = false, .lead = 0.1, .minSpeed = 80});  // back out for another run
    chassis.waitUntilDone();
    chassis.moveToPose(-105, 53, -91, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 100}); // push with wings out
    wings.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPose(-66.9, 85, -142, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 80});
    wings.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPose(-98, 59, -91, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 100});  // final wing push
    wings.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPose(-70, 73, -44.2, 1500, {.forwards = false, .lead = 0.1, .minSpeed = 80});
    wings.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPose(-95, 112, -39, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 80}); // end clear of the goal
    chassis.waitUntilDone();
}

/**
 * One-minute programming-skills routine.
 *
 * Scores the preloads, fires a short catapult burst from the match-load
 * bar, then crosses the field and pushes triballs into the far goal using
 * the hang piston as a plow.
 */
void oneMinSkillsAuton() {
    chassis.setPose(0, 0, 0);

    // Score the first triball into the near goal
    chassis.moveToPose(10.14, 18, -0.41, 1500);
    intake = 100;
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(11, 30, 1000, true, 127);     // ram it in
    chassis.moveToPoint(10, 8, 2000, false, 80);      // back away

    // Catapult burst from the match-load bar
    chassis.moveToPose(7.5, 4.53, 111, 2000);
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    hangpiston.set_value(true);                       // brace with the hang piston
    // timedcata(30000,-127);                         // full 30s volley (full-skills version)
    timedcata(1000, -127);                            // short burst for the 1-minute run
    hangpiston.set_value(false);

    // Cross the field
    chassis.moveToPose(-12, -5, -88, 2000);
    chassis.waitUntil(1);
    resetcata();                                      // re-cock while driving
    chassis.waitUntilDone();
    chassis.moveToPoint(-80, -5, 3000, true, 95);
    intake = 127;
    chassis.waitUntilDone();

    // Front pushes into the far goal
    chassis.moveToPose(-103, 26.9, 361, 2000);
    intake = 90;
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(-105, 30, 1000, true, 127);   // ram
    chassis.waitUntilDone();
    chassis.moveToPoint(-105, 20, 1000, false, 127);  // back out
    chassis.waitUntilDone();
    chassis.moveToPoint(-105, 30, 1000, true, 127);   // ram again
    chassis.waitUntilDone();

    // Reposition through mid-field
    chassis.moveToPoint(-105, 16, 1500, false, 127);
    chassis.waitUntilDone();
    chassis.moveToPose(-90, 20.5, 92, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54, 20.5, 2000, true, 100);
    chassis.waitUntilDone();
    chassis.moveToPose(-54, 20.5, -5, 1000);
    chassis.waitUntilDone();

    // Side pushes using the hang piston as a plow
    chassis.moveToPose(-95, 55, -90, 2000);
    hangpiston.set_value(true);
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    chassis.moveToPoint(-70, 55, 1000, false, 127);
    chassis.moveToPoint(-100, 55, 1000, true, 127);
    chassis.moveToPoint(-70, 55, 1000, false, 127);   // pushing into the goal
    chassis.waitUntil(1);
    hangpiston.set_value(false);
    chassis.waitUntilDone();

    // Sweep down the far wall and make the final pushes
    chassis.moveToPoint(-54, 55, 1000, false, 127);
    chassis.moveToPose(-50, 55, 0, 2000);
    chassis.moveToPoint(-50, 120, 2500, false, 100);
    chassis.waitUntilDone();
    hangpiston.set_value(true);
    chassis.moveToPose(-83, 55, -90, 3000);
    chassis.moveToPoint(-100, 55, 1000, true, 127);
    chassis.moveToPoint(-70, 55, 1000, false, 127);
    chassis.moveToPoint(-100, 55, 1000, true, 127);
    chassis.moveToPoint(-55, 55, 1000, false, 100);
    hangpiston.set_value(false);
}

// ===========================================================================
// LEGACY / PRACTICE ROUTINES
// These are earlier revisions kept for reference and fallback. They are not
// called by autonomous() but still compile so they can be swapped in quickly.
// ===========================================================================

/**
 * Older far-side route from before the Provincials revision.
 * Collects the lone triball, pushes it over with the hang piston, then
 * works the bottom of the field toward the hang bar.
 */
void oldFarSideAuton() {
    chassis.setPose(0, 0, -34);                       // set the starting pose
    chassis.moveToPose(-36, 56, -34, 2000);           // drive to the triball
    intake = 127;
    chassis.waitUntil(1);
    intake = -127;                                    // intake it
    chassis.waitUntilDone();
    chassis.moveToPose(-36, 56, -40, 500);            // small turn to grasp the ball
    intake = -50;
    chassis.moveToPoint(-1, 52, 2000, 120);           // push toward the goal
    chassis.waitUntil(1);
    hangpiston.set_value(true);
    intake = 127;
    chassis.waitUntil(3);
    intake = 0;
    chassis.waitUntilDone();
    hangpiston.set_value(false);
    chassis.moveToPoint(-12, 52, 3000, false, 120);   // pull back out
    chassis.waitUntilDone();
    chassis.moveToPose(-39.65, 34.92, 256, 4000);     // grab the third ball
    intake = -127;
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(-15, -5, -180, 2500);          // drive to the other side triball position
    intake = -127;
    chassis.waitUntilDone();
    intake = 0;
    chassis.turnTo(-10, -5, 2000, true, 120);
    // chassis.turnTo(-5,0,2000,true,120);
    chassis.moveToPose(10, 12, 49, 3000);
    chassis.waitUntil(5);
    hangpiston.set_value(true);
    chassis.waitUntilDone();
    hangpiston.set_value(false);
    chassis.moveToPoint(-1, 0, 500, false, 120);
    chassis.waitUntilDone();
    chassis.moveToPose(14.5, 20, 3.33, 2000);         // line up on the goal
    intake = 55;
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5, 8, 1000, false, 120);   // push cycles
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5, 27, 1000, true, 127);
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5, 8, 1000, false, 127);
    intake = 0;
}

/**
 * Older AWP route that used the hang piston for the corner descore.
 */
void oldAwpAuton() {
    chassis.setPose(0, 0, 10.4);                      // set the starting pose
    chassis.moveToPose(8, 57, 10, 3000);              // drive to the first triball
    intake = 127;
    chassis.waitUntil(1);
    intake = -127;                                    // intake the triball
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(0, 0, 1000, false);           // return to the start
    intake = -127;
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(0, 0, 117, 1000);              // turn toward the corner
    chassis.moveToPoint(-23, 14.579, 2000, false, 80);
    chassis.moveToPose(-23, 15.579, 179, 2000);
    chassis.moveToPoint(-24, 30, 1200, false);        // push-pull on the goal
    chassis.moveToPoint(-24, 16.8, 1200, true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, 30, 1000, false);
    chassis.moveToPose(-13, 4.5, 137, 2000);          // line up on the corner ball
    chassis.moveToPose(-17.5, 4.58, 86, 2000);
    hangpiston.set_value(true);                       // descore with the hang piston
    chassis.moveToPose(-11.5, 4.58, 90, 2000);
    chassis.waitUntil(4);
    hangpiston.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPose(33, -2, 86, 2000);             // drive to touch the hang bar
    intake = 127;
    chassis.waitUntilDone();
    intake = 0;
}

/**
 * AWP variant that scores with the wings open, then descores the corner
 * with the back wings and finishes touching the hang bar.
 */
void backWingAwpAuton() {
    chassis.setPose(0, 0, 10.4);                      // set the robot's starting pose
    chassis.moveToPose(8, 54, 10, 3000);              // drive to the first triball
    intake = 127;                                     // spin outward to throw off the rubber bands
    chassis.waitUntil(1);
    intake = -127;                                    // reverse to intake the triball
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 30;                                      // hold the triball gently
    chassis.moveToPoint(8, 45, 1000, false);          // back up a bit
    chassis.waitUntilDone();
    chassis.moveToPose(8, 45, 90, 1000, {.forwards = true, .minSpeed = 20}); // face the goal
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 48, 1000, true);          // drive forward into the goal
    wings.set_value(true);                            // wings out for a wider push
    chassis.waitUntilDone();
    intake = 0;
    wings.set_value(false);
    chassis.moveToPose(-12, -1, 50, 3500, {.forwards = false, .lead = 0.08});  // back across to the bar
    chassis.waitUntilDone();
    chassis.moveToPose(-12, -1, -75, 1500, {.forwards = false, .lead = 0.08}); // turn to the corner triball
    backwings.set_value(1);                           // sweep it out with the back wings
    chassis.waitUntilDone();
    chassis.moveToPose(18, -3, 94, 3500, {.forwards = true, .lead = 0.05});    // touch the hanging bar
    backwings.set_value(0);
    intake = 127;
    chassis.waitUntilDone();
}

/**
 * Conservative AWP: intake the alliance ball, descore the corner from a
 * nearly stationary position, then drive to touch the hang bar.
 */
void safeAwpAuton() {
    chassis.setPose(0, 0, -45);                       // start angled into the corner
    // chassis.moveToPose(-8.4,1.6,-40.6, 2000);      // (alternate first move)
    chassis.moveToPoint(-2, 5, 2000, true);           // creep toward the alliance ball
    intake = 110;
    chassis.waitUntil(3);
    intake = 0;
    chassis.waitUntilDone();
    backwings.set_value(true);                        // back wings out for the descore
    chassis.moveToPoint(0, 0, 2000, false);           // return to the corner
    chassis.moveToPose(0, 0, -75, 2000);              // sweep across the corner ball
    chassis.moveToPose(0, 0, -45, 2000);              // and back
    intake = 0;
    backwings.set_value(false);
    chassis.waitUntilDone();
    // chassis.moveToPose(31.5,14.8,90,2000,{.forwards = true, .lead= 0.1, .minSpeed = 80}); // (faster bar approach)
    chassis.moveToPose(46.5, -5, 90, 2000);           // drive to touch the hang bar
    intake = 127;
    chassis.waitUntilDone();
    intake = 0;

    // Alternate corner-descore ending kept from testing:
    // chassis.moveToPose(-13,4.5,137, 2000);
    // chassis.moveToPose(-17.5,4.58,86, 2000);
    // backwings.set_value(true);
    // chassis.moveToPose(-11.5,4.58,90, 2000);
    // chassis.waitUntil(4);
    // backwings.set_value(false);
    // chassis.waitUntilDone();
    // chassis.moveToPose(33,-2,86, 2000);
    // intake = 127;
    // chassis.waitUntilDone();
    // intake = 0;
}

/**
 * Goal-side pushing sequence fragment used while practicing skills.
 * Repeated wing pushes into the far goal from the side and front.
 */
void goalPushFragment() {
    chassis.moveToPoint(-52, 33, 2000, true, 100);
    chassis.waitUntilDone();
    chassis.moveToPose(-52, 33, -90, 2000, {.forwards = true, .lead = 0.1, .minSpeed = 80});
    wings.set_value(true);
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    chassis.moveToPoint(-80, 55, 1000, true, 127);    // first push
    wings.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-100, 55, 1000, false, 127);
    chassis.waitUntilDone();
    chassis.moveToPoint(-70, 55, 1000, true, 127);
    chassis.waitUntil(1);
    wings.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54, 55, 1000, false, 127);
    chassis.waitUntilDone();
    chassis.moveToPose(-50, 55, 0, 2000, {.forwards = false, .lead = 0.1, .minSpeed = 80});
    chassis.waitUntilDone();
    chassis.moveToPoint(-50, 110, 2500, false, 100);  // sweep down the wall
    chassis.waitUntilDone();
    wings.set_value(true);
    chassis.moveToPose(-90, 63, -90, 3000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-100, 55, 1000, true, 127);   // final push cycles
    chassis.waitUntilDone();
    chassis.moveToPoint(-70, 55, 1000, false, 127);
    chassis.moveToPoint(-100, 55, 1000, true, 127);
    chassis.moveToPoint(-55, 55, 1000, false, 100);
    wings.set_value(false);
}

// ===========================================================================
// UNFINISHED DRAFT — "new AWP"
// Preserved exactly as written; never compiled because of a typo on the
// first line ("100]]]" should be a timeout value). Kept for reference only.
// ===========================================================================
/*
void newAwpDraft() {
    chassis.moveToPoint(0,0,100]]],false);
    intake = -127;
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(-10.5,2,-53,1000);
    delay(1000);
    chassis.moveToPose(-24 ,16.5, 0,2000);
    delay(1000);
    chassis.moveToPose(-20,5.5,-47,2000);
    delay(1000);
    chassis.moveToPoint(-13,0.5,1000,false,80);
    delay(1000);
    backwings.set_value(true);
    chassis.waitUntil(2);
    chassis.waitUntilDone();
    backwings.set_value(false);
    chassis.moveToPose(-13, 0.5, -79, 1000);
    delay(1000);
    chassis.moveToPose(-13, 0.5, -51, 1000);
    delay(1000);
    chassis.moveToPose(3.5, 0, 82.8, 2000);
    delay(1000);
    chassis.moveToPose(33,-2,86, 2000);
}
*/
