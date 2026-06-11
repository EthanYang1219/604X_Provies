/**
 * \file functions.cpp
 *
 * Shared helper functions for team 604X: raw drive helpers, in-house PID
 * drive/turn routines (used before the switch to LemLib), and timed
 * subsystem helpers for the intake and catapult.
 *
 * Power values are in the PROS motor range of -127..127.
 */

#include "main.h"
#include "math.h"

/**
 * Zeroes every motor encoder and the inertial sensor rotation.
 * Called at the start of each in-house PID routine.
 */
void resetSens(){
leftFrontMotor.tare_position();
leftMidMotor.tare_position();
leftBackMotor.tare_position();
catapult.tare_position();
rightFrontMotor.tare_position();
rightMidMotor.tare_position();
rightBackMotor.tare_position();
intake.tare_position();
inertial_sensor.tare_rotation();
}

/** Sets all six drive motors to HOLD so the robot resists being pushed. */
void BrakeOn(){
	leftFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftMidMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightMidMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);


}

/** Sets all six drive motors back to COAST for normal driving. */
void BrakeOff(){
	leftFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	leftMidMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	leftBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightMidMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);

}

/**
 * Arcade-style drive: applies forward power plus a turning offset to all
 * six drive motors. Used by opcontrol and by the in-house PID routines.
 *
 * \param powerforward  forward/backward power (-127..127)
 * \param powerturning  turning power, positive = clockwise (-127..127)
 */
void Powerdrive(int powerforward, int powerturning){

	leftFrontMotor = powerforward + powerturning;
	rightFrontMotor = powerforward - powerturning;
	leftMidMotor = powerforward + powerturning;
	rightMidMotor = powerforward - powerturning;
	leftBackMotor = powerforward + powerturning;
	rightBackMotor = powerforward - powerturning;
}

/**
 * Tank-style drive: sets the left and right halves of the drivetrain
 * independently. Used by the arc-drive PID routines.
 *
 * \param leftPower   power for the three left motors (-127..127)
 * \param rightPower  power for the three right motors (-127..127)
 */
void PowerdriveSide(int leftPower, int rightPower){

	leftFrontMotor = leftPower;
	rightFrontMotor = rightPower;
	leftMidMotor = leftPower;
	rightMidMotor = rightPower;
	leftBackMotor = leftPower;
	rightBackMotor = rightPower;
}

/**
 * Drives forward at a fixed power (60) for the given duration, then stops.
 *
 * \param time  duration in milliseconds
 */
void timedmove(int time){
	Powerdrive(60,0);
	delay(time);
	Powerdrive(0,0);
}

/**
 * Drives at a scaled power for the given duration using a busy-wait timer.
 *
 * \param time   duration in milliseconds
 * \param power  power multiplier applied to the base speed
 */
void millisdrive(int time, int power){
	int st = millis();
	while(time > millis() - st){
		Powerdrive(84*power,0 );
	}
	Powerdrive(SpeedCap(0),0);
}

/**
 * Turns in place at a scaled power for the given duration.
 *
 * \param time   duration in milliseconds
 * \param turnp  turning power multiplier
 */
void turning(int time, int turnp){
	int st = millis();
	while(time > millis() - st){
		Powerdrive(0,60*turnp);
	}
	Powerdrive(0,0);
}

/**
 * Bang-bang turn to an absolute heading using the inertial sensor.
 * Blocks until the heading is within 0.5 degrees of the target.
 *
 * \param target  target heading in degrees
 */
void turn(int target){
    while(fabs(target - inertial_sensor.get_rotation()) > 0.5){
        if(target > inertial_sensor.get_rotation()){
            Powerdrive(0,-45);
        }else if(target < inertial_sensor.get_rotation()){
            Powerdrive(0,45);

        }
    }Powerdrive(0,0);
}

/**
 * Bang-bang drive to an encoder position on the left-front motor.
 * Blocks until within 2 ticks of the target.
 *
 * \param target  target position in encoder ticks
 */
void drive(int target){
	leftFrontMotor.tare_position();
    while(abs(target - leftFrontMotor.get_position()) > 2 ){
        if(target > leftFrontMotor.get_position()){
            Powerdrive(30,0);
        }else if(target < leftFrontMotor.get_position()){
            Powerdrive(-30,0);

        }
    }Powerdrive(0,0);

}

/**
 * Converts a distance in inches to motor encoder ticks for the drivetrain
 * (3.25" wheels, 3:5 external gearing, 300 ticks per revolution).
 *
 * \param distance  distance in inches
 * \return equivalent encoder ticks
 */
double InchtoTicks(double distance){
	double external = (double)3/5;
	double internal = (double)300;
	double diameter = 3.25;
	double PI = 3.141;

	return  (double(distance/PI/diameter/external*internal));
}


/**
 * In-house PID turn to an absolute heading using the inertial sensor.
 * The integral term only accumulates within 3 degrees of the target to
 * prevent windup. Exits when within 2.75 degrees or when the timer runs out.
 *
 * \param degrees  target heading in degrees
 * \param kP,kI,kD PID gains
 * \param timer    timeout in milliseconds
 */
void PIDturn (int degrees, double kP, double kI, double kD, int timer){
	resetSens();
	double difference = degrees-inertial_sensor.get_rotation();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();

	while((abs(degrees-inertial_sensor.get_rotation())>2.75) && timer > millis() - st){
			difference = degrees-inertial_sensor.get_rotation();
			if(fabs(degrees-inertial_sensor.get_rotation()) < 3){
			integral += difference;
			}
			derivative = difference - past_difference;
			past_difference = difference;
			power = difference*kP + integral*kI + derivative*kD;
			Powerdrive(0,SpeedCap(power));

			pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", difference);
			pros::screen::print(TEXT_MEDIUM, 2, "Angle: %f \n", rotation_sensor.get_angle());
			pros::screen::print(TEXT_MEDIUM, 3, "exit timer: %d \n");
			pros::delay(20);
		}
	Powerdrive(0,0);
}


/**
 * Clamps a motor power value to the legal -127..127 range.
 *
 * \param speed  requested power
 * \return power clamped to [-127, 127]
 */
int SpeedCap(int speed){
	int limit = 127;

	if(abs(speed) <= limit ){
		return(speed);
	}else if(speed < -limit){
	    return -limit;
	}else {
		return limit;
	}

}

/**
 * Clamps a motor power value to a caller-specified limit.
 *
 * \param speed  requested power
 * \param limit  maximum magnitude allowed
 * \return power clamped to [-limit, limit]
 */
int SpeedCapLimit(int speed, int limit){

	if(abs(speed) <= limit ){
		return(speed);
	}else if(speed < -limit){
	    return -limit;
	}else {
		return limit;
	}

}

/**
 * Runs the intake at the given power for a duration, then stops it.
 *
 * \param time   duration in milliseconds
 * \param power  intake power (-127..127)
 */
void timedintake(int time, int power){
	int seconds = millis();
	while(time > millis() - seconds){
		intake = power;
	}
	intake = 0;
}

/**
 * Runs the catapult at the given power for a duration, then stops it.
 * Used for sustained match-load volleys in skills.
 *
 * \param time   duration in milliseconds
 * \param power  catapult power (-127..127)
 */
void timedcata(int time, int power){
	int seconds = millis();
	while(time > millis() - seconds){
		catapult = power;
	}
	catapult = 0;
}

/**
 * Re-cocks the catapult: pulls it down until the rotation sensor reads
 * the set angle (3700 centidegrees), then stops.
 */
void resetcata(){

	while(rotation_sensor.get_angle() < 3700){
			catapult = 127;

	}
	catapult = 0;

}




/**
 * In-house straight-line PID drive with heading correction.
 * Distance PID runs on the left-front motor encoder while a proportional
 * heading loop (kP = 3.5) holds the robot at 0 degrees. The integral term
 * only accumulates within 10 ticks of the target.
 *
 * \param inches   distance to drive (negative = backwards)
 * \param kP,kI,kD distance PID gains
 * \param time     timeout in milliseconds
 */
void PIDdrive(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = 0;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference;
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference;
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference;

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;


		Powerdrive(SpeedCap(power),r_power);
	}


	Powerdrive(0,0);


}


/**
 * PID arc drive curving to the RIGHT: drives the target distance while a
 * proportional heading loop pulls the robot toward +40 degrees. The right
 * side gets the heading correction (left capped at 60, right at 95).
 *
 * \param inches   distance to drive
 * \param kP,kI,kD distance PID gains
 * \param time     timeout in milliseconds
 */
void PIDdriverightArc(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = 40;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference;
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference;
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference;

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;


		PowerdriveSide(SpeedCapLimit(power*2,60),SpeedCapLimit(power*2+r_power, 95));
	}


	Powerdrive(0,0);


}

/**
 * PID arc drive curving to the LEFT: mirror of PIDdriverightArc with a
 * -60 degree heading target and the correction applied to the left side.
 *
 * \param inches   distance to drive
 * \param kP,kI,kD distance PID gains
 * \param time     timeout in milliseconds
 */
void PIDdriveleftArc(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = -60;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference;
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference;
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference;

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;


		PowerdriveSide(SpeedCapLimit(power*2+r_power, 95),SpeedCapLimit(power*2,60));
	}


	Powerdrive(0,0);


}

/**
 * Reversed (backwards) PID arc drive to the right: same structure as
 * PIDdriverightArc but with negated drive power and heading gain so the
 * robot arcs while driving in reverse.
 *
 * \param inches   distance to drive
 * \param kP,kI,kD distance PID gains
 * \param time     timeout in milliseconds
 */
void PIDfdriverightArc(int inches, double kP, double kI, double kD, int time) {
    resetSens();
    double target = InchtoTicks(inches);
    double difference = target - leftFrontMotor.get_position();
    double power;
    double integral = 0.0;
    double past_difference = 0.0;
    double derivative;

    // Turning
    double degrees = 40;
    int r_power;
    double r_integral = 0.0;
    double r_past_difference = 0.0;
    double r_derivative;
    double r_difference;
    double r_kP = -3.5;  // Adjust the sign to turn in the negative direction
    double r_kI = 0;
    double r_kD = 0;

    int st = millis();

    while ((fabs(target - leftFrontMotor.get_position()) > 5 || (fabs(degrees - inertial_sensor.get_rotation()) > 0.75)) && time > millis() - st) {
        pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
        pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);

        // Forward movement PID
        past_difference = difference;
        difference = target - leftFrontMotor.get_position();
        derivative = difference - past_difference;

        if (fabs(target - leftFrontMotor.get_position()) < 10) {
            integral += difference;
        }

        power = difference * kP + integral * kI + derivative * kD;

        // Turning PID
        r_past_difference = r_difference;
        r_difference = degrees - inertial_sensor.get_rotation();
        r_derivative = r_difference - r_past_difference;

        if (fabs(degrees - inertial_sensor.get_rotation()) < 2.5) {
            r_integral += r_difference;
        }

        r_power = r_difference * r_kP;

        // Adjust the sign of the power for both forward movement and turning
        PowerdriveSide(SpeedCapLimit(-power * 2, 60), SpeedCapLimit(-power * 2 + r_power, 95));
    }
}
