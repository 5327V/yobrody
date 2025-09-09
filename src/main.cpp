#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#define MATCH_COLOR true // true for red, false for false
#define ROOM_TEMP 72.0
// field size in inches (12 feet = 144 inches)
constexpr double FIELD_SIZE = 144.0;

// robot-specific offset from sensor to center of robot
// (tune these values depending on where your sensors are mounted)
constexpr double HORIZONTAL_SENSOR_OFFSET = 7.0; // distance from sensor to robot center (x direction)
constexpr double VERTICAL_SENSOR_OFFSET   = 7.0; // distance from sensor to robot center (y direction)
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({2, 11, 15},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-16, -20, -10}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-4);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(17);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -4);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              10 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(13, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            19, // derivative gain (kD) // 7 -> 19
                                            3, // anti windup
                                            1, // small error range, in inches
                                            50, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            200, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             20, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             100, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pros::Link link(1, "my_link", pros::E_LINK_TX); //radio port
std::vector<pros::Rotation> rotation_all = pros::Rotation::get_all_devices();  // All rotation sensors that are connected
std::vector<pros::Motor> motor_all = pros::Motor::get_all_devices();  // All rotation sensors that are connected
pros::Motor bottomIntake(13);
pros::Motor hood(19);
pros::Optical ballSens(14);
pros::Distance leftHorzWallSens(21);
pros::Distance rightHorzWallSens(21);
pros::Distance verticalWallSens(22);
pros::ADIDigitalOut matchloader('B');
pros::ADIDigitalOut middleGoal('D');
pros::ADIDigitalOut middleGoalDescorer('F');
pros::ADIDigitalOut leftWing('E');
pros::ADIDigitalOut rightWing('A');

double xPose = 0;
double yPose = 0;


void relocalizeWithWallLeft() {
    // read distances from sensors
    double horizDist = leftHorzWallSens.get();   // mm
    double vertDist  = verticalWallSens.get();     // mm

    // convert to inches
    horizDist /= 25.4;
    vertDist  /= 25.4;


    xPose = horizDist + HORIZONTAL_SENSOR_OFFSET; //left
    yPose = FIELD_SIZE - vertDist  - VERTICAL_SENSOR_OFFSET; //front

    chassis.setPose(xPose, yPose, chassis.getPose().theta);
}
void relocalizeWithWallRight() {
    // read distances from sensors
    double horizDist = rightHorzWallSens.get();   // mm
    double vertDist  = verticalWallSens.get();     // mm

    // convert to inches
    horizDist /= 25.4;
    vertDist  /= 25.4;

    double xPose = 0;
    double yPose = 0;

    xPose = FIELD_SIZE - horizDist + HORIZONTAL_SENSOR_OFFSET; //right
    yPose = FIELD_SIZE - vertDist  - VERTICAL_SENSOR_OFFSET; //front

    chassis.setPose(xPose, yPose, chassis.getPose().theta);
}

void exitDist(lemlib::Pose target, double dist){
    chassis.waitUntil(fabs(chassis.getPose().distance(target) - dist));
    chassis.cancelMotion();

}

void relativeDriveToDistance(double distance, double timeout, double maxSpeed){
    lemlib::Pose currentPose = chassis.getPose();
    bool forwards;
    if(distance < 0){
        forwards = false;
    } else {
        forwards = true;
    }
    double dx = distance * cos(currentPose.theta);
    double dy = distance * sin(currentPose.theta);
    chassis.moveToPoint(currentPose.x + dx, currentPose.y + dy, timeout, {forwards, maxSpeed});
}
// Global variables
int current_auton_selection = 0;
bool auto_started = false;
double conveyorPower;
double hoodPower;
bool isColorSorting = false;

void scoreLongGoal(){
	middleGoal.set_value(false);
	conveyorPower = 127;
	hoodPower = -127;
	bottomIntake.move(conveyorPower);
	hood.move(hoodPower);
}
void intakeHoard(){
	middleGoal.set_value(false);
	conveyorPower = 127;
	hoodPower = 10;
	bottomIntake.move(conveyorPower);
	hood.move(hoodPower);
}
void scoreMiddleGoal(){
	middleGoal.set_value(true);
	conveyorPower = 127;
	hoodPower = 127;
	bottomIntake.move(conveyorPower);
	hood.move(hoodPower);
}
void outtake(){
	middleGoal.set_value(false);
	conveyorPower = -127;
	hoodPower = 127;
	bottomIntake.move(conveyorPower);
	hood.move(hoodPower);
}
void stopIntake(){
	middleGoal.set_value(false);
	conveyorPower = 0;
	hoodPower = 0;
	bottomIntake.move(conveyorPower);
	hood.move(hoodPower);
}
void antiJam(){
	if(fabs(bottomIntake.get_actual_velocity()) < 5 && conveyorPower > 0 && fabs(bottomIntake.get_torque()) > 0.8 && !isColorSorting){
		bottomIntake.move(-conveyorPower);
		pros::delay(190);
		bottomIntake.move(conveyorPower);
	}
	if(fabs(hood.get_actual_velocity()) < 5 && hoodPower > 0 && fabs(hood.get_torque()) > 0.8 && !isColorSorting){
		hood.move(-hoodPower);
		pros::delay(190);
		hood.move(hoodPower);
	}
    if(isColorSorting){
        if((ballSens.get_hue() > 2 && ballSens.get_hue() < 10) && MATCH_COLOR){
            bottomIntake.move(-127);
            hood.move(127);
        }
        if((ballSens.get_hue() > 100 && ballSens.get_hue() < 150) && !MATCH_COLOR){
            bottomIntake.move(-127);
            hood.move(127);
        }
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    middleGoal.set_value(true);
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    middleGoal.set_value(false);
    ballSens.set_led_pwm(100);
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
}

void disabled() {}

void competition_initialize() {}


void left_elims(){
	pros::Task([=]{
		while(true){
			antiJam();
			pros::delay(20);
		}
	});
    chassis.setPose(0,0,0);
   intakeHoard();
   chassis.turnToHeading(-7, 150);
   chassis.moveToPoint(-11, 23,1400, {.forwards = true, .maxSpeed = 50});
   pros::delay(900);
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.turnToHeading(-150, 600);
   chassis.waitUntilDone();
   matchloader.set_value(false);
   lemlib::Pose tp(-42,-2);
   //chassis.moveToPoint(-9, -5, 3000, {.forwards = true, .maxSpeed = 80});
   chassis.moveToPoint(-42, -2, 2500, {.forwards = true, .maxSpeed = 80}); //edited from 0 to -2
   exitDist(tp, 1);
   chassis.turnToHeading(180, 700);
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(700);
   chassis.tank(0, 0);
   scoreLongGoal();
   pros::delay(1200);
   matchloader.set_value(true);
   chassis.moveToPoint(-35, -10, 800, {.forwards = true, .maxSpeed = 80});
   chassis.turnToHeading(182.4, 600);
   chassis.moveToPoint(-35.5, -39, 1400, {.forwards = true, .maxSpeed = 80});
   intakeHoard();
   pros::delay(1200);
   chassis.moveToPoint(-35, 0, 1000, {.forwards = false, .maxSpeed = 80});
   chassis.turnToHeading(180, 600);
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(300);
   chassis.tank(0, 0);
   scoreLongGoal();
   pros::delay(1000);
   chassis.tank(90, 90);
   pros::delay(400);
   chassis.tank(0, 0);
   pros::delay(10);
   chassis.tank(-100, -100);
   pros::delay(900);
   chassis.tank(0, 0);
}
void right_elims(){
pros::Task([=]{
		while(true){
			antiJam();
			pros::delay(20);
		}
	});	
chassis.setPose(0,0,0);
   intakeHoard();
   chassis.turnToHeading(7, 150);
   chassis.moveToPoint(11, 23,1400, {.forwards = true, .maxSpeed = 50});
   pros::delay(900);
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.turnToHeading(150, 600);
   chassis.waitUntilDone();
   matchloader.set_value(false);
   //chassis.moveToPoint(-9, -5, 3000, {.forwards = true, .maxSpeed = 80});
   chassis.moveToPoint(42, 0, 2500, {.forwards = true, .maxSpeed = 80});
   chassis.waitUntilDone();
   chassis.turnToHeading(180, 700);
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(700);
   chassis.tank(0, 0);
   scoreLongGoal();
   pros::delay(1200);
   matchloader.set_value(true);
   chassis.moveToPoint(36, -10, 800, {.forwards = true, .maxSpeed = 80});
   chassis.turnToHeading(180, 600);
   chassis.moveToPoint(36.5, -39, 1400, {.forwards = true, .maxSpeed = 80});
   intakeHoard();
   pros::delay(1200);
   chassis.moveToPoint(37.5, 0, 1000, {.forwards = false, .maxSpeed = 80});
   chassis.turnToHeading(180, 600);
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(400);
   chassis.tank(0, 0);
   scoreLongGoal();
   pros::delay(1000);
   chassis.tank(90, 90);
   pros::delay(400);
   chassis.tank(0, 0);
   pros::delay(10);
   chassis.tank(-100, -100);
   pros::delay(900);
   chassis.tank(0, 0);
}
void left_elims_middle(){
	pros::Task([=]{
		while(true){
			antiJam();
			pros::delay(20);
		}
	});
   chassis.setPose(0,0,0);
   intakeHoard();
   chassis.turnToHeading(-7, 150);
   chassis.moveToPoint(-11, 23,1400, {.forwards = true, .maxSpeed = 50});
   pros::delay(900);
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.moveToPoint(2.6, 24,1400, {.forwards = false, .maxSpeed = 60});
   chassis.turnToHeading(-135, 600);
   chassis.tank(-70, -70);
   pros::delay(200);
   chassis.tank(0,0);
   rightWing.set_value(true);
   matchloader.set_value(false);
   pros::delay(30);
   chassis.moveToPoint(-44, -2, 2000, {.forwards = true, .maxSpeed = 70});
   chassis.turnToHeading(180,600);
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.tank(100,100);
   pros::delay(1600);
   chassis.tank(0,0);
   chassis.cancelMotion();
   chassis.moveToPose(-39, 0, 180, 700, {.forwards = false, .lead = 0.1, .maxSpeed = 70});
   chassis.moveToPoint(-39, 25, 1600, {.forwards = false, .maxSpeed = 60});
   pros::delay(1400);
   scoreLongGoal();
}
void left_elims_bottom(){
	pros::Task([=]{
		while(true){
			antiJam();
			pros::delay(20);
		}
	});
   chassis.setPose(0,0,0);
   intakeHoard();
   chassis.turnToHeading(-7, 150);
   chassis.moveToPoint(-11, 23,1400, {.forwards = true, .maxSpeed = 50});
   pros::delay(900);
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.moveToPoint(2.6, 24,1400, {.forwards = false, .maxSpeed = 60});
   chassis.turnToHeading(-135, 600);
   chassis.tank(-70, -70);
   pros::delay(200);
   chassis.tank(0,0);
   rightWing.set_value(true);
   matchloader.set_value(false);
   pros::delay(30);
   rightWing.set_value(false);
   chassis.moveToPoint(chassis.getPose().x, 16, 1000, {.forwards = true, .maxSpeed = 90});
   chassis.turnToHeading(90,600);
   chassis.moveToPoint(26, 16, 2000, {.forwards = true, .maxSpeed = 90, .earlyExitRange = 2});
   chassis.moveToPoint(39, 16, 1200, {.forwards = true, .maxSpeed = 70, .earlyExitRange = 1});
   matchloader.set_value(true);
   matchloader.set_value(false);
   chassis.waitUntilDone();
   chassis.waitUntilDone();
   chassis.turnToHeading(-43, 1200, {.direction = AngularDirection::CW_CLOCKWISE});
   chassis.waitUntilDone();
   chassis.tank(80,80);
   pros::delay(1200);
   chassis.tank(0,0);
   outtake();
   pros::delay(100);
   chassis.tank(-80,-80);
   pros::delay(100);
   chassis.tank(127,127);
   pros::delay(100);
   chassis.tank(0,0);
   chassis.moveToPoint(40, 0, 700, {.forwards = false, .maxSpeed = 80});
   chassis.moveToPoint(57, -2, 2000, {.forwards = false, .maxSpeed = 80});
   chassis.turnToHeading(180, 600);
   chassis.waitUntilDone();
   chassis.tank(90, 90);
   pros::delay(2000);
    
}


   /*
   chassis.turnToHeading(-90, 600);
   chassis.turnToHeading(9, 600);
   chassis.moveToPoint(14, 43, 1200, {.forwards = true, .maxSpeed = 60});
   chassis.waitUntilDone();
   chassis.tank(50, 50);
   pros::delay(200);
   outtake();
   chassis.tank(127, -127);
   pros::delay(60);
   chassis.tank(-127, 127);
   pros::delay(60);
   chassis.tank(127, -127);
   pros::delay(60);
   chassis.tank(-127, 127);
   pros::delay(60);
   chassis.tank(0,0);
   pros::delay(1000);
   chassis.tank(-5, -90);
   pros::delay(300);
   chassis.moveToPoint(-20, 10, 3000, {.forwards = false, .maxSpeed = 80});
   chassis.moveToPoint(-35, 6, 1500, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   matchloader.set_value(true);
   chassis.turnToHeading(90, 500);
   chassis.turnToHeading(180, 600);
   chassis.waitUntilDone();
   chassis.tank(80, 80);
   pros::delay(800);
   chassis.moveToPoint(-50, 0, 500, {.forwards = false, .maxSpeed = 100});
    
    chassis.moveToPose(-50, 36, 180, 2000, {.forwards = false, .lead = 0.1, .maxSpeed = 100});
	*/

void autonomous() {

   auto_started = true;

   right_elims();




  
   /*
   chassis.moveToPoint(-13, 23, 5000, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
   pros::delay(1000);
   matchloader.set_value(true);
   chassis.moveToPoint(-20,10, 5000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
   chassis.turnToHeading(160, 600);
   chassis.moveToPoint(-24,20, 800, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
   chassis.waitUntilDone();
   bottomIntake.move(127);
   hood.move(-127);
   pros::delay(1300);
   chassis.moveToPoint(-24,15, 800, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
   chassis.waitUntilDone();
   chassis.turnToHeading(180, 500);
   chassis.moveToPoint(-24,-40, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 80, .earlyExitRange = 1});
   bottomIntake.move(127);
   hood.move(5);
   chassis.waitUntilDone();
   pros::delay(300);
   chassis.moveToPoint(-29,10, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 80, .earlyExitRange = 1});
   chassis.turnToHeading(180, 500);
   chassis.moveToPoint(-29,24, 1700, {.forwards = false, .maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
   chassis.waitUntilDone();
   bottomIntake.move(127);
   hood.move(-127);
   pros::delay(1500);
   chassis.tank(80, 80);
   pros::delay(600);
   chassis.tank(0, 0);
   pros::delay(10);
   chassis.tank(-127, -127);
   pros::delay(1500);
   chassis.tank(0, 0);

*/
   



}

/**
 * Runs in driver control
 */
void opcontrol() {
	bool bPressed;
	bool yPressed;
	bool downPressed;
	bool rightPressed;
    isColorSorting = false;
    int trackerDcs = 0;
    // controller
    // loop to continuously update motors
    while (true) {
		antiJam();
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX, true);

		//intake
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intakeHoard();
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			scoreLongGoal();
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			scoreMiddleGoal();
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			leftWing.set_value(true);
            middleGoalDescorer.set_value(true);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			rightWing.set_value(true);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			outtake();
		}else{
			stopIntake();
			leftWing.set_value(false);
			rightWing.set_value(false);
            middleGoalDescorer.set_value(false);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			bPressed = !bPressed;
			matchloader.set_value(bPressed);
		}
        bool intakeOverheated = bottomIntake.get_temperature() > ROOM_TEMP;
        bool hoodOverheated = hood.get_temperature() > ROOM_TEMP;
        std::vector<double> leftTemps = leftMotors.get_temperature_all();
        std::vector<double> rightTemps = rightMotors.get_temperature_all();
        if(intakeOverheated){
            pros::lcd::print(0, "Intake Overheated");
        }
        if(hoodOverheated){
            pros::lcd::print(1, "Hood Overheated");
        }

        bool leftDriverOverheatedAverage = (leftTemps[0] + leftTemps[1] + leftTemps[2]) / 3 > ROOM_TEMP;
        bool rightDriverOverheatedAverage = (rightTemps[0] + rightTemps[1] + rightTemps[2]) / 3 > ROOM_TEMP;
        if(leftDriverOverheatedAverage || rightDriverOverheatedAverage){
            pros::lcd::print(2, "Drive Overheated");
        }

        //handle radio disconnects
        if(!link.connected()){
            controller.rumble("_");
        }
        //handle tracker disconnects & motor disconnects
        size_t rotPortsConnected = rotation_all.size();
        size_t motorPortsConnected = motor_all.size();
        if(rotPortsConnected < 1){
            controller.rumble("_");
            pros::lcd::print(5, "Tracker Disconnected");
            trackerDcs++;
        }
        if(motorPortsConnected < 7){
            controller.rumble("_");
            pros::lcd::print(6, "Motor(s) Disconnected");
        }
        // print information to the screen
		pros::lcd::print(3, "X: %.2f, Y: %.2f, Z: %.2f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
        pros::lcd::print(4, "LDist: %.2f, RDist: %.2f, BackDist: %.2f", leftHorzWallSens.get()/25.4, rightHorzWallSens.get()/25.4, verticalWallSens.get()/25.4);
        pros::lcd::print(7, "DCS: %d", trackerDcs);
        // delay to save resources
        pros::delay(20);
    }
}


