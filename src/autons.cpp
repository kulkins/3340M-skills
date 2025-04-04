#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */


 //hello this is gio
void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .33005, .035, 1.8, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

void PID_Tuning(){
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(50);
}
void awp_blue_neg(){
  intake1.setVelocity(100, percent);
  togglePiston();
  chassis.set_heading(90);
  chassis.drive_max_voltage = 12.0;
  chassis.drive_distance(-19);
  togglePiston(); //Get the mobile goal
   wait(20,msec);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(30);
  intake1.spin(reverse); //Preparing intake to pick up the rings
  chassis.drive_distance(20);
  chassis.drive_distance(-8);
  chassis.drive_distance(18); //grabbing the ring
  wait(20,msec);
  chassis.turn_to_angle(-45);

  chassis.drive_distance(4); //double check scoring the second ring
  chassis.drive_distance(-4);
  chassis.drive_distance(4); 
  chassis.drive_distance(-4);
  chassis.turn_to_angle(212);
  chassis.drive_distance(50);

//  chassis.drive_distance(10);
//  chassis.turn_to_angle(-20);
  //chassis.turn_to_angle(240);
  //chassis.drive_distance(7); 
}
void blue_pos(){
  intake1.setVelocity(100, percent);
  Clamp.set(true);
  chassis.turn_to_angle(25);
  chassis.drive_distance(30);
  intake1.spin(reverse);
  wait(1.5, seconds);
  intake1.stop(vex::brakeType::hold);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-16);
  Clamp.set(false);
  // chassis.drive_distance(20);
  // Intake.spin(reverse);
  // wait(2, seconds);
  // Intake.stop(vex::brakeType::hold);
}


void blue_neg(){
  intake1.setVelocity(100, percent);
  togglePiston();
  chassis.set_heading(90);
  chassis.drive_max_voltage = 12.0;
  chassis.drive_distance(-20);
  togglePiston(); //Get the mobile goal
   wait(20,msec);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(35);
  intake1.spin(reverse); //Preparing intake to pick up the rings
  chassis.drive_distance(20);
  chassis.drive_distance(-8);
  chassis.drive_distance(18); //grabbing the ring
  wait(20,msec);
  chassis.turn_to_angle(-45);

  chassis.drive_distance(4); //double check scoring the second ring
  chassis.drive_distance(-4);
  chassis.drive_distance(4); 
  chassis.drive_distance(-4);

  chassis.drive_distance(10);
  chassis.turn_to_angle(-20);
  //chassis.turn_to_angle(240);
  //chassis.drive_distance(7); 
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    //task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void autonREDN(){
  intake1.setVelocity(100,pct);
  intake2.setVelocity(100,pct);
  chassis.set_heading(0);
  chassis.drive_distance(-17);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-6);
  intake1.spin(reverse);
  wait(350,msec);
  chassis.drive_distance(6);
  Clamp.set(true);
  chassis.turn_to_angle(230);
  chassis.drive_distance(-33);
  chassis.drive_max_voltage=2;
  chassis.drive_distance(-8);
  Clamp.set(false);
  wait(100,msec);
  chassis.drive_max_voltage=12;
  chassis.turn_to_angle(45);
  intake2.spin(reverse);
  chassis.drive_distance(22);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-10);
  chassis.drive_max_voltage = 12;
  chassis.turn_to_angle(340);
  chassis.drive_distance(20);
  chassis.turn_to_angle(180);
  chassis.drive_distance(25);
}
void autonBLUEN(){
  intake1.setVelocity(100,pct);
  intake2.setVelocity(100,pct);
  chassis.set_heading(180);
  chassis.drive_distance(-17);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(-6);
  intake1.spin(reverse);
  wait(350,msec);
  chassis.drive_distance(6);
  Clamp.set(true);
  chassis.turn_to_angle(-230);
  chassis.drive_distance(-33);
  chassis.drive_max_voltage=2;
  chassis.drive_distance(-6);
  Clamp.set(false);
  wait(100,msec);
  chassis.drive_max_voltage=12;
  chassis.turn_to_angle(-45);
  intake2.spin(reverse);
  chassis.drive_distance(22);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-10);
  chassis.drive_max_voltage = 12;
  chassis.turn_to_angle(-340);
  chassis.drive_distance(20);
  chassis.turn_to_angle(-180);
  chassis.drive_distance(25);
}

void autonREDP(){
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(180);
  chassis.turn_to_angle(270);
  chassis.turn_to_angle(360);
}

void Skills_autonomous(){
  // chassis.drive_max_voltage = 12;
  // chassis.drive_distance(5);
}