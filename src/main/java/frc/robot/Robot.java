/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class Robot extends TimedRobot {
 
  // Controllers 
  XboxController driver;
  XboxController mech;

  //Motors 
  //Drive
  TalonSRX leftDrive1;
  VictorSPX leftDrive2;
  TalonSRX rightDrive1;
  VictorSPX rightDrive2;

  //ARM
  TalonSRX armRight;
  VictorSPX armLeft;

  //Elevator 
  TalonSRX elevator; 

  //Cargo intake 
  VictorSP cargoIntake; 
  
  // Solenoids 
  //Climb
  Solenoid climbUp;
  Solenoid climbDown; 

  // Hatch Extender
  Solenoid hatchExtendOut;
  Solenoid hatchExtendIn; 

  // Hatch Punchers
  Solenoid hatchPunchTopOut;
  Solenoid hatchPunchTopIn;

  Solenoid hatchPunchBottomOut;
  Solenoid hatchPunchBottomIn; 

  // Box 
  Solenoid boxClose;
  Solenoid boxOpen; 

  //Constants 
  //Elevator PID
  double elevatorP = 0;
  double elevatorI = 0;
  double elevatorD = 0;
  double elevatorF = 0;

  // Elevator Heights 
  double elevatorTop = 0;
  double elevatorBottom = -200; // copied from old code 
  double elevatorClimb = -300; 

  // ARM PID 
  double armP = 0;
  double armI = 0; 
  double armD = 0;
  double armF = 0;

  //Arm Heights 
  double armTop = -100;
  double armBottom = -200;
  double armClimb = -300;

  //Punch timer code 
  long timeStartedPunching; 
  long timeToPunch = 2000; // 2000ms or 2 seconds 
  long timeSincePunched; 
  boolean startedPunching = false;



  @Override
  public void robotInit() {

    //initialize everything to the correct ports 
    this.driver = new XboxController(0);
    this.mech = new XboxController(1);

    // CAN Stuff 
    this.leftDrive1 = new TalonSRX(1);
    this.leftDrive2 = new VictorSPX(2);
    this.rightDrive1 = new TalonSRX(3);
    this.rightDrive2 = new VictorSPX(4);

    this.armRight = new TalonSRX(5);
    this.armLeft = new VictorSPX(6);

    this.elevator = new TalonSRX(7);

    // RIO PWM Stuff
    this.cargoIntake = new VictorSP(0); // this is okay because its pwm 

    // Solenoid stuff 
    this.climbDown = new Solenoid(1, 0); // in PCM 1 port 0 
    this.climbUp = new Solenoid(1, 1); // PCM 1 port 1

    this.hatchPunchTopIn = new Solenoid(0, 4); // PCM 0 port 4
    this.hatchPunchTopOut = new Solenoid(0, 5); // PCM 0 port 5 

    this.hatchExtendIn = new Solenoid(0, 6); // PCM 0 port 6
    this.hatchExtendOut = new Solenoid(0, 7); // PCM 0 port 7

    this.boxClose = new Solenoid(1, 4); // PCM 1 port 4
    this.boxOpen = new Solenoid(1,5); // PCM 1 port 5

    this.hatchPunchBottomIn = new Solenoid(1,6); // PCM 1 port 6
    this.hatchPunchBottomOut = new Solenoid(1, 7); // PCM 1 port 7


  }

  public void configureSpeedControllers(){ 
    //Set the drive victors to follow their talons 
    this.leftDrive2.follow(this.leftDrive1);
    this.rightDrive2.follow(this.rightDrive1);

    //configure the elevators pid constants 
    this.elevator.selectProfileSlot(0, 0);
    this.elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,20);
    this.elevator.setSensorPhase(true); // make false if encoder is backwards
    this.elevator.config_kP(0, this.elevatorP);
    this.elevator.config_kI(0, this.elevatorI);
    this.elevator.config_kD(0, this.elevatorD);
    this.elevator.config_kF(0, this.elevatorF);
    this.elevator.setSelectedSensorPosition(0); // reset the encoder 

    //configure the arms pid constants 
    this.armRight.selectProfileSlot(0, 0);
    this.armRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,20);
    this.armRight.setSensorPhase(true); // make false if encoder is backwards
    this.armRight.config_kP(0, this.armP);
    this.armRight.config_kI(0, this.armI);
    this.armRight.config_kD(0, this.armD);
    this.armRight.config_kF(0, this.armF);
    this.armRight.setSelectedSensorPosition(0); // reset the encoder

  }

  


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() { // Main teleop loop 

    //Calculate Drive Output 
    double x = this.driver.getX(Hand.kRight);
    double y = this.driver.getY(Hand.kLeft);

    this.setDrive(y + x, y - x);

    // Cargo intake code
    double output = this.driver.getTriggerAxis(Hand.kRight);
    if(output > 0.2){
      this.cargoIntake.set(1.0); // change to output for variable speed 
    } else {
      this.cargoIntake.set(0);
    }


    // Hatch intake code 

    //extender
    if(this.mech.getBButton()){ // only extend when b is pressed 
      this.setHatchExtender(true);
    } else {
      this.setHatchExtender(false);
    }

    //box 
    if(this.mech.getAButton()){ // toggle closed 
      this.setBox(true);
    } else if(this.mech.getYButton()){ // toggle to open 
      this.setBox(false);
    }

    //Puncher 
    if(this.mech.getTriggerAxis(Hand.kLeft) > 0.2){ // we want to punch 
      if(!this.startedPunching){ // only do this 1 time 
        this.timeStartedPunching = System.currentTimeMillis();
        this.startedPunching = true; // this means we only do it the first time we pressed the trigger and not as we keep holding it
      } else {
        this.timeSincePunched = System.currentTimeMillis() - this.timeStartedPunching;
        if(this.timeSincePunched < this.timeToPunch){ // keep punching 
          this.setPunchTop(true);
          this.setPunchBottom(true);
        } else {
          this.setPunchTop(false);
          this.setPunchBottom(false);
        }
      }
    } else {
      this.startedPunching = false; // reset for next time we want to punch 
      this.setPunchTop(false);
      this.setPunchBottom(false);
    }

    // Elevator code 
    System.out.println(this.getElevatorPosition());
    
    //Basic manual control 
		if (this.mech.getPOV() == 0){
		  this.setElevatorManual(0.1);
		} else if (this.mech.getPOV() == 180){
      this.setElevatorManual(-0.1);
		} else{
			this.setElevatorManual(0.0);
    }


    // Arm code 
    // sets pid target based on which button is pressed
    System.out.println(this.getArmPosition());
		if (driver.getBumper(Hand.kRight)) {
      this.setArmPID(this.armTop);
		} else if (driver.getBumper(Hand.kLeft)) {
      this.setArmPID(this.armBottom);
		} else if (driver.getPOV() == 180) { 
      this.setArmPID(this.armClimb);
		} else {
		  this.setArmManual(0.0);
    }

    //Climber code 
    if(this.mech.getStartButton()){
      this.setClimber(true);
    } else {
      this.setClimber(false);
    }



  }

  @Override
  public void disabledInit(){

  }

  @Override
  public void disabledPeriodic(){

  }




  //Functions to make our lives easier 

  // Drive
  public void setDrive(double leftOut, double rightOut){
    this.leftDrive1.set(ControlMode.PercentOutput, leftOut);
    this.rightDrive1.set(ControlMode.PercentOutput, rightOut);
  }

  //Elevator 
  public void setElevatorManual(double output){
    this.elevator.set(ControlMode.PercentOutput, output); // not using PID
  }

  public void setElevatorPID(double targetValue){ // will use pid to get to the target
    this.elevator.set(ControlMode.Position, targetValue);
  }

  public double getElevatorPosition(){ // returns the elevator height in ticks
    return this.elevator.getSelectedSensorPosition();
  }

  // Arm 
  public void setArmManual(double output){
    this.armRight.set(ControlMode.PercentOutput, output); // not using PID
  }

  public void setArmPID(double targetValue){ // will use pid to get to the target
    this.armRight.set(ControlMode.Position, targetValue);
  }

  public double getArmPosition(){ // returns the arm height in ticks
    return this.armRight.getSelectedSensorPosition();
  }


  public void setPunchTop(boolean out){ // true for out, false for in
    this.hatchPunchTopIn.set(!out);
    this.hatchPunchTopOut.set(out);
  }

  public void setPunchBottom(boolean out){ // true for out, false for in
    this.hatchPunchBottomIn.set(!out);
    this.hatchPunchBottomOut.set(out);
  }

  public void setHatchExtender(boolean out){
    this.hatchExtendIn.set(!out);
    this.hatchExtendOut.set(out);
  }

  public void setBox(boolean closed){
    this.boxClose.set(closed);
    this.boxOpen.set(!closed);
  }

  public void setClimber(boolean up){
    this.climbUp.set(up);
    this.climbDown.set(!up);
  }


 

}
