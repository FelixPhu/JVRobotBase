package org.sciborgs1155.robot;
import org.sciborgs1155.lib.CommandRobot;

public class Robot extends CommandRobot {
  public Robot() {
    super(0.02);
  }
}

Drive drive = new Drive();

private void configureBindings() {
	drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));
}