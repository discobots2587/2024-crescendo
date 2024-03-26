// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.LinearRegression;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_VOLTS_PER_SEC = 0.1;

  private final int numSuppliers;

  private FeedForwardCharacterizationData[] data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> v0Supplier, v1Supplier, v2Supplier, v3Supplier;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public FeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> v0Supplier, Supplier<Double> v1Supplier, Supplier<Double> v2Supplier, Supplier<Double> v3Supplier) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.v0Supplier = v0Supplier;
    this.v1Supplier = v1Supplier;
    this.v2Supplier = v2Supplier;
    this.v3Supplier = v3Supplier;
    numSuppliers = 4;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new FeedForwardCharacterizationData[numSuppliers];
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
      voltageConsumer.accept(voltage);
      switch(numSuppliers){
        case 4:
          data[3].add(v3Supplier.get(), voltage);
        case 3:
          data[2].add(v2Supplier.get(), voltage);
        case 2:
          data[1].add(v1Supplier.get(), voltage);
        case 1:
          data[0].add(v0Supplier.get(), voltage);
      }
      // for(int i = 0; i < numSuppliers; i++){
      //   data[i].add(velocitySupplier.get(), voltage);
      // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    for(int i = 0; i < data.length; i++){
      System.out.println("Supplier " + i + ":");
      data[i].print();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      LinearRegression regression =
          new LinearRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray());

      System.out.println("FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.intercept()));
      System.out.println(String.format("\tkV=%.5f", regression.slope()));
    }
  }
}
