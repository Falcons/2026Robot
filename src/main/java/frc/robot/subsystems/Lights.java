// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {
  private final Spark blinkin = new Spark(LightConstants.lightPwm);
  private final Map<String, Double> lightCodeMap = new HashMap<String, Double>();
  private final Map<Integer, String> priorityToLight = new HashMap<Integer, String>();
  private PriorityQueue<Integer> lightPriorities = new PriorityQueue<>();

  /** Creates a new Lights. */
  public Lights() {
    lightCodeMap.put("Hot Pink", 0.57);
    lightCodeMap.put("Dark Red", 0.59);
    lightCodeMap.put("Red", 0.61);
    lightCodeMap.put("Red Orange", 0.63);
    lightCodeMap.put("Orange", 0.65);
    lightCodeMap.put("Gold", 0.67);
    lightCodeMap.put("Yellow", 0.69);
    lightCodeMap.put("Lawn Green", 0.71);
    lightCodeMap.put("Lime", 0.73);
    lightCodeMap.put("Dark Green", 0.75);
    lightCodeMap.put("Green", 0.77);
    lightCodeMap.put("Blue Green", 0.79);
    lightCodeMap.put("Aqua", 0.81);
    lightCodeMap.put("Sky Blue", 0.83);
    lightCodeMap.put("Dark Blue", 0.85);
    lightCodeMap.put("Blue", 0.87);
    lightCodeMap.put("Blue Violet", 0.89);
    lightCodeMap.put("Violet", 0.91);
    lightCodeMap.put("White", 0.93);
    lightCodeMap.put("Grey", 0.95);
    lightCodeMap.put("Dark Grey", 0.97);
    lightCodeMap.put("Black", 0.99);
    lightCodeMap.put("Strobe Red", -0.11);
    lightCodeMap.put("Strobe Blue", -0.15);
    lightCodeMap.put("End To End Blend Color 1 to 2", 0.45);
    lightCodeMap.put("Heartbeat White", -0.21);

    priorityToLight.put(0, "Heartbeat White"); // auto fire - flashing purple
    priorityToLight.put(1, "White"); // auto fire aiming - purple
    priorityToLight.put(2, "Strobe Red"); // manual transfer - flashing pink
    priorityToLight.put(3, "Hot Pink"); // manual spink shooter - pink
    priorityToLight.put(4, "Red"); // hood up - red
    priorityToLight.put(5, "Blue"); // intake spin - blue
    priorityToLight.put(6, "Green"); // hood up - green
    priorityToLight.put(7, "End To End Blend Color 1 to 2"); // default

    lightPriorities.add(7); // add lowest prioity
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!lightPriorities.isEmpty()) {
      set(priorityToLight.get(lightPriorities.peek()));
    }
    SmartDashboard.putNumber("Lights/peak", lightPriorities.peek());
  }

  /**
   * sets the blinkin light from double
   * @param lightCode code based on spark speed(-1 to 1)
   */
  public void set(double lightCode){
    blinkin.set(lightCode);
  }

  /**
   * sets the blinkin light from String
   * @param lightCode string key for light code
   */
  public void set(String lightCode){
    blinkin.set(lightCodeMap.get(lightCode));
  }

  /**
   * get the current light code as a double
   * @return light code as double
   */
  public double getAsDouble(){
    return blinkin.get();
  }

  /**
   * get the current light code as a string 
   * @return light code as string or empty string if undifined code
  */
  public String getAString(){
    for(Entry<String, Double> entry : lightCodeMap.entrySet()){
      if(entry.getValue().equals(blinkin.get())) return entry.getKey();
    }
    return "";
  }

  /**
   * add a queue to display this light
   * @param priority lower is higher priotty
   */
  public void addQueue(int priority) {
    if (!lightPriorities.contains(priority)) {
      lightPriorities.add(priority);
    }
  }
  /**
   * remove that queue to display this light cant remove 
   * the number 7 since that is the default queue
   * @param priority the number of prioty to remove
   */
  public void removeQueue(int priority) {
    if (priority == 7) return;
    lightPriorities.remove(priority);
  }
}
