// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {
  private final PWM blinkin = new PWM(LightConstants.lightPwm);
  private final Map<String, Integer> lightCodeMap = new HashMap<String, Integer>();
  private final Map<Integer, String> priorityToLight = new HashMap<Integer, String>();
  private PriorityQueue<Integer> lightPriorities = new PriorityQueue<>();
  private Timer timer = new Timer();

  /** Creates a new Lights. */
  public Lights() {
    timer.start();
    lightCodeMap.put("5V", 2125);
    lightCodeMap.put("Hot Pink", 1785);
    lightCodeMap.put("Dark Red", 1795);
    lightCodeMap.put("Red", 1805);
    lightCodeMap.put("Red Orange", 1815);
    lightCodeMap.put("Orange", 1825);
    lightCodeMap.put("Gold", 1835);
    lightCodeMap.put("Yellow", 1845);
    lightCodeMap.put("Lawn Green", 1855);
    lightCodeMap.put("Lime", 1865);
    lightCodeMap.put("Dark Green", 1875);
    lightCodeMap.put("Green", 1885);
    lightCodeMap.put("Blue Green", 1895);
    lightCodeMap.put("Aqua", 1905);
    lightCodeMap.put("Sky Blue", 1915);
    lightCodeMap.put("Dark Blue", 1925);
    lightCodeMap.put("Blue", 1935);
    lightCodeMap.put("Blue Violet", 1945);
    lightCodeMap.put("Violet", 1955);
    lightCodeMap.put("White", 1965);
    lightCodeMap.put("Grey", 1975);
    lightCodeMap.put("Dark Grey", 1985);
    lightCodeMap.put("Black", 1995);
    lightCodeMap.put("Strobe Red", 1445);
    lightCodeMap.put("Strobe Blue", 1455);
    lightCodeMap.put("Strobe White", 1475);

    lightCodeMap.put("End To End Blend Color 1 to 2", 1725);
    lightCodeMap.put("Heartbeat White", 1395);

    priorityToLight.put(0, "Strobe White"); // auto fire - flashing purple
    priorityToLight.put(1, "White"); // auto fire aiming - purple
    priorityToLight.put(2, "Yellow"); // manual transfer - flashing pink
    priorityToLight.put(3, "Hot Pink"); // manual spink shooter - pink
    priorityToLight.put(4, "Red"); // hood up - red
    priorityToLight.put(5, "Blue"); // intake spin - blue
    priorityToLight.put(6, "Green"); // hood down - green
    // priorityToLight.put(7, "End To End Blend Color 1 to 2"); // default

    lightPriorities.add(6); // add lowest prioity

    reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!lightPriorities.isEmpty()) {
      set(priorityToLight.get(lightPriorities.peek()));
    }
    SmartDashboard.putNumber("Lights/peak", lightPriorities.peek());

    if(timer.advanceIfElapsed(LightConstants.resetInterval)){
      reset();
    };
  }

  /**
   * resets blinkin to 5V mode
   */
  public void reset(){
    try {
      int currentColor = getAsInteger();
      set("5V");
      wait(5);
      set("Black");
      wait(5);
      set(currentColor);
    }catch(Exception err){
      err.printStackTrace();
    }
  }

  /**
   * sets the blinkin light from double
   * @param lightCode code based on spark speed(-1 to 1)
   */
  public void set(int lightCode){
    blinkin.setPulseTimeMicroseconds(lightCode);
  }

  /**
   * sets the blinkin light from String
   * @param lightCode string key for light code
   */
  public void set(String lightCode){
    set(lightCodeMap.get(lightCode));
  }

  /**
   * get the current light code as a double
   * @return light code as double
   */
  public int getAsInteger(){
    return blinkin.getPulseTimeMicroseconds();
  }

  /**
   * get the current light code as a string 
   * @return light code as string or empty string if undifined code
  */
  public String getAString(){
    for(Entry<String, Integer> entry : lightCodeMap.entrySet()){
      if(entry.getValue().equals(getAsInteger())) return entry.getKey();
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
    if (priority == 6) return;
    lightPriorities.remove(priority);
  }
}
