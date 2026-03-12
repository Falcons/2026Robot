// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {
  private final Spark blinkin = new Spark(LightConstants.lightPwm);
  private final Map<String, Double> lightCodeMap = new HashMap<String, Double>();

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
