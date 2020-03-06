package  frc.team8051.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.team8051.Robot;

public class ColorSensor extends Subsystem {

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
    // private static ColorMatch m_colorMatcher = new ColorMatch();
    // private static Color kBlueTarget= ColorMatch.makeColor(0.143, 0.427, 0.429);
    // private static Color kGreenTarget= ColorMatch.makeColor(0.197, 0.561, 0.240);
    // private static Color kRedTarget= ColorMatch.makeColor(0.561, 0.232, 0.114);
    // private static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    // private final I2C.Port i2cPort = I2C.Port.kOnboard;
    // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    // private int numberOfTimesSpun = 0;
    // private int numberOfBlues = 0;
    // private int numberOfReds = 0;
    // private int numberOfYellows = 0;
    // private int numberOfGreens = 0;
    // private String PreviousColor = "";
    // private int ColorCounter = -1;
    // private String currentColor = "";
    // private boolean isPressed = false;
    // @Override
    // protected void initDefaultCommand() {

    // }


    // public static void initializeColor(){
    //     m_colorMatcher.addColorMatch(kBlueTarget);
    //     m_colorMatcher.addColorMatch(kGreenTarget);
    //     m_colorMatcher.addColorMatch(kRedTarget);
    //     m_colorMatcher.addColorMatch(kYellowTarget);
    // }
    // public void setToZero(){
    //     numberOfTimesSpun = 0;
    //     numberOfBlues = 0;
    //     numberOfReds = 0;
    //     numberOfYellows = 0;
    //     numberOfGreens = 0;
    // }

    // public void periodic(){
    //     if(Robot.getInstance().getOI().joystick.getRawButton(1)){
    //               isPressed = true;
    //          }
    //     if(Robot.getInstance().getOI().joystick.getRawButton(2)){
    //         isPressed = false;
    //     }
    //     if(isPressed == true){
    //         SenseColor();
    //     }
    // }

    // public void SenseColor() {

    //     Color detectedColor = m_colorSensor.getColor();
    //     String colorString;
    //     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    //     if (match.color == kBlueTarget) {
    //         colorString = "Blue";
    //         if(ColorCounter != -1){
    //             PreviousColor = currentColor;
    //         }
    //         if(ColorCounter == -1){
    //             ColorCounter +=1;
    //         }
    //         currentColor = "Blue";
    //         if(currentColor != PreviousColor){
    //             currentColor = "Blue";
    //             numberOfBlues +=1;
    //         }
    //     }
    //     else if (match.color == kRedTarget) {
    //         colorString = "Red";
    //         if(ColorCounter != -1){
    //             PreviousColor = currentColor;
    //         }
    //         if(ColorCounter == -1){
    //             ColorCounter +=1;
    //         }
    //         currentColor = "Red";
    //         if(currentColor != PreviousColor){
    //             currentColor = "Red";
    //             numberOfReds +=1;
    //         }
    //     }
    //     else if (match.color == kGreenTarget) {
    //         colorString = "Green";
    //         if(ColorCounter != -1){
    //             PreviousColor = currentColor;
    //         }
    //         if(ColorCounter == -1){
    //             ColorCounter +=1;
    //         }
    //         currentColor = "Green";
    //         if(currentColor != PreviousColor){
    //             currentColor = "Green";
    //             numberOfGreens +=1;
    //         }
    //     } else if (match.color == kYellowTarget) {
    //         colorString = "Yellow";
    //         if(ColorCounter != -1){
    //             PreviousColor = currentColor;
    //         }
    //         if(ColorCounter == -1){
    //             ColorCounter +=1;
    //         }
    //         currentColor = "Yellow";
    //         if(currentColor != PreviousColor){
    //             currentColor = "Yellow";
    //             numberOfYellows +=1;
    //         }
    //     } else {
    //         colorString = "Unknown";
    //     }
    //     SmartDashboard.putNumber("Red", detectedColor.red);
    //     SmartDashboard.putNumber("Green", detectedColor.green);
    //     SmartDashboard.putNumber("Blue", detectedColor.blue);
    //     SmartDashboard.putNumber("Confidence", match.confidence);
    //     SmartDashboard.putString("Detected Color", colorString);
    //     SmartDashboard.putString("Current Color", currentColor);
    // //    SmartDashboard.putString("Previous Color", PreviousColor);
    //     SmartDashboard.putNumber("Blues", numberOfBlues);
    //     SmartDashboard.putNumber("Reds", numberOfReds);
    //     SmartDashboard.putNumber("Greens", numberOfGreens);
    //     SmartDashboard.putNumber("Yellows", numberOfYellows);
    //     SmartDashboard.putNumber("Spins", numberOfTimesSpun);
    //     if (numberOfBlues + numberOfGreens + numberOfReds + numberOfYellows == 9) {
    //         numberOfTimesSpun += 1;
    //         numberOfYellows = 0;
    //         numberOfReds = 0;
    //         numberOfBlues = 0;
    //         numberOfGreens = 0;
    //         System.out.println("Spins: " + numberOfTimesSpun);
    //     }
    // }
}
/*package frc.team8051.subsystems;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
 public class ColorSensor extends Subsystem {
     private static ColorMatch m_colorMatcher = new ColorMatch();
     private static Color kBlueTarget= ColorMatch.makeColor(0.143, 0.427, 0.429);
     private static Color kGreenTarget= ColorMatch.makeColor(0.197, 0.561, 0.240);
     private static Color kRedTarget= ColorMatch.makeColor(0.561, 0.232, 0.114);
     private static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
     private final I2C.Port i2cPort = I2C.Port.kOnboard;
     private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
     private int numberOfTimesSpun = 0;
     private Integer numberOfBlues = 0;
     private Integer numberOfReds = 0;
     private Integer numberOfYellows = 0;
     private Integer numberOfGreens = 0;
     private String PreviousColor = "";
     private int ColorCounter = -1;
     private String currentColor = "";
     String colorString;
     @Override
     protected void initDefaultCommand() {

     }


     public static void initializeColor(){
         m_colorMatcher.addColorMatch(kBlueTarget);
         m_colorMatcher.addColorMatch(kGreenTarget);
         m_colorMatcher.addColorMatch(kRedTarget);
         m_colorMatcher.addColorMatch(kYellowTarget);
     }
     public void setToZero(){
         numberOfTimesSpun = 0;
         numberOfBlues = 0;
         numberOfReds = 0;
         numberOfYellows = 0;
         numberOfGreens = 0;
     }
     public  void colorAdder(String color, Integer colorCounted){
         colorString = color;
                 if(ColorCounter != -1){
                 PreviousColor = currentColor;
                 }
                 if(ColorCounter == -1){
                 ColorCounter +=1;
                 }
                 currentColor = color;
                 if(currentColor != PreviousColor){
                 currentColor = color;
                 colorCounted +=1 ;
                 }
                 }

public void SenseColor() {

        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
        colorAdder("Blue",numberOfBlues);
        }
        else if (match.color == kRedTarget) {
        colorAdder("Red",numberOfReds);
        }
        else if (match.color == kGreenTarget) {
        colorAdder("Green",numberOfGreens);
        } else if (match.color == kYellowTarget) {
        colorAdder("Yellow",numberOfYellows);
        } else {
        colorString = "Unknown";
        }
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        SmartDashboard.putString("Current Color", currentColor);
        //    SmartDashboard.putString("Previous Color", PreviousColor);
        SmartDashboard.putNumber("Blues", numberOfBlues);
        SmartDashboard.putNumber("Reds", numberOfReds);
        SmartDashboard.putNumber("Greens", numberOfGreens);
        SmartDashboard.putNumber("Yellows", numberOfYellows);
        SmartDashboard.putNumber("Spins", numberOfTimesSpun);
        if (numberOfBlues + numberOfGreens + numberOfReds + numberOfYellows == 9) {
        numberOfTimesSpun += 1;
        numberOfYellows = 0;
        numberOfReds = 0;
        numberOfBlues = 0;
        numberOfGreens = 0;
        System.out.println("Spins: " + numberOfTimesSpun);
        }
        }
        }
        */



