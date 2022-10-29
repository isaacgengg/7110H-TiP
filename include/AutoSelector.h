#include "vex.h"

int autonToRun = 0;

class Button
{
  public:
    int x, y, width, height;
    std::string text;
    vex::color buttonColor, textColor;
    
    Button(int x, int y, int width, int height, std::string text, vex::color buttonColor, vex::color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render()
    {
      Brain.Screen.drawRectangle(x, y, width, height, buttonColor);
      Brain.Screen.printAt(x + 45, y + 25, false, text.c_str());
    }

    bool isClicked()
    {
      if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= x && Brain.Screen.xPosition() <= x + width &&
      Brain.Screen.yPosition() >= y && Brain.Screen.yPosition() <= y + height) {
      return true;}
      else return false;
    }
};

Button autonButtons[] = {
  Button(50, 50, 100, 50, "1", vex::white, vex::black),
  Button(200, 50, 100, 50, "2", vex::white, vex::black),
  Button(350, 50, 100, 50, "3", vex::white, vex::black),
  Button(50, 110, 100, 50, "4", vex::white, vex::black),
  Button(200, 110, 100, 50, "5", vex::white, vex::black),
  Button(350, 110, 100, 50, "6", vex::white, vex::black),
  Button(50, 170, 100, 50, "7", vex::white, vex::black),
  Button(200, 170, 100, 50, "8", vex::white, vex::black),
  Button(350, 170, 100, 50, "9", vex::white, vex::black),
};