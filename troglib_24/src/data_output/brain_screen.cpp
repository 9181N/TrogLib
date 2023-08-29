#include "robot_config.h"
#include "sensor_data.h"
#include <iostream>
#include <string>
#include "vex.h"
using namespace vex;

float logo_timer = 0;
float screen_refresh_time = 50;

void print_to_brain()
{
  // Refresh Screen
  // left side
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setFont(prop30);
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("X: %.2f", bot.x);
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("Y: %.2f", bot.y);
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("H: %.2f", bot.h_deg);
  // right side
  Brain.Screen.setCursor(7, 26);
  Brain.Screen.print("TX: %3.2f", bot.perpindicular_inch);
  Brain.Screen.setCursor(8, 26);
  Brain.Screen.print("TY: %3.2f", bot.parallel_inch);

  Brain.Screen.setCursor(1, 26);
  Brain.Screen.print("Hue:%3.2f", 1.0);
}

void controller_screen() {
    Controller.Screen.clearLine(1);
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("X%5.1f Y%5.1f H%5.1f", bot.x, bot.y, bot.h_deg);
}

void refresh_screen()
{
  Brain.Screen.setPenColor(black);
  if (1)
  {
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(0, 0, 120, 240);
    Brain.Screen.drawRectangle(360, 0, 120, 240);

    Brain.Screen.setFillColor(green);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawCircle(60, 60, 40);
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFillColor(black);
  }
  else
  {
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(0, 0, 120, 240);
    Brain.Screen.drawRectangle(360, 0, 120, 240);
  }
}

void refresh_logo()
{
  if (logo_timer >= 2000)
  {
    logo_timer = 0;
    Brain.Screen.drawImageFromFile("240x240 TNTN logo.png", 120, 0);
  }
  logo_timer += screen_refresh_time;
}

int brain_readout_thread()
{
  Brain.Screen.drawImageFromFile("240x240 TNTN logo.png", 120, 0);

  while (1)
  {
    refresh_screen();
    refresh_logo();
    print_to_brain();
    controller_screen();
    wait(screen_refresh_time, msec);
  }
  return 0;
}