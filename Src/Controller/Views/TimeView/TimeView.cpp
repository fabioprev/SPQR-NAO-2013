/**
* @file Controller/Views/TimeView.cpp
*
* Implementation of class TimeView
*
* @author Colin Graf
* @author Arne Böckmann
*/

#include <QString>
#include "TimeView.h"
#include "Platform/Thread.h"
#include "Controller/RobotConsole.h"
#include "TimeWidget.h"

TimeView::TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), info(info) {}

SimRobot::Widget* TimeView::createWidget()
{
  return new TimeWidget(*this);
}
