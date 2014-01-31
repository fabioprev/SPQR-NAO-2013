#include "BHToolBar.h"
#include "ConsoleRoboCupCtrl.h"
#include "Platform/SystemCall.h"
#include <QTimer>
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/KeyStates.h"

QMenu* BHToolBar::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("B-Human"));
  QAction* standAct = new QAction(QIcon(":/Icons/stand.png"), tr("&MotionRequest stand"), menu);
  QAction* sitDownAct = new QAction(QIcon(":/Icons/sitDown.png"), tr("&MotionRequest sitDown"), menu);
  QAction* pressChestButtonAct = new QAction(QIcon(":/Icons/chestButton.png"), tr("&Press and Release Chest Button"), menu);
  connect(standAct  , SIGNAL(triggered()), this, SLOT(stand()));
  connect(sitDownAct, SIGNAL(triggered()), this, SLOT(sitDown()));
  connect(pressChestButtonAct, SIGNAL(triggered()), this, SLOT(pressChestButton()));
  menu->addAction(standAct);
  menu->addAction(sitDownAct);
  menu->addAction(pressChestButtonAct);
  return menu;
}

void BHToolBar::stand()
{
  setPlayDead();
  QTimer::singleShot(300, this, SLOT(setStand()));
}

void BHToolBar::sitDown()
{
  setSitDown();
  QTimer::singleShot(4000, this, SLOT(setPlayDead()));
}

void BHToolBar::setPlayDead()
{

  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::playDead;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::setSitDown()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::sitDown;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::setStand()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::stand;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::playDead;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::pressChestButton()
{
	KeyStates keyState;
	keyState.pressed[KeyStates::Key::chest] = true;

	console.setRepresentation("KeyStates", keyState);

  QTimer::singleShot(80, this, SLOT(releaseChestButton()));
}

void BHToolBar::releaseChestButton()
{
	KeyStates keyState;
	keyState.pressed[KeyStates::Key::chest] = false;

	console.setRepresentation("KeyStates", keyState);

  QTimer::singleShot(5, this, SLOT(unchangeButtons()));
}

void BHToolBar::unchangeButtons()
{
  console.executeConsoleCommand("set representation:KeyStates unchanged");
}