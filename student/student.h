#include <ros/ros.h>
#include <QPointF>

// external helper functions.  Don't change these lines!
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayTurtle(int nw_or);
void displayTurtle(int nw_or, int visits);

bool moveTurtle(QPointF& pos_);
