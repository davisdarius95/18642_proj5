/*
 * Original author: Milda Zizyte
 *
 * The functions in this file are called by student_maze.cpp
 *
 * This file shall encompass the logic of the maze solving algorithm,
 * from the point of the view of the turtle.
 *
 * (You may rename the functions or introduce new ones.
 * They are here to get you started.)
 */


#include "student.h"
#include "dariusd_student_turtle.h"

int upBump(bool bumped);
int leftBump(bool bumped);
int downBump(bool bumped);
int rightBump(bool bumped);

typedef enum {
    up,
    right,
    down,
    left,
}direction;

typedef enum {
    hitWall, 
    checkSpace,
    clearSpace,
}state;

static bool b;

static direction d;
static state s;

/*
 * Function called by student_maze.cpp to return a turtle action
 * 
 * Cannot use the absolute coordinates of the turtle in the maze
 * Returns an action to perform, such as: turn left, turn right, move forward
 * (replace "void" return value with type of your choosing
 */

int turtleAction(int dir, int st) {
	s = (state)st;
	switch ((direction)dir) {
        case up:
          d = (direction)upBump(b);
          break;

        case left:
          d = (direction)leftBump(b);
          break;

        case down:
          d = (direction)downBump(b);
          break;

        case right:
          d = (direction)rightBump(b);
          break;

        default:
          ROS_ERROR("Directional error");
          break;
        }
    return d;
}

int getState(){
    return s;
}

/* Function called by student_maze.cpp whenever student_maze has detected a bump
 *
 * Should update some internal state in your code
 */

void resultedInBump(bool bump) {
	b = bump;
}

/*The functions upBump() - rightBump() each change the orientation of the turtle respective to its orientation
  when the funtion was called. Each function changes the orientation so the turtle is using the right-hand rule to 
  solve the maze*/
int upBump(bool bumped){
  switch(s){
    case clearSpace:
       ROS_INFO("UB, state: clearSpace, Direction: right");
       d = right;
       s = checkSpace;
       break;
    case hitWall:
      ROS_INFO("UB, state: hitWall, Direction: left");
      if(bumped){
        d = left;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
      ROS_INFO("UB, state: checkSpace, Direction: left");
      if(bumped){
        d = left;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    default:
         ROS_ERROR("Up case: undefined state");
         break;
  }
  return d;
}

int leftBump(bool bumped){
  switch(s){
    case clearSpace:
       ROS_INFO("LB, state: clearSpace, Direction: up");
       d = up;
       s = checkSpace;
       break;
    case hitWall:
       ROS_INFO("LB, state: hitWall, Direction: down");
       if(bumped) {
        d = down;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
       ROS_INFO("LB, state: checkSpace, Direction: down");
       if(bumped) {
        d = down;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    default:
       ROS_ERROR("Left case: undefined state");
       break;
  }
 return d;
}

int downBump(bool bumped){
  switch(s){
    case clearSpace:
       ROS_INFO("DB, state: clearSpace, Direction: left");
       d = left;
       s = checkSpace;
       break;
    case hitWall:
      ROS_INFO("DB, state: hitWall, Direction: right");
      if (bumped) {
        d = right;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
       ROS_INFO("DB, state: checkSpace, Direction: right");
       if (bumped) {
        d = right;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    default:
       ROS_ERROR("Down case: undefined state");
       break;
  }
 return d;
}

int rightBump(bool bumped){
  switch(s){
    case clearSpace:
       ROS_INFO("RB, state: clearSpace, Direction: down");
       d = down;
       s = checkSpace;
       break;
    case hitWall:
      ROS_INFO("RB, state: hitWall, Direction: up");
      if(bumped) {
        d = up;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
       ROS_INFO("RB, state: checkSpace, Direction: up");
       if(bumped) {
        d = up;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    default:
       ROS_ERROR("Right case: undefined state");
       break;
  }
 return d;
}
