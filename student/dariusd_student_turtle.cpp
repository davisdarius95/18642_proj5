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
static int x = 11;
static int y = 11;
static int v = 0;
static int temp = 0;
static int visits[23][23] = {0};

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

void incrementVisits(int dir){
    switch((direction)dir){
        case up:
        if (x>0){
            x--;
            visits[x][y]++;
        }else{
            break;
        }
            break;
        case left:
        if (y>0){
            y--;
            visits[x][y]++;
        }else{
            break;
        }
            break;
        case down:
        if (x<22){
            x++;
            visits[x][y]++;
        }else{
            break;
        }
            break;
        case right:
        if (y<22){
            y++;
            visits[x][y]++;
        }else{
            break;
        }
            break;
        default:
            ROS_ERROR("Array incrementation error");
            break;
    }
    v = visits[x][y];
    //return v;
}

int getNumVisits(){
  return v;
}

/*The functions upBump() - rightBump() each change the orientation of the turtle respective to its orientation
  when the funtion was called. Each function changes the orientation so the turtle is using the right-hand rule to 
  solve the maze*/
int upBump(bool bumped){
  switch(s){
    case clearSpace:
       d = right;
       s = checkSpace;
       break;
    case hitWall:
      if(bumped){
        d = left;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
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
  incrementVisits(d);
  return d;
}

int leftBump(bool bumped){
  switch(s){
    case clearSpace:
       d = up;
       s = checkSpace;
       break;
    case hitWall:
       if(bumped) {
        d = down;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
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
  incrementVisits(d);
 return d;
}

int downBump(bool bumped){
  switch(s){
    case clearSpace:
       d = left;
       s = checkSpace;
       break;
    case hitWall:
      if (bumped) {
        d = right;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
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
  incrementVisits(d);
 return d;
}

int rightBump(bool bumped){
  switch(s){
    case clearSpace:
       d = down;
       s = checkSpace;
       break;
    case hitWall:
      if(bumped) {
        d = up;
        s = hitWall;
      }else{
        s = clearSpace;
      }
      break;
    case checkSpace:
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
 incrementVisits(d);
 return d;
}
