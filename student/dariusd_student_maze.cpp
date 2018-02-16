/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:Darius Davis
 * ANDREW ID: dariusd
 * LAST UPDATE: 2/9/18
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 */

//m1.maze file format:
//    first line is start square
//    second line is end square
//    remaining lines are maze walls

// external helper functions.  Don't change these lines!

// OK to change below this point
#include "student.h"
#include "dariusd_student_turtle.h"

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

typedef struct coordinates {
    uint32_t xa = 0;
    uint32_t ya = 0;
    uint32_t xb = 0;
    uint32_t yb = 0;
}coo;

typedef struct miscellaneous {
    bool stepAllowed;
    bool finished;
    bool moving;
    bool bumped;
    int countdown;
    const int TIMEOUT = 17; // bigger number slows down simulation so you can see what's happening
    int numVisits = 0;
}misc;

static direction d;
static state s;
static coo place;
static misc m;

// this procedure takes one step in the maze and returns   true=done / false=not done

bool moveTurtle(QPointF & pos_) {

    m.moving = true;

    if (m.countdown == 0) {
      place.xa = pos_.x();
      place.ya = pos_.y();
      place.xb = pos_.x();
      place.yb = pos_.y();
      
      switch (d) {
      case right:
           place.xb += 1;
           break;
      case up:
           place.yb += 1;
           break;
      case left:
           place.xb += 1;
           place.yb += 1;
           place.ya += 1;
           break;
      case down:
           place.xb += 1;
           place.yb += 1;
           place.xa += 1;
           break;
      default:
           break;
           ROS_ERROR("Movement error");
      }

        m.bumped = bumped(place.xa, place.ya, place.xb, place.yb);
        resultedInBump(m.bumped);
        m.finished = atend(pos_.x(), pos_.y());

        d = (direction)turtleAction(d, s);
        s = (state)getState();
        m.stepAllowed = (s == 2);
        m.moving = true;


        /*If there is a clear space and the turtle is not finished, this function moves
          it forward one step.*/
        if (m.stepAllowed == true && m.finished == false) {
            
            switch (d) {
            case left:
                 pos_.setY(pos_.y() + 1);
		 m.numVisits = getNumVisits();
     		 displayTurtle(d, m.numVisits);
                 break;
            case down:
                 pos_.setX(pos_.x() + 1);
		 m.numVisits = getNumVisits();
     		 displayTurtle(d, m.numVisits);
                 break;
            case right:
                 pos_.setY(pos_.y() - 1);
		 m.numVisits = getNumVisits();
     		 displayTurtle(d, m.numVisits);
                 break;
            case up:
                 pos_.setX(pos_.x() - 1);
		 m.numVisits = getNumVisits();
     		 displayTurtle(d, m.numVisits);
                 break;
            default:
                 ROS_ERROR("stepAllowed & finished: step error");
                 break;
            }
            m.stepAllowed = true;
            m.moving = false;
        }
    }

    if(m.countdown == 0){
      m.countdown = m.TIMEOUT;
    }else{
      m.countdown -= 1;
    }
    // display the turtle -- must call this function before returning!
    displayTurtle(d, m.numVisits);
    return (m.finished);
}


