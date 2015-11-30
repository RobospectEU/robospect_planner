#ifndef PATH_H
#define PATH_H

#include <string.h>
#include <vector>
#include <queue>
#include <stdint.h>
#include <math.h>
#include <cstdlib>

#include <robospect_planner/Component.h>  //enum with ReturnValue for methods
#include <geometry_msgs/Pose2D.h>

#define MIN_ANGLE_BEZIER   0.261799388	// ángulo (radianes) mínimo entre segmentos de la recta para los que ajustaremos a una curva de BEZIER
#define BEZIER_CONTROL_POINTS  5
#define MAX_SPEED_LVL1	0.5
#define MAX_SPEED_LVL2	0.3

using namespace std;

//! Data structure for a Magnet
typedef struct MagnetStruct{
  //! Id of the magnet
  int iID;
  //! X position
  double dX;
  //! Y position
  double dY;
}MagnetStruct;

//! Data structure for a Waypoint
typedef struct Waypoint{
  //! Id of the magnet
  int iID;
  //! X position
  double dX;
  //! Y position
  double dY;
  //! Orientation
  double dA;
  //! Speed to arrive to the point
  double dSpeed;
}Waypoint;

//!  Class to manage the waypoints and magnets of the current path
class Path{
  
/****** MEMBERS ******/
 public:
  //! Current waypoint
  int iCurrentWaypoint;
  //! Current magnet
  int iCurrentMagnet;
  
 private:
  //! Mutex to control the access
  pthread_mutex_t mutexPath;
  //! Vector to store all the Waypoints
  vector <Waypoint> vPoints;
  //! Vector to store all the magnets
  vector <MagnetStruct> vMagnets;
  //! Flag to control the optimization
  bool bOptimized;

/******* METHODS *****/
 public:
  //! Constructor for a path
  Path();

  //! Destructor
  ~Path();

  /******* WAYPOINTS *****/
  //! Adds a new waypoint
  ReturnValue AddWaypoint(Waypoint point);

  //! Adds a vector of waypoints
  ReturnValue AddWaypoint(vector <Waypoint> po);
  
  //! Creates a new point from the desired position to the first waypoint
  ReturnValue CreateInterpolatedWaypoint(geometry_msgs::Pose2D pose);
			
  //! Returns the size of the vector points
  inline unsigned int Size(){ return vPoints.size();}

  //! Returns the next waypoint
  ReturnValue GetNextWaypoint(Waypoint *wp);

  //! Returns the last waypoint
  ReturnValue BackWaypoint(Waypoint *wp);

  //! Gets the current waypoint
  ReturnValue GetCurrentWaypoint(Waypoint *wp);

  //! Gets selected waypoint
  ReturnValue GetWaypoint(int index, Waypoint *wp);

  //! Gets the current Waypoint in the path
  inline int GetCurrentWaypointIndex(){
    return iCurrentWaypoint;
  }

  //! Sets the current Waypoint to index
  ReturnValue SetCurrentWaypoint(int index);

  //! Increase waypoint's number
  inline void NextWaypoint(){
    pthread_mutex_lock(&mutexPath);
    iCurrentWaypoint++;
    pthread_mutex_unlock(&mutexPath);
  }

  //! Returns the number of waypoints
  inline int NumOfWaypoints(){
    return vPoints.size();
  }

  /******* MAGNETS ******/
  //! Adds a new magnet
  ReturnValue AddMagnet(MagnetStruct magnet);

  //! Adds a vector of magnets
  ReturnValue AddMagnet(vector <MagnetStruct> po);
    
  //! Returns the next magnet
  ReturnValue GetNextMagnet(MagnetStruct *mg);
  
  //! Returns the back magnet
  ReturnValue BackMagnet(MagnetStruct * mg);
  
  //! Gets the current magnet
  ReturnValue GetCurrentMagnet( MagnetStruct * mg );
  
  //! Gets the previous magnet
  ReturnValue GetPreviousMagnet( MagnetStruct * mg );

  //! Gets the current MagnetStruct in the path
  inline int GetCurrentMagnetIndex(){
    return iCurrentMagnet;
  }

  //! Sets the current magnet to index
  ReturnValue SetCurrentMagnet(int index);

  //! Increase magnet's number
  inline void NextMagnet(){
    pthread_mutex_lock(&mutexPath);
    iCurrentMagnet++;
    pthread_mutex_unlock(&mutexPath);
  }

  //! Gets the last magnet
  ReturnValue GetLastMagnet( MagnetStruct * mg );
  
  //! Returns the number of magnets
  inline int NumOfMagnets(){
    return vMagnets.size();
  }

  /****** OTHER METHODS ********/

  //! Obtains the points for a quadratic Bezier's curve
  //! \param cp0 as player_pose2d_t, control point 0
  //!	\param cp1 as player_pose2d_t, control point 1
  //!	\param cp2 as player_pose2d_t, control point 2
  //!	\param t as float, [0 ... 1]
  //!	\return Point over the curve
  Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t);

  //! Function that calculate the distance to deccelerate from target speed
  //! \param target_speed as double, speed on m/s
  //! \return distance on meters
  double DistForSpeed(double target_speed);
  
  //! Modifies and adds new waypoints to the route for improving the path
  //! \param distance as double, used for the calculation of the new points
  //! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
  //! \return OK
  ReturnValue Optimize(double distance);

  /****** SUPPORT ********/
  //! Overloaded operator +=
  inline Path &operator+=(const Path &a){
    AddWaypoint(a.vPoints);
    AddMagnet(a.vMagnets);
    return *this;
  }

  //! Cross product
  inline double dot2( Waypoint w, Waypoint v) {
    return (w.dX*v.dX + w.dY*v.dY);
  }
  
  //! Clears the waypoints and magnets
  void Clear();
  
  //! Prints all the waypoints
  void Print();

};

#endif
