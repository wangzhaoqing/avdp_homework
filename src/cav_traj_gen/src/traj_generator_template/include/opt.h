/*
 * opt.h
 * This is the main class for trajectory optimization
 * We already define some structes and variables that might be useful
 * Do not change the structes, revise other things as your need
 * Your main goal is to finish the constructor function Opt(), ini() and run()
 * **YOU REALLY NEED TO FINISH THIS**
 */
#ifndef OPT_H
#define OPT_H

#include "XM_path.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "ros/ros.h"

#include "spline.h"
#include "dataDefine.hpp"


#define TIME_GAP_PATH (0.25)
#define TIME_GAP_CONTROL (0.04)
#define MAX_ACCURANCY (0.1)

/***********************************************************************/
/*         Definitions of your own data structure can be here          */
/*                      Below are some examples                        */
/***********************************************************************/

class Point_Xd : public X_Point
{
public:
    void ini()
    {
        x = y = z = 0;
        t = heading = v = cr = 0;
        x_d = x_dd = y_d = y_dd = 0;
        bound_left = bound_right = 0;
        beta = gama = slipangle = 0;
        mileage = 0;
    }

    double x_d = 0;
    double x_dd = 0;
    double y_d = 0;
    double y_dd = 0;

    double bound_left = 0;
    double bound_right = 0;

    double beta = 0;
    double gama = 0;

    double slipangle = 0;

    using X_Point::dist2XY;
};


class Trajectory_S
{
    public:
	Trajectory_S()
	{
		Point_Xd pt;
		path.assign(1,pt);
	}

	void ini()
	{
		minDObj = 100, minDbound = 3.0;
		max_vx= 0, min_vx= 0, max_acc_x= 0, min_acc_x= 0, max_acc_y= 0;
		max_cr = max_yaw_rate = 0;
		Jcost = 0;

		feasible = true;
	}

    double final_tf = 0;
    double final_xf = 0;
    double final_yf = 0;

    double cx[6], cy[6];

    // goal input
    double ini_y0 = 0, ini_y0_d = 0, ini_y0_dd = 0;
    double ini_x0_d = 0, ini_x0_dd = 0;

    //the goal
    double ref_tf = 0;
    double ref_yf = 0;
    double ref_xf = 0, ref_xf_d = 0, ref_xf_dd = 0;

    // threshold for detection
    double minDObj = 100, minDbound = 3.0;

    // dynamic limits
    double max_vx = 0, min_vx = 0, max_acc_x = 0, min_acc_x = 0, max_acc_y = 0;
    double max_cr = 0, max_yaw_rate = 0;

    //feasible
    bool feasible = true;
    int num = 0;

    //flag of best path
    bool flag_best = false;


    //cost
    double Jcost = 0;

    std::vector<Point_Xd> path;
};


/*****************************************************************************************/
/*                  This is the main class for trajectory optimization                   */
/*        We already defined some variables might be useful, keep them as your need      */
/*                 And feel free to define your own functions and variables              */
/* Notice: do not change the input PlanningEnv_S *_env and output Trajectory_S traj_best */
/*              You need to finish the constructor function Opt(), ini() and run()       */
/*****************************************************************************************/

/**
 * @brief The main class for optimal trajectory generation.
 * You need to utilize info in `PlanningEnv_S` to generate optimal trajectory `traj_best`
 * @todo Finish the constructor function Opt(), ini() and run().
 * Define your own functions and variables as needed.
 * Basic Math functions are included in `include/include_x`. Try your best to use them.
 */
class Opt
{
public:
    double ego_radius = 1.0;
    double safe_buffer = 0.2;

    // constructor
    Opt()
    {

        sample_params.d_min = -4.0; 
        sample_params.d_max = 4.0;
        sample_params.d_samples = 33; 
        
        sample_params.v_min = 1.0;
        sample_params.v_max = 8.0;
        sample_params.v_samples = 8; 
        

        sample_params.T_min = 3.0; 
        sample_params.T_max = 6.0; 
        sample_params.T_samples = 4;
        
        candidate_trajectories.clear();
        last_planning_time = 0;
    };
    ~Opt(){};
    
    int ini(PlanningEnv_S *_env);
    void run();
    
    /// @brief The optimal planning result. 
    /// This data structure should not be changed. Do remember to fill it after your planning.
    Trajectory_S traj_best;


    std::vector<Trajectory_S> candidate_trajectories;

    /// @brief [Example] The goal point for trajectory optimization.
    /// It will be shown in 3d display. 
    /// If you don't need it, do remember to revise display_3d_mode1.cpp as well.
    Point_Xd pt_goal;
    Point_Xd pt_goal_real;

    //-----------------------------------
    // Settings for your trajectory generation, these are changed with gui (see mainloop.cpp)
    bool flag_checkObstacles = true;
    bool flag_ay_soft_const = true;
    double target_time = 3;
    
    //-----------------------------------
    // Settings for your trajectory generation, these are changed in mainloop.cpp.
    double set_max_speed = 8; //m/s
    double set_max_acc = 4.5;  //m/s/s
    double set_min_acc = -8;   //m/s/s
    double set_max_yawrate = 0.8*PI/2.0;
    double set_max_acc_y = 4.5; //max lateral acceleration
    double set_wait_speed = 1.5; //m/s
    double set_obstacle_sensitivity = 3.0; //[1,2,3,4,5]

private:
    /// @brief The planning environment for trajectory optimization. 
    /// This pointer should point to the global planning environment in data pool DataPool.
    PlanningEnv_S *_env = NULL;

    Trajectory_S last_traj_best_;
    double last_planning_time;
    struct StitchParams {
        double max_lateral_error = 0.1;
        double max_longitudinal_error = 0.5;
        double max_time_gap = 0.2;
    } stitch_params_;

    void copy_generator();

    void genControlPath(Trajectory_S *const traj, double t_gap);

    // Frenet State
    struct FrenetState {
        double s;      // Longitudinal position
        double s_d;    // Longitudinal velocity
        double s_dd;   // Longitudinal acceleration
        double d;      // Lateral position
        double d_d;    // Lateral velocity
        double d_dd;   // Lateral acceleration
    };
    std::vector<double> ref_arc_length;

    // Sample parameters for trajectory generation
    struct SampleParams {
        // Lateral sampling
        std::vector<double> d_targets;      // Target lateral offset
        double d_min = -1.0;                 // Minimum lateral offset
        double d_max = 1.0;                  // Maximum lateral offset
        int d_samples = 5;                    // Number of lateral samples
        
        // Longitudinal sampling
        std::vector<double> s_dot_targets;   // Target longitudinal velocity
        double v_min = 0.0;                    // Minimum speed
        double v_max = 10.0;                   // Maximum speed
        int v_samples = 10;                      // Number of speed samples
        
        // Time sampling
        std::vector<double> T_targets;        // Target time
        double T_min = 2.0;                     // Minimum planning time
        double T_max = 5.0;                     // Maximum planning time
        int T_samples = 4;                       // Number of time samples
    } sample_params;
    
    // Candidate trajectory set
    // std::vector<Trajectory_S> candidate_trajectories;
    
    // Utility functions
    void updateRefArcLength();
    bool getStitchedStartState(FrenetState &start_state, Point_Xd &start_point);
    bool findPointAtTime(const Trajectory_S &traj, double time, Point_Xd &point) const;
    double calculateLateralError(double x, double y, const Point_Xd &ref) const;
    FrenetState frenetFromCartesian(double x, double y, double heading, 
                                    double vx, double vy, double ax, double ay);
    FrenetState getCurrentFrenetState();
    void generateCandidateTrajectories(const FrenetState &start_state);
    void generateFrenetTrajectory(const FrenetState& start, 
                                  double d_target, 
                                  double s_dot_target, 
                                  double T_target,
                                  Trajectory_S& traj);
    Point_Xd frenetToCartesian(const FrenetState& frenet_state);
    bool frenetToCartesian(Trajectory_S& traj);
    bool checkCollision(const Trajectory_S& traj);
    double calculateCost(const Trajectory_S& traj);
    void selectBestTrajectory();
    int findNearestRefPathPoint(double x, double y);
    double calculateFrenetS(const X_Point& ref_point, double x, double y);
    double calculateFrenetD(const X_Point& ref_point, double x, double y, double theta);
};

#endif
