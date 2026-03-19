/*
 * opt.cpp
 * This is the main class for trajectory optimization
 * We already define some structes and variables that might be useful
 * Your main goal is to finish the constructor function ini() and run()
 * **YOU REALLY NEED TO FINISH THIS**
 */
#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>

#include "opt.h"
using namespace std;

/**
 * @brief The init function for trajectory optimization class.
 * @todo In this function, you need to initialize the planning environment 
 * and other variables that matter for your trajectory generation method.
 * @param planning_env global planning environment in the data pool
 */
int Opt::ini(PlanningEnv_S *planning_env)
{
	_env = planning_env;
    // Initialize sample
    // Lateral sampling
    sample_params.d_targets.clear();
    double d_step = (sample_params.d_max - sample_params.d_min) / (sample_params.d_samples - 1);
    for (int i = 0; i < sample_params.d_samples; ++i) {
        sample_params.d_targets.push_back(sample_params.d_min + i * d_step);
    }
    
    // velocity sampling
    sample_params.s_dot_targets.clear();
    double v_step = (sample_params.v_max - sample_params.v_min) / (sample_params.v_samples - 1);
    for (int i = 0; i < sample_params.v_samples; ++i) {
        sample_params.s_dot_targets.push_back(sample_params.v_min + i * v_step);
    }
    
    // time sampling
    sample_params.T_targets.clear();
    double T_step = (sample_params.T_max - sample_params.T_min) / (sample_params.T_samples - 1);
    for (int i = 0; i < sample_params.T_samples; ++i) {
        sample_params.T_targets.push_back(sample_params.T_min + i * T_step);
    }
    
    ROS_INFO("Opt initialized successfully");
	return 0;
}

/**
 * @brief The run function for trajectory optimization class. 
 * It is called in a loop (see GUI/widgets/3d_display/mainloop.cpp).
 * @todo In this function, you need to implement your trajectory optimization algorithm.
 * You can use the global planning environment `_env` to access the planning environment.
 * You can also use the global trajectory `traj_best` to store the optimal trajectory.
 */
void Opt::run()
{
	if (_env == NULL || _env->refPathVec.empty()) {
		//ROS_ERROR("Environment is NULL or reference path is empty");
		return;
	}
	ROS_INFO("Current state: x=%.2f, y=%.2f, heading=%.2f, speed_x=%.2f, acc_x=%.2f, yaw_rate=%.2f", 
			 _env->x, _env->y, _env->heading, _env->speed_x, _env->acc_x, _env->yaw_rate);
	/*****************************************************************************/
    ref_arc_length.clear();
    if (!_env->refPathVec.empty()) {
        ref_arc_length.push_back(0.0);
        for (size_t i = 1; i < _env->refPathVec.size(); ++i) {
            double dx = _env->refPathVec[i].x - _env->refPathVec[i-1].x;
            double dy = _env->refPathVec[i].y - _env->refPathVec[i-1].y;
            double dist = sqrt(dx*dx + dy*dy);
            ref_arc_length.push_back(ref_arc_length.back() + dist);
        }
    }
	// 1. Aciquire current Frenet state of the ego vehicle
    FrenetState current_state = getCurrentFrenetState();
	ROS_INFO("Current Frenet state: s=%.2f, s_d=%.2f, d=%.2f, d_d=%.2f", 
			 current_state.s, current_state.s_d, current_state.d, current_state.d_d);
    
    // 2. Generate candidate trajectories based on current Frenet state and sampling parameters
    candidate_trajectories.clear();
    generateCandidateTrajectories(current_state);
	ROS_INFO("%lu/%d trajectories feasible", candidate_trajectories.size(), sample_params.d_samples * sample_params.v_samples * sample_params.T_samples);
    
    // 3. Select the best trajectory from the candidate set based on the calculated cost
    selectBestTrajectory();

	ROS_INFO("Best trajectory final state: s_dot=%.2f, d=%.2f, t=%.2f", traj_best.ref_xf, traj_best.ref_yf, traj_best.final_tf);
    /*
	ROS_INFO("Best trajectory: ");
	for (size_t i = 0; i < traj_best.path.size(); ++i) {
		const Point_Xd& pt = traj_best.path[i];
		ROS_INFO("t=%.2f, x=%.2f, y=%.2f, heading=%.2f, v=%.2f", 
				 pt.t, pt.x, pt.y, pt.heading, pt.v);
	}*/

    // 4. Reformat the trajectory for control (0.04s per point)
    if (traj_best.feasible && !traj_best.path.empty()) {
        genControlPath(&traj_best, TIME_GAP_CONTROL);
        
        if (traj_best.path.size() > 0) {
            pt_goal = traj_best.path.back();
        }
    } else {
		ROS_WARN("No feasible trajectory found!");
		copy_generator(); // Fallback to reference path copying
		genControlPath(&traj_best, TIME_GAP_CONTROL);
		if (traj_best.path.size() > 0) {
			pt_goal = traj_best.path.back();
		}
	}
	return;
}


/**
 * @brief get current Frenet state of the ego vehicle
 * @return FrenetState struct containing s, s_d, s_dd, d, d
*/

Opt::FrenetState Opt::getCurrentFrenetState()
{
    FrenetState state;
    int nearest_idx = findNearestRefPathPoint(_env->x, _env->y);
    const X_Point& ref_point = _env->refPathVec[nearest_idx];
    
    double dx = _env->x - ref_point.x;
    double dy = _env->y - ref_point.y;
    double proj = dx * cos(ref_point.heading) + dy * sin(ref_point.heading);
    state.s = ref_arc_length[nearest_idx] + proj;
    
    state.d = -dx * sin(ref_point.heading) + dy * cos(ref_point.heading);
    
    double cos_theta = cos(ref_point.heading - _env->heading);
    state.s_d = _env->speed_x * cos_theta;
    state.s_dd = 0.0;
    state.d_d = _env->speed_x * sin(_env->heading - ref_point.heading);
    state.d_dd = 0.0;
    
    return state;
}

/**
 * @brief generate candidate trajectories based on current Frenet state and sampling parameters
 */
void Opt::generateCandidateTrajectories(FrenetState current_state)
{
    // walk through all combinations of sampling parameters (Deterministic sampling)
    for (double d_target : sample_params.d_targets) {
        for (double s_dot_target : sample_params.s_dot_targets) {
            for (double T_target : sample_params.T_targets) {
                
                Trajectory_S traj;
                
                generateFrenetTrajectory(current_state, d_target, s_dot_target, T_target, traj);
                
                if (!frenetToCartesian(traj)) {
                    continue;
                }
                
                if (flag_checkObstacles && !checkCollision(traj)) {
                    continue;
                }
                
                traj.Jcost = calculateCost(traj);
                
                candidate_trajectories.push_back(traj);
            }
        }
    }
}

/**
 * @brief Generate control points for the selected trajectory. The control points are sampled at a fixed time interval (e.g., 0.04s) for the controller to follow.
 */
void Opt::generateFrenetTrajectory(const FrenetState& start, 
                                   double d_target, 
                                   double s_dot_target, 
                                   double T_target,
                                   Trajectory_S& traj)
{
    double T = T_target;
    
    // d(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    double a0 = start.d;
    double a1 = start.d_d;
    double a2 = start.d_dd / 2.0;

    double T2 = T * T;
	double T3 = T2 * T;
	double T4 = T3 * T;
	double T5 = T4 * T;

	double delta_d = d_target - a0;

	double a3 = (10.0 * delta_d - 6.0 * a1 * T - 1.5 * a2 * T2) / T3;
	double a4 = (-15.0 * delta_d + 8.0 * a1 * T + 1.5 * a2 * T2) / T4;
	double a5 = (6.0 * delta_d - 3.0 * a1 * T - 0.5 * a2 * T2) / T5;
    
    traj.cx[0] = a0;
    traj.cx[1] = a1;
    traj.cx[2] = a2;
    traj.cx[3] = a3;
    traj.cx[4] = a4;
    traj.cx[5] = a5;

    // s(t) = b0 + b1*t + b2*t^2 + b3*t^3 + b4*t^4
    double b0 = start.s;
    double b1 = start.s_d;
    double b2 = start.s_dd / 2.0;
    
    double A = s_dot_target - b1 - 2 * b2 * T;
	double B = -2 * b2;
	double b3 = A / (T2) - B / (3 * T);
	double b4 = -A / (2 * T3) + B / (4 * T2);
    
    traj.cy[0] = b0;
    traj.cy[1] = b1;
    traj.cy[2] = b2;
    traj.cy[3] = b3;
    traj.cy[4] = b4;
    
    traj.final_tf = T;
    traj.ref_xf = s_dot_target;
    traj.ref_yf = d_target;
}

/**
 * @brief Frenet to Cartesian conversion for the generated trajectory. This function converts the Frenet trajectory (defined by cx and cy) into a sequence of Cartesian points (x, y) that the vehicle can follow.
 */
bool Opt::frenetToCartesian(Trajectory_S& traj)
{
    traj.path.clear();
    if (_env->refPathVec.empty() || ref_arc_length.empty()) return false;
    
    double dt = TIME_GAP_PATH;
    double T = traj.final_tf;
    
    for (double t = 0; t <= T + 1e-6; t += dt) {
        // 计算 d(t), s(t) 等（与原代码相同）
        double d = 0, d_d = 0, d_dd = 0;
        for (int i = 0; i < 6; ++i) {
            double ti = pow(t, i);
            d += traj.cx[i] * ti;
            if (i > 0) d_d += i * traj.cx[i] * pow(t, i-1);
            if (i > 1) d_dd += i * (i-1) * traj.cx[i] * pow(t, i-2);
        }
        
        double s = 0, s_d = 0, s_dd = 0;
        for (int i = 0; i < 5; ++i) {
            double ti = pow(t, i);
            s += traj.cy[i] * ti;
            if (i > 0) s_d += i * traj.cy[i] * pow(t, i-1);
            if (i > 1) s_dd += i * (i-1) * traj.cy[i] * pow(t, i-2);
        }
        
        // 边界裁剪
        if (s < ref_arc_length.front()) s = ref_arc_length.front();
        if (s > ref_arc_length.back()) s = ref_arc_length.back();
        
        // 二分查找 s 在弧长数组中的区间
        int idx = 0;
        for (size_t i = 0; i < ref_arc_length.size() - 1; ++i) {
            if (s >= ref_arc_length[i] && s <= ref_arc_length[i+1]) {
                idx = i;
                break;
            }
        }
        // 线性插值得到参考点坐标和航向
        double ratio = (s - ref_arc_length[idx]) / (ref_arc_length[idx+1] - ref_arc_length[idx]);
        const X_Point& p1 = _env->refPathVec[idx];
        const X_Point& p2 = _env->refPathVec[idx+1];
        double ref_x = p1.x + ratio * (p2.x - p1.x);
        double ref_y = p1.y + ratio * (p2.y - p1.y);
        double ref_heading = p1.heading + ratio * (p2.heading - p1.heading);
        // 航向归一化
        while (ref_heading > M_PI) ref_heading -= 2*M_PI;
        while (ref_heading < -M_PI) ref_heading += 2*M_PI;
        
        // 计算全局坐标
        Point_Xd pt;
        pt.x = ref_x - d * sin(ref_heading);
        pt.y = ref_y + d * cos(ref_heading);
        
        // 航向角
        double heading_offset = 0.0;
        if (fabs(s_d) > 1e-6) {
            heading_offset = atan2(d_d, s_d);
        } else {
            heading_offset = (d_d > 0) ? M_PI/2 : -M_PI/2;
        }
        pt.heading = ref_heading + heading_offset;
        while (pt.heading > M_PI) pt.heading -= 2*M_PI;
        while (pt.heading < -M_PI) pt.heading += 2*M_PI;
        
        pt.v = sqrt(s_d * s_d + d_d * d_d);
        pt.t = t;
        
        // 可选曲率计算
        double denominator = pow(s_d*s_d + d_d*d_d, 1.5);
        if (denominator > 1e-6) {
            pt.cr = (s_d * d_dd - d_d * s_dd) / denominator;
        } else {
            pt.cr = 0.0;
        }
        
        // 边界信息近似取最近参考点的边界（或插值）
        pt.bound_left = p1.bound_left + ratio * (p2.bound_left - p1.bound_left);
        pt.bound_right = p1.bound_right + ratio * (p2.bound_right - p1.bound_right);
		// 边界检查：横向偏移 d 必须在 [-bound_right, bound_left] 内
		if (d < -pt.bound_right || d > pt.bound_left) {
			return false; // 超出道路边界，轨迹不可行
		}
        
        traj.path.push_back(pt);
    }
    
    return !traj.path.empty();
}

/**
 * @brief Check if the trajectory collides with any obstacles. This function checks each point in the trajectory against the positions of the obstacles and returns false if any point is too close to an obstacle.
 */
bool Opt::checkCollision(const Trajectory_S& traj)
{
    if (_env->obstacleVec.empty()) {
        return true;
    }
    
    for (const auto& pt : traj.path) {
		
        for (const auto& obs : _env->obstacleVec) {
            double dist = sqrt(pow(pt.x - obs.x_local, 2) + pow(pt.y - obs.y_local, 2));
            
            double safe_dist = obs.radius + 1.0;
            
            if (dist < safe_dist) {
                return false;
            }
        }
    }
    
    return true;
}

/**
 * @brief Calculate the cost of a trajectory based on multiple factors such as lateral offset, speed error, smoothness, and obstacle distance. The cost is a weighted sum of these factors, and the trajectory with the lowest cost will be selected as the optimal trajectory.
 */
double Opt::calculateCost(const Trajectory_S& traj)
{
    double cost = 0.0;
    
    // 1. 横向偏移代价（期望在车道中心）
    double avg_d = 0;
    for (const auto& pt : traj.path) {
        // 计算横向偏移（简化）
        int nearest = findNearestRefPathPoint(pt.x, pt.y);
        double dx = pt.x - _env->refPathVec[nearest].x;
        double dy = pt.y - _env->refPathVec[nearest].y;
        double d = abs(dx * sin(_env->refPathVec[nearest].heading) - 
                      dy * cos(_env->refPathVec[nearest].heading));
        avg_d += d;
    }
    avg_d /= traj.path.size();
    cost += 1.0 * avg_d;  // 权重1.0
    
    // 2. 速度误差代价（期望保持目标速度）
    double target_speed = set_max_speed;
    double avg_speed_error = 0;
    for (const auto& pt : traj.path) {
        avg_speed_error += fabs(pt.v - target_speed);
    }
    avg_speed_error /= traj.path.size();
    cost += 0.5 * avg_speed_error;  // 权重0.5
    
    // 3. 轨迹平滑度代价
    double jerk_cost = 0;
    for (int i = 2; i < traj.path.size(); ++i) {
        // 近似计算加加速度
        double jerk_x = (traj.path[i].v * cos(traj.path[i].heading) - 
                        2 * traj.path[i-1].v * cos(traj.path[i-1].heading) + 
                        traj.path[i-2].v * cos(traj.path[i-2].heading)) / (TIME_GAP_PATH * TIME_GAP_PATH);
        double jerk_y = (traj.path[i].v * sin(traj.path[i].heading) - 
                        2 * traj.path[i-1].v * sin(traj.path[i-1].heading) + 
                        traj.path[i-2].v * sin(traj.path[i-2].heading)) / (TIME_GAP_PATH * TIME_GAP_PATH);
        jerk_cost += sqrt(jerk_x * jerk_x + jerk_y * jerk_y);
    }
    jerk_cost /= traj.path.size();
    cost += 0.3 * jerk_cost;  // 权重0.3
    
    // 4. 障碍物距离代价
    double min_obs_dist = 100;
    for (const auto& pt : traj.path) {
        for (const auto& obs : _env->obstacleVec) {
            double dist = sqrt(pow(pt.x - obs.x_local, 2) + pow(pt.y - obs.y_local, 2));
            min_obs_dist = min(min_obs_dist, dist);
        }
    }
    if (min_obs_dist < 5.0) {
        cost += 2.0 * (5.0 - min_obs_dist);  // 距离越近代价越大
    }
    
    return cost;
}

/**
 * @brief Select the best trajectory from the candidate set based on the calculated cost. This function sorts the candidate trajectories by their cost and selects the one with the lowest cost as the optimal trajectory to be followed by the vehicle.
 */
void Opt::selectBestTrajectory()
{
    if (candidate_trajectories.empty()) {
        ROS_WARN("No feasible trajectory found");
        traj_best.feasible = false;
        return;
    }
    
    sort(candidate_trajectories.begin(), candidate_trajectories.end(),
         [](const Trajectory_S& a, const Trajectory_S& b) {
             return a.Jcost < b.Jcost;
         });
    
    traj_best = candidate_trajectories[0];
    traj_best.feasible = true;
    traj_best.flag_best = true;
}

/**
 * @brief find the nearest point index on reference path for given (x, y)
 * @return index of the nearest point on reference path
 */
int Opt::findNearestRefPathPoint(double x, double y)
{
    int nearest_idx = 0;
    double min_dist = 1e9;
    
    for (size_t i = 0; i < _env->refPathVec.size(); ++i) {
        double dist = _env->refPathVec[i].dist2XY(x, y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    return nearest_idx;
}

/**
 * @brief calculate Frenet longitudinal position s based on reference point and current (x, y)
 */
double Opt::calculateFrenetS(const X_Point& ref_point, double x, double y)
{
    double dx = x - ref_point.x;
    double dy = y - ref_point.y;
    double proj = dx * cos(ref_point.heading) + dy * sin(ref_point.heading);
    
    return ref_point.mileage + proj;
}

/**
 * @brief calculate Frenet lateral offset d based on reference point and current (x, y)
 */
double Opt::calculateFrenetD(const X_Point& ref_point, double x, double y, double theta)
{
    double dx = x - ref_point.x;
    double dy = y - ref_point.y;
    
    return -dx * sin(ref_point.heading) + dy * cos(ref_point.heading);
}

void Opt::copy_generator()
{
	if(_env == NULL) return;
	if(_env->refPathVec.size()<1) return;
	// clear the old trajectory
	traj_best.path.clear();
	traj_best.feasible = false;

	// update new NPN
    _env->NPN = XM::find_NPN(&_env->refPathVec, _env->x, _env->y);
	_env->NPN = MAX(0, MIN(_env->refPathVec.size()-1, _env->NPN));

	int N = _env->refPathVec.size();
	int N_max = MIN(N, _env->NPN+400);
    printf("N_max=%d\n", N_max);

	// set target point
	pt_goal.x = _env->refPathVec.at(N_max-1).x;
	pt_goal.y = _env->refPathVec.at(N_max-1).y;
	pt_goal.heading = _env->refPathVec.at(N_max-1).heading;
	
	// find and copy the future 50 points
	for (int i = _env->NPN; i < N_max; ++i)
	{
		Point_Xd pt;
		pt.x = _env->refPathVec.at(i).x;
		pt.y = _env->refPathVec.at(i).y;
		pt.heading = _env->refPathVec.at(i).heading;
		pt.t = _env->refPathVec.at(i).t-_env->refPathVec.at(_env->NPN).t;
		pt.v = _env->refPathVec.at(i).v;
		pt.cr = _env->refPathVec.at(i).cr;
		pt.bound_left = _env->refPathVec.at(i).bound_left;
		pt.bound_right = _env->refPathVec.at(i).bound_right;
		pt.mileage = _env->refPathVec.at(i).mileage;
		traj_best.path.push_back(pt);
	}
	traj_best.feasible = true;

}

