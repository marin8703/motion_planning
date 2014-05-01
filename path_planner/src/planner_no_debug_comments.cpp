#include "ros/ros.h"
#include "laser_obstacle_id/Obstacles.h"
#include "laser_obstacle_id/ObstaclesSrv.h"
#include "sensor_msgs/LaserScan.h"	
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

//will explicitly use namespaces for clarity
//using namespace std;
using namespace Eigen;

std::ofstream log_file;

namespace d_hold
{
	//Size of obstacle map, currently square only
	unsigned int map_size = 100;
	
	//Size of the back step in certain situations
	unsigned int step_back = 2; //was 2
	
	//Use to signal end of path (at goal)
	bool end = false;
	
	//N matrix hold
	std::vector<std::vector<float> > matrix_N;
	
	//Degree used during interpolation with B-Spline
	int interpolation_degree = 2;
	
	//this holds the location of "robot" before an obstacle was encountered, is used to avoid obstacle
	std::vector<int> pre_collison_hold(2,0);
	
	//the safe "cussion" distance around an obstacle that we want to maintain for our interpolation points
	//note that the actual final path may be closer than the safe distance, be aware
	int safety_cussion = 3;//was 2
	
	//overlook radius
	//should be at least or greater than safety_cussion above
	int overlook_radius = 4;	//when this is large it may interfere with negative indeces if center point is near origin
	
	//storage for the initial and final positions at begining of program
	std::vector<std::vector<float> > initial_final_positions(2, std::vector<float>(2,0.0));
	
	//limit on size of interval betwen interpolation points
	int ip_interval_limit = 7;
	
	//global storage of the final control points
	std::vector<std::vector<float> > final_CP;
	
	//global storage for the knot vector for plotting the curve, used mainly by motion_info()
	std::vector<float> t_storage;
}

std::vector<std::vector<int> > generate_map(float current_pos_[], float goal_pos_[])
{
	std::vector<std::vector<int> > obs_map(d_hold::map_size, std::vector<int>(d_hold::map_size,0));
	
	for(unsigned int i = 0; i < d_hold::map_size; i++)
	{
		for(unsigned int k = 0; k < d_hold::map_size; k++)
		{
			if(i >= 11 && i <= 16)	//obstacle 1
			{
				if(k >= 4 && k <= 24)
				{
					obs_map[i][k] = 1;
				}
				else
				{
					obs_map[i][k] = 0;
				}
			}
			else if(i >= 20 && i <= 24)	//obstacle 2
			{
				if(k >= 20 && k <= 35)
				{
					obs_map[i][k] = 1;
				}
				else
				{
					obs_map[i][k] = 0;
				}
			}
			else if(i >= 35 && i <= 42)	//obstacle 3
			{
				if(k >= 20 && k <= 70)
				{
					obs_map[i][k] = 1;
				}
				else
				{
					obs_map[i][k] = 0;
				}
			}
			else if(i >= 57 && i <= 65)	//obstacle 4
			{
				if(k >= 50 && k <= 90)
				{
					obs_map[i][k] = 1;
				}
				else
				{
					obs_map[i][k] = 0;
				}
			}
			else
			{
				obs_map[i][k] = 0;
			}
			
			//Insert current position and goal_position in obstacle map (informational only)
			//The position data is converted to int with rounding
			if(i == (int (current_pos_[0]+0.5)) && k == (int (current_pos_[1]+0.5)))
				obs_map[i][k] = 6; //random number to signify start (initial position)
			if(i == (int (goal_pos_[0]+0.5)) && k == (int (goal_pos_[1]+0.5)))
				obs_map[i][k] = 7; //Chose 7 to signify goal position because it is after 6
		}
	}
	
	return obs_map;
	
	//Map Legend:
	//0=empty
	//1=occupied
	//3=occupied by virtual obstacle
	//2=control point
	//5=interpolation point
	//6=initial position
	//7=final position
}

int mark_map(std::vector<std::vector<int> >& obstacle_map_, int x_, int y_, int mark_)
{
	obstacle_map_[x_][y_] = mark_;
	
	return 0;
}

int is_match(std::vector<std::vector<int> > obstacle_map_, int x_, int y_)
{
//	ROS_INFO("checking for obstacle...");
	if(obstacle_map_[x_][y_] == 1)// || obstacle_map_[x_][y_] == 3) ignore this for now
	{
		return 1;
	}
	
	return 0;	//1=match, 0=no match
}

int closest_points(std::vector<std::vector<int> >& obstacle_map_, std::vector<int> step_back_, std::vector<int>& dir_xy_, float& d_close_)
{
	//locations of noteworthy points and tangent of obstacle
	std::vector<std::vector<int> > closest(2,std::vector<int>(2,0));
	
	//distance to current and previous search location from search grid center, d_xy is current
	std::vector<float> d_close(2, 1000.0);
	float d_xy = 0.0;
	int ret_stat = 0;
	
	//search around with padding
	//the padding is d_hold::safety_cussion, because we need to look at least that far
	//away to be able to see if we have the safety distance
	//seach grid is standard from top left, going right and down
	for(int i_y = step_back_[1]+d_hold::overlook_radius; i_y >= step_back_[1]-d_hold::overlook_radius; i_y--)
	{
		for(int i_x = step_back_[0]-d_hold::overlook_radius; i_x <= step_back_[0]+d_hold::overlook_radius; i_x++)
		{
			if(obstacle_map_[i_x][i_y] == 1)
			{
				d_xy = sqrt(pow((step_back_[0]-i_x),2) + pow((step_back_[1]-i_y),2));
				if(d_xy <= d_close[1])
				{
					if(d_xy < d_close[0])
					{
						d_close[1] = d_close[0];
						closest[1][0] = closest[0][0];
						closest[1][1] = closest[0][1];
				
						d_close[0] = d_xy;
						closest[0][0] = i_x;
						closest[0][1] = i_y;
					}
					else
					{
						d_close[1] = d_xy;
						closest[1][0] = i_x;
						closest[1][1] = i_y;
					}
				}
			}
		}
	}
	if(d_close[0] != 1000 && d_close[1] != 1000)
	{
		d_close_ = d_close[0];
		ret_stat = 1;
	}
	else
		ret_stat = -1;
	
	//get direction
	dir_xy_[0] = closest[0][0] - closest[1][0];
	dir_xy_[1] = closest[0][1] - closest[1][1];

	return ret_stat;
}

int corridor_width(std::vector<std::vector<int> >& obstacle_map_, std::vector<int>& position_, std::vector<int>& dir_xy_, float& c_width_)
{
	//locations of noteworthy points and tangent of obstacle
	std::vector<std::vector<int> > pos_close(2,std::vector<int>(2,0));
	
	//distance to current and previous search location from search grid center, d_xy is current
	std::vector<float> c_close(2, 1000.0);
	float d_xy = 0.0;
	int status = 0;
	
//	ROS_INFO("entering corridor_width...");
	
	//search around with padding, get 2 closest points to current position
	//the padding is d_hold::safety_cussion, because we need to look at least that far
	//away to be able to see if we have the safety distance
	//seach grid is standard from top left, going right and down
	for(int i_y = position_[1]+d_hold::overlook_radius; i_y >= position_[1]-d_hold::overlook_radius; i_y--)
	{
		for(int i_x = position_[0]-d_hold::overlook_radius; i_x <= position_[0]+d_hold::overlook_radius; i_x++)
		{
//			ROS_INFO("i_x= %d, i_y= %d, obstacle_map_= %d...", i_x, i_y, obstacle_map_[i_x][i_y]);	//debug
			if(obstacle_map_[i_x][i_y] == 1)
			{
//				ROS_INFO("i_x= %d, i_y= %d, obstacle_map_= %d...", i_x, i_y, obstacle_map_[i_x][i_y]);	//debug
				d_xy = sqrt(pow((position_[0]-i_x),2) + pow((position_[1]-i_y),2));
				if(d_xy <= c_close[1])
				{
					if(d_xy < c_close[0])
					{
						c_close[1] = c_close[0];
						pos_close[1][0] = pos_close[0][0];
						pos_close[1][1] = pos_close[0][1];
				
						c_close[0] = d_xy;
						pos_close[0][0] = i_x;
						pos_close[0][1] = i_y;
//						ROS_INFO("pos_close[0]= %d, %d; i_x= %d, i_y= %d...", pos_close[0][0], pos_close[0][1], i_x, i_y);	//debug
					}
					else
					{
						c_close[1] = d_xy;
						pos_close[1][0] = i_x;
						pos_close[1][1] = i_y;
//						ROS_INFO("pos_close[1]= %d, %d; i_x= %d, i_y= %d...", pos_close[1][0], pos_close[1][1], i_x, i_y);	//debug
					}
				}
			}
		}
	}
	if(c_close[0] != 1000 && c_close[1] != 1000)	//2 close points found, so assuming corridor
	{
		float c_width = 0.0;
		//width of cooridor
		c_width = sqrt(pow((pos_close[1][0]-pos_close[0][0]),2) + pow((pos_close[1][1]-pos_close[0][1]),2));
		if((c_width / 2.0) < float (d_hold::safety_cussion*1.0) && c_width > 1.0)	//checking if there will be safety cussion on aech side downthe cooridoor
		{
			//safety_cussion will not be satisfied, so create a virtual obstacle to block the corridor ("gate"),
			//by connecting the 2 points
			
//			ROS_INFO("generating virtual gate (obstacle)...");
			
			//get slope
			int c_slope_x = pos_close[1][0] - pos_close[0][0];
			int c_slope_y = pos_close[1][1] - pos_close[0][1];
			int mark_pos_x = 0, mark_pos_y = 0;
			for(double j = 0.0; j <= 10.0; j = j + 0.1)
			{
				mark_pos_x = int ((pos_close[0][0]+c_slope_x*j)+0.5);
				mark_pos_y = int ((pos_close[0][1]+c_slope_y*j)+0.5);
//				mark_map(obstacle_map_, mark_pos_x, mark_pos_y, 3);	
				if(mark_pos_x == pos_close[1][0] && mark_pos_y == pos_close[1][1])
				{
					status = 1;	//something found
					break;
				}
			}
			if(status != 1)	//meaning complete gate was not generated
			{
				status = -2;
			}
		}
		
	}
	else
		status = 0;	//no corridor found, 
	
	//get direction
	dir_xy_[0] = pos_close[1][0] - pos_close[0][0];
	dir_xy_[1] = pos_close[1][1] - pos_close[0][1];

	return status;	//-1=error, no width; 0= no corridor found, no width; 1= corridor found, returning width, -2=unable to create virtual "gate"
}

//step along the curve
int forward_step(std::vector<std::vector<int> >& obstacle_map_, std::vector<std::vector<float> > interpolation_points_, int cur_segment_, int& hold_x_, int& hold_y_)
{
	int ret_status = 0;
	int corridor_status = 0;
	float slope_x_int = 0, slope_y_int = 0;
	float step_interval = 0.1;
	int num_ip = int (interpolation_points_.size());
	int prev_hold_x=0, prev_hold_y=0;
	bool exit_both_loops = false;
	float interval_hold = 0.0;
	std::vector<int> back_step_proper(2,0.0);
	float back_step_distance=0.0;
	
	//the follwing variables are used for checking the narrow path constraint
	int narrow_path_obstacle = 0;
	float d_narrow_path=0.0;
	std::vector<int> dir_xy_dummy(2,0);
	std::vector<int> search_location(2,0);
	
	ROS_INFO("stepping forward...");

	for(int i = cur_segment_; i < num_ip; i++)	//iterate over each segment (one segment between each concecutive 2 points
	{
		//slope between begining and ending point of segment, which we will move along and look for obstacles
		slope_x_int = (interpolation_points_[i+1][0]-interpolation_points_[i][0]);
		slope_y_int = (interpolation_points_[i+1][1]-interpolation_points_[i][1]);
		
		for(double k = 0.00; k <= 1.0; k = k + 0.01)
		{
			d_hold::pre_collison_hold[0] = hold_x_;
			d_hold::pre_collison_hold[1] = hold_y_;	
			
			hold_x_ = int ((interpolation_points_[i][0] + slope_x_int*k)+0.5);
			hold_y_ = int ((interpolation_points_[i][1] + slope_y_int*k)+0.5);
			search_location[0] = hold_x_;
			search_location[1] = hold_y_;
						
			//Check current pos for occupancy
			if(is_match(obstacle_map_, hold_x_, hold_y_))// || narrow_path_obstacle==1)
			{				
				//back stepping proper amount
				for(double l = 0.0; l <= 1.0; l = l+0.01)
				{
					back_step_proper[0] = int ((interpolation_points_[i][0] + slope_x_int*(k-l))+0.5);
					back_step_proper[1] = int ((interpolation_points_[i][1] + slope_y_int*(k-l))+0.5);
					
					back_step_distance = sqrt(pow((hold_x_-back_step_proper[0]),2) + pow((hold_y_-back_step_proper[1]),2));
					
					if(d_hold::step_back <= int (back_step_distance +0.5))
					{
						d_hold::pre_collison_hold[0] = back_step_proper[0];
						d_hold::pre_collison_hold[1] = back_step_proper[1];
						break;
					}
				}	
				ret_status = 1;
				exit_both_loops = true;
			}
			//do if a narrow corridor is around
			if(narrow_path_obstacle==1)
			{
				ret_status = 4;
				exit_both_loops = true;
			}			
			if(int (d_hold::initial_final_positions[1][0]+0.5)==hold_x_ && int (d_hold::initial_final_positions[1][1]+0.5)==hold_y_)
			{
				ret_status = 3;
				exit_both_loops = true;
			}
			//check if limit on interval between control points is reached
			interval_hold = sqrt( pow((1.0*hold_x_-interpolation_points_[i][0]),2) + pow((1.0*hold_y_-interpolation_points_[i][1]),2))+0.5;
			if(d_hold::ip_interval_limit <= int (interval_hold) && ret_status != 1)
			{
				ret_status = 5;	//insert ip at current point
				exit_both_loops = true;
			}
			if(exit_both_loops)	//requestd to exit both loops and complete this stepping instance
			{
				i = 100000; //number thats more than it ever should be to prevent re loop
				k = 100000.0;	//number thats more than it ever should be to prevent re loop
			}
//			ROS_INFO("Dfs5...");	//debug
			narrow_path_obstacle = 0;	//reset the narrow_path check
			corridor_status = 0;		//reset corridor_status
			interval_hold = 0.0;		//reset interval hold
		}
	}
	ROS_INFO("stepping complete...");

	return ret_status;	//1=obstacle found, 0=nothing found, 3=goal found, 4=narrow pathway found(obstacle), 5=ip_interval_limit reached
}	

int add_new_ip(std::vector<std::vector<float> >& interpolation_points_, std::vector<float>& new_ip_)
{
	//uncomment after debuging
	if(new_ip_[0] == 0 && new_ip_[1] == 0)
		return -1;
		
	//store last ip in incoming vector (the goal position interpolation point)
	std::vector<float> hold_goal_ip(2,0);
	hold_goal_ip[0] = interpolation_points_[interpolation_points_.size()-1][0];
	hold_goal_ip[1] = interpolation_points_[interpolation_points_.size()-1][1];
	
	
	//Place new ip as last interpolation point
	interpolation_points_[interpolation_points_.size()-1][0] = new_ip_[0];
	interpolation_points_[interpolation_points_.size()-1][1] = new_ip_[1];
	
	//Complete by placing the goal back as the last interpolation point
	interpolation_points_.push_back(std::vector<float>(2,0));
	interpolation_points_[interpolation_points_.size()-1][0] = hold_goal_ip[0];
	interpolation_points_[interpolation_points_.size()-1][1] = hold_goal_ip[1];
	
	ROS_INFO("add_new_ip completed...");
	return 1;
}

//given the next collision point, find proper placement of a new control point to avoid obstacle
int avoid_obstacle(std::vector<std::vector<int> >& obstacle_map_, std::vector<float>& next_collision_, std::vector<std::vector<float> >& interpolation_points_, int cur_segment_, std::vector<float>& new_ip_)
{
	ROS_INFO("avoiding obstacle...");	
	
	std::vector<int> dir_xy(2,0);		//direction at our initial closest point
	std::vector<int> step_dir_xy(2,0);	//direction but once moving along tangent
	float d_close = 0.0, step_d_close = 0.0;
	std::vector<int> step_tangent(2,0);
	std::vector<std::vector<int> > new_ips(2, std::vector<int>(2,0));
	int clo_pts_return_status = 0;
	int dir_1_intersected = 0, dir_2_intersected = 0;
	float interval_hold = 0.0;
	
	//center of search grid
	std::vector<int> step_back = d_hold::pre_collison_hold;

	//find closest tangent (use slope again)
	//remember dir gives 2 tangents, both directions
	closest_points(obstacle_map_, step_back, dir_xy, d_close);
	
	//moving in first direction (doesnt matter which)
	for(int i = 0; i<d_hold::map_size ; i++)	//shouldnt have to go farther than the map size
	{
		step_tangent[0] = i*dir_xy[0] + step_back[0];
		step_tangent[1] = i*dir_xy[1] + step_back[1];		

		clo_pts_return_status = closest_points(obstacle_map_, step_tangent, step_dir_xy, step_d_close);	
		if(int (is_match(obstacle_map_, step_tangent[0], step_tangent[1])) || step_tangent[0] >= d_hold::map_size || step_tangent[1] >= d_hold::map_size)
		{
			dir_1_intersected = 1;
			break;
		}
		if(int (step_d_close+0.3) >= d_hold::safety_cussion && clo_pts_return_status == 1)	//using 0.3 here to push the rounding towards the lower rounding
		{
			new_ips[0][0] = step_tangent[0];
			new_ips[0][1] = step_tangent[1];
			break;
		}
		interval_hold = 0.0;
		clo_pts_return_status = 0;
	}
	ROS_INFO("first direction complete...");	//debug
			
	//next other direction (oposite from previous *(-1))
	step_d_close = 0.0;
	clo_pts_return_status = 0;
	for(int i = 0; i<d_hold::map_size ; i++)	//shouldnt have to go farther than the map size
	{
		step_tangent[0] = i*dir_xy[0]*(-1) + step_back[0];
		step_tangent[1] = i*dir_xy[1]*(-1) + step_back[1];
	
		clo_pts_return_status = closest_points(obstacle_map_, step_tangent, step_dir_xy, step_d_close);
		if(int (is_match(obstacle_map_, step_tangent[0], step_tangent[1])) || step_tangent[0] >= d_hold::map_size || step_tangent[1] >= d_hold::map_size)
		{
			dir_2_intersected = 1;
			break;
		}
		if(int (step_d_close+0.3) > d_hold::safety_cussion && clo_pts_return_status == 1)	//using 0.3 here to push the rounding towards the lower rounding
		{
			new_ips[1][0] = step_tangent[0];
			new_ips[1][1] = step_tangent[1];
			break;
		}
		interval_hold = 0.0;
		clo_pts_return_status = 0;
	}
	ROS_INFO("second direction complete...");	//debug
		
	//Compare teh new_ips to see which is close to the goal and choose that one.
	int d_ip_1 = int ( sqrt(pow((d_hold::initial_final_positions[1][0]-new_ips[0][0]),2) + pow((d_hold::initial_final_positions[1][1]-new_ips[0][1]),2)) );
	if(dir_1_intersected)	//used to ignre this direction because it crossed another obstacle
	{
		d_ip_1 = 10000;	//more than it should ever be
	}	
	int d_ip_2 = int ( sqrt(pow((d_hold::initial_final_positions[1][0]-new_ips[1][0]),2) + pow((d_hold::initial_final_positions[1][1]-new_ips[1][1]),2)) );
	if(dir_2_intersected)	//used to ignre this direction because it crossed another obstacle
	{
		d_ip_2 = 10000;	//more than it should ever be
	}	
	if(d_ip_1 <= d_ip_2)
	{
		new_ip_[0]= float (new_ips[0][0]*1.0);
		new_ip_[1]= float (new_ips[0][1]*1.0);
	}
	if(d_ip_2 < d_ip_1)
	{
		new_ip_[0] = float (new_ips[1][0]*1.0);
		new_ip_[1] = float (new_ips[1][1]*1.0);
	}

	ROS_INFO("avoid_obstacle complete...");	//debug
	return 1;
}

//Search path of current control points for obstacle intersections
int search_path(std::vector<std::vector<int> >& obstacle_map_, std::vector<std::vector<float> >& interpolation_points_, std::vector<float>& next_collision_, int& cur_segment_)
{
	std::vector<float> next_collision(2,0);
	int search_status = 0;	//0=no obstacle returned, 1=obstacle returned, -1=error
	int hold_x=0, hold_y=0;
	bool repeat_call = false;
	
	do	//should run once and not repeat, unless a virtual obstacle was created
	{
		//double check nessesity of this block
		if(cur_segment_ == int (interpolation_points_.size()))
		{
			d_hold::end = true;
			return 0;
		}
	
		int step_check = forward_step(obstacle_map_, interpolation_points_, cur_segment_, hold_x, hold_y);
		if(step_check == 1)	//obstacle found
		{
			//set new obstacle point
			next_collision[0] = hold_x;
			next_collision[1] = hold_y;
			search_status = 1;
		
			//This executes after a collision is found and is used to see if indeed there has been a new collision
			if(next_collision[0] != 0 && next_collision[1] != 0)
			{
				next_collision_ = next_collision;
				cur_segment_++;
			}
		}
		if(step_check == 3)	//goal reached
		{
			search_status = 3;
		}	
		if(step_check == 5)// && search_status != 1)	//ip_interval_limit exceeded
		{
			//create vector for easier passing
			std::vector<float> new_ip(2,0.0);
			new_ip[0] = hold_x*1.0;
			new_ip[1] = hold_y*1.0;
			
			//add new ip with current position
			add_new_ip(interpolation_points_, new_ip);
			
			mark_map(obstacle_map_, new_ip[0], new_ip[1], 5);
			
			search_status = 5;
		}
	}while(repeat_call);	//should run once and not repeat, unless a virtual obstacle was created
	
	ROS_INFO("search completed...");
	return search_status;	//0=no obstacle returned, 1=obstacle returned, -1=error, 3=goal is reached, 5=new ip due to interval exceeding limit
}

//generate blending function specific to the input parameters
float generate_N_i_p(std::vector<float>& t_, std::vector<float>& p_, unsigned int i_, unsigned int p_i_, unsigned int degree_, unsigned int n_, std::vector<int> non_zero_int_p_ )
{
	degree_ = degree_ + 1;	//nessesary because of the whole degree/order misunderstanding
	std::vector<std::vector<float> > n_hold(n_+degree_+2+1, std::vector<float>(n_+degree_+2+1, 0.0));
	
	int order = degree_;	//not + 1 in this case because we want k to be the degree value???
	
	for(int k = 1; k<=order; k++)
	{
		if(k == 1)
		{
			for(int i = 0; i < n_+k+1; i++)//i_+degree_
			{
				if(t_[i] <= p_[p_i_] && p_[p_i_] <= t_[i+1])
				{				
					n_hold[i][k] = 1.0;
				}
			}
		}
		if(k == 2)
		{
			float a1=0.0, a2=0.0, b1=0.0, b2=0.0;
			for(int i = 0; i < n_+k+1; i++)
			{				
				a1=0.0;
				a2=0.0;
				b1=0.0;
				b2=0.0;
				
				a1 = ((p_[p_i_] - t_[i]) / (t_[i+k-1] - t_[i]));
				if(std::isfinite(a1)==0)
					a1=0.0;
			
				a2 = n_hold[i][k-1];
				if(std::isnan(a2)!=0)
					a2=0.0;
			
				b1 = ((t_[i+k] - p_[p_i_]) / (t_[i+k] - t_[i+1]));
				if(std::isfinite(b1)==0)
					b1=0.0;
			
				b2 = n_hold[i+1][k-1];
				if(std::isnan(b2)!=0)
					b2=0.0;
				
				n_hold[i][k] = a1*a2 + b1*b2;
			}
		}		
		if(k == 3)
		{
			float a1=0.0, a2=0.0, b1=0.0, b2=0.0;
			for(int i = 0; i < n_+k+1; i++)
			{
				a1=0.0;
				a2=0.0;
				b1=0.0;
				b2=0.0;
				
				a1 = ((p_[p_i_] - t_[i]) / (t_[i+k-1] - t_[i]));
				if(std::isfinite(a1)==0)
					a1=0.0;
			
				a2 = n_hold[i][k-1];
				if(std::isnan(a2)!=0)
					a2=0.0;
			
				b1 = ((t_[i+k] - p_[p_i_]) / (t_[i+k] - t_[i+1]));
				if(std::isfinite(b1)==0)
					b1=0.0;
			
				b2 = n_hold[i+1][k-1];
				if(std::isnan(b2)!=0)
					b2=0.0;
				
				n_hold[i][k] = a1*a2 + b1*b2;
			}
		}	
	}
	
	return n_hold[i_][degree_];
}

int eq_solve(std::vector<std::vector<float> >& matrix_N_, std::vector<std::vector<float> >& interpolation_points_, std::vector<std::vector<float> >& control_points_)
{
	
	std::vector<std::vector<float> > interpolation_points = interpolation_points_;
	std::vector<std::vector<float> > control_points(interpolation_points_.size(), std::vector<float>(2,0.0));
	
	int com_size = int (matrix_N_.size());
	
	MatrixXf m_N(com_size, com_size);
	for(int i = 0; i < com_size; i++)
	{
		for(int j = 0; j < com_size; j++)
		{
			m_N(i,j) = float (matrix_N_[i][j]);
		}
	}
	
	VectorXf d_x(com_size), d_y(com_size);
	for(int i = 0; i < com_size; i++)
	{
		d_x(i) = interpolation_points_[i][0];	//for x
		d_y(i) = interpolation_points_[i][1];	//for y
	}
	
	VectorXf p_x(com_size), p_y(com_size);
	for(int i = 0; i < com_size; i++)
	{
		p_x(i) = 0.0;	//for x
		p_y(i) = 0.0;	//for y
	}
	
	p_x = m_N.householderQr().solve(d_x);
	assert(d_x.isApprox(m_N*p_x));
	
	p_y = m_N.householderQr().solve(d_y);
	assert(d_y.isApprox(m_N*p_y));

	//assign new CPs
	for(int k = 0; k < com_size; k++)
	{
		control_points[k][0] = p_x(k);
		control_points[k][1] = p_y(k);
	}
	
	//assign new CPs to the external CP matrix
	control_points_ = control_points;	//return by reference the new cps

	ROS_INFO("eq_solve completed...");	//debug
	return 0;
}

//interpolation function, gives control points with interpolation point input
int interpolate_points(std::vector<std::vector<float> >& interpolation_points_, 
	std::vector<std::vector<float> >& control_points_)
{
	unsigned int degree = d_hold::interpolation_degree;	//p
	unsigned int n = interpolation_points_.size()-1;
	unsigned int m = n + degree +1;	//so then have m+1 number of knots
	
	//generate parameters
	std::vector<float> L(n, 0);
	float L_total = 0.0;
	float p_hold = 0.0;
	for(unsigned int k=0; k < n; k++)
	{
		L[k] = sqrt( sqrt(pow((interpolation_points_[k+1][0]-interpolation_points_[k][0]),2)
			+ pow((interpolation_points_[k+1][1]-interpolation_points_[k][1]),2)));
		L_total = L_total + L[k];
	}
	
	std::vector<float> p(n+1,0);
	for(unsigned int k = 0; k < n; k++)
	{
		p_hold = 0.0;
		for(int i=0;i<k;i++)
		{
			p_hold = p_hold + L[i];
		}
		p[k] = p_hold / L_total;
	}
	p[n] = 1;
	ROS_INFO("parameter vector generated...");

	//generate knot vector
	std::vector<float> t(m+1,0);
	float t_hold=0.0;
	for(unsigned int j = 1; j <= n-degree; j++)
	{
		t_hold = 0.0;
		for(unsigned int i=j; i <= j+degree-1; i++)
		{
			t_hold = t_hold + p[i];
		}
		t[j+degree] = t_hold / degree;
	}

	for(unsigned int j=m-degree; j <= m; j++)
	{
		t[j] = 1;
	}
	ROS_INFO("knot vector generated...");

	//begine generation of N matrix
	std::vector<int> non_zero_intervals;
	for(unsigned int i=0; i<int (p.size()); i++)
	{
		if((t[i+1]-t[i]) !=0)
		{
			non_zero_intervals.push_back(i);
		}
	}
	
	//find non zero intervals of the parameter vector p
	std::vector<int> non_zero_int_p;
	for(unsigned int i=0; i< int (p.size())-1;i++)
	{
		if((p[i+1]-t[i]) !=0)
		{
			non_zero_int_p.push_back(i);
		}
	}

	std::vector<std::vector<float> > matrix_N(n+1, std::vector<float>(n+1,0.0));	
	float hold_matrix_N = 0.0;
	for(unsigned int p_i = 0; p_i <= n; p_i++)
	{
		hold_matrix_N = 0.0;
		for(unsigned int i = 0; i <= n; i++)
		{			
			hold_matrix_N = generate_N_i_p(t, p, i, p_i, 2, n, non_zero_int_p);					
			if(std::isnan(hold_matrix_N) == 0)
				matrix_N[p_i][i] = hold_matrix_N;
		}
	}
	
	//Solve interpolation_points = matrix_N*control_points
	eq_solve(matrix_N, interpolation_points_, control_points_);
	
	ROS_INFO("Interpolate_points completed...");

	return 0;
}

std::vector<std::vector<float> > plan_path(std::vector<std::vector<int> >& obstacle_map_, float current_pos_[], float goal_pos_[])
{
	std::vector<std::vector<float> > control_points(1,std::vector<float>(2,0));
	std::vector<std::vector<float> > interpolation_points(1,std::vector<float>(2,0));
	std::vector<float> next_collision_hold(2,0);
	std::vector<float> new_ip(2,0);
	int search_stat = 0;
	int cur_segment = 0;
	int count=0;	//debug, erase when done
	int avoid_obstacle_status = 0;
	
	interpolation_points[0][0] = current_pos_[0];
	interpolation_points[0][1] = current_pos_[1];
	interpolation_points.push_back(std::vector<float>(2,0));
	interpolation_points[interpolation_points.size()-1][0] = goal_pos_[0];
	interpolation_points[interpolation_points.size()-1][1] = goal_pos_[1];
	
	//main control point search loop
	//move along the polynomial from the previous iteration
	do
	{
//		if(count == 1)		//to limit loop in debug
//			d_hold::end = true;
		
		search_stat = search_path(obstacle_map_, interpolation_points, next_collision_hold, cur_segment);
		if(search_stat == 1) //Gives the next point where collision occurs along the path
		{
			avoid_obstacle_status = avoid_obstacle(obstacle_map_, next_collision_hold, interpolation_points, cur_segment, new_ip);
			if(avoid_obstacle_status == 1) //pos for new interpolation pont to avoid obs.
			{
				add_new_ip(interpolation_points, new_ip); //adds the new ip to the total ip vector	
			}
			if(avoid_obstacle_status == 2)	//new ip created due to exceeding the ip interval limit during tangent_stepping
			{
				cur_segment++;	//so that next time it satarts from this new ip
			}
		}
		if(search_stat == 5 && avoid_obstacle_status != 2)	//new ip created because exceeded interval limit
		{
			cur_segment++;	//so that next time it satarts from this new ip
		}
		if(search_stat == 3)
		{
			d_hold::end = true;
		}
//		count++;	//to limit loop in debug
		avoid_obstacle_status = 0;
		search_stat = 0;
	}while(!d_hold::end);
	search_stat = 0;
	
	//function to interpolate the points
	interpolate_points(interpolation_points, control_points);	
	
	ROS_INFO("plan_path() completed...");

	return control_points;
}

int plot_path(std::vector<std::vector<float> > P_cp)
{
	//initialize variables
	int n=0, order=0, degree=0, num_cp=0, m=0;
	std::vector<std::vector<float> > path_xy;
	
	log_file << "Control Points: \n";
	for(int i=0; i<P_cp.size();i++)
	{
		for(int k=0;k<2;k++)
		{
			log_file << P_cp[i][k]<<",";
		}
		log_file << "\n";
	}
	log_file << "\n\n";
	ROS_INFO("CP output loop complete...");
	
	degree = 2;
	order = degree + 1;
	num_cp = P_cp.size();
	n = num_cp - 1;
	
	//number of knots
	m = n + order + 1;
	
	//generate knot vector
	std::vector<int> t(m,0);
	for(int t_i=order; t_i <= n; t_i++)
	{
		t[t_i] = t_i - order + 1;
	}
	for(int t_i=(m-order); t_i <= (n+order); t_i++)
	{
		t[t_i] = n - order + 2;
	}
	
	std::vector<int> non_zero_intervals;
	for(unsigned int i=0; i<m;i++)
	{
		if((t[i+1]-t[i]) !=0)
		{
			non_zero_intervals.push_back(i);
		}
	}
	ROS_INFO("Knot vector generated...");
		
	//variables for cox de boor P
	int l;
	int count=0;
	
	//generate x-y matrix of curve at some resolution using cox de boor
	for(float u=0; u <= t[t.size()-1]; u=u+0.05)
	{
		//interval of u
		for(int t_i=0; t_i < t.size()-1; t_i++)
		{
			if(u >= t[t_i] && u < t[t_i+1])
				l = t_i;
		}
		
		std::vector<std::vector<float> > P_cox_x(order, std::vector<float>(l+1,0.0)), P_cox_y(order,
			std::vector<float>(l+1,0.0));
		
		//method
		for(int r = 0; r < order; r++)
		{
			for(int i = (l-order+1+r); i<=l; i++)
			{
				if(r == 0)
				{
					P_cox_x[r][i] = P_cp[i][0];
					P_cox_y[r][i] = P_cp[i][1];
				}
				else
				{
					P_cox_x[r][i] = ((u-t[i])/(t[i+order-r]-t[i]))*P_cox_x[r-1][i] 
						+ ((t[i+order-r]-u)/(t[i+order-r]-t[i]))*P_cox_x[r-1][i-1];
					P_cox_y[r][i] = ((u-t[i])/(t[i+order-r]-t[i]))*P_cox_y[r-1][i] 
						+ ((t[i+order-r]-u)/(t[i+order-r]-t[i]))*P_cox_y[r-1][i-1];					
				}
			}
		}
		path_xy.push_back(std::vector<float>(2,0));
		path_xy[count][0] = P_cox_x[order-1][l];
		path_xy[count][1] = P_cox_y[order-1][l];
		count++;
	}
	ROS_INFO("Cox-de-boor complete...");
	
	//Dysplay points for debug and log
	log_file << "Path points: \n";
	for(int i=0; i < path_xy.size(); i++)
	{
		log_file << path_xy[i][0] << ", " << path_xy[i][1] << "\n";
	}
	log_file << "\n\n";
	
	ROS_INFO("plot_path() complete...");
		
	return 0;
}

//same as plot_path generally, but this one is used by the motion_info function mostly, takes a specific parameter,
//and uses the Cps stored in the namespaced_hold::final_CP
std::vector<float> evaluate_path_point(float u_)	
{
	//initialize variables
	int n=0, order=0, degree=0, num_cp=0, m=0;
	std::vector<float> point_xy(3,0.0);
	
	degree = 2;
	order = degree + 1;
	num_cp = d_hold::final_CP.size();
	n = num_cp - 1;
	
	//number of knots
	m = n + order + 1;
	
	//generate knot vector
	std::vector<int> t(m,0);
	for(int t_i=order; t_i <= n; t_i++)
	{
		t[t_i] = t_i - order + 1;
	}
	for(int t_i=(m-order); t_i <= (n+order); t_i++)
	{
		t[t_i] = n - order + 2;
	}
	ROS_INFO("Knot vector generated...");
	
	//store the knot vector in namespace d_hold, to be used by the motion_info() function
	for(int t_i=1; t_i < t.size(); t_i++)
	{
		d_hold::t_storage.push_back(t[t_i]);
	}
	
	//find non zero intervals in knot vector
	std::vector<int> non_zero_intervals;
	for(unsigned int i=0; i<m;i++)
	{
		if((t[i+1]-t[i]) !=0)
		{
			non_zero_intervals.push_back(i);
		}
	}
		
	//variables for cox de boor P
	int l;
	
	//generate curve value at given u using cox de boor
	//if user values is larger than allower break out
	std::vector<float> error_return(2,-1.0);
	if(0.0 > u_ || u_> t[t.size()-1])
		return error_return;
		
	//interval of u
	for(int t_i=0; t_i < t.size()-1; t_i++)
	{
		if(u_ >= t[t_i] && u_ < t[t_i+1])
			l = t_i;
	}
	
	std::vector<std::vector<float> > P_cox_x(order, std::vector<float>(l+1,0.0)), P_cox_y(order,
		std::vector<float>(l+1,0.0));
	
	//method
	for(int r = 0; r < order; r++)
	{
		for(int i = (l-order+1+r); i<=l; i++)
		{
			if(r == 0)
			{
				P_cox_x[r][i] = d_hold::final_CP[i][0];
				P_cox_y[r][i] = d_hold::final_CP[i][1];
			}
			else
			{
				P_cox_x[r][i] = ((u_-t[i])/(t[i+order-r]-t[i]))*P_cox_x[r-1][i] 
					+ ((t[i+order-r]-u_)/(t[i+order-r]-t[i]))*P_cox_x[r-1][i-1];
				P_cox_y[r][i] = ((u_-t[i])/(t[i+order-r]-t[i]))*P_cox_y[r-1][i] 
					+ ((t[i+order-r]-u_)/(t[i+order-r]-t[i]))*P_cox_y[r-1][i-1];				
			}
		}
	}
	point_xy[0] = P_cox_x[order-1][l];
	point_xy[1] = P_cox_y[order-1][l];
		
	ROS_INFO("Cox-de-boor for u complete...");

	ROS_INFO("evaluate_path_point completed...");
	
	return point_xy;
}

int motion_info()
{
	//call with random u, so it can update teh global knot vector, result not used
	evaluate_path_point(0.0);
	
	//possibly when you put in last point, whatever it is, that there might be a segmentation error, sincce
	//we also get the point after that
	float i_u = 0.0;	//when to get this data from curve
	float dt = 0.01;	//using this as the dt interval
	
	std::cout << "Enter parameter to evaluate (0:" << ( d_hold::t_storage[int (d_hold::t_storage.size()-1)] -0.1) << "):" << std::endl;	//randomly choose 0.1
	std::cin >> i_u;
	
	std::vector<float> t(3,0.0);
	std::vector<float> b(3,0.0);
	std::vector<float> n(3,0.0);
	float k = 0.0;
	float rho = 0.0;
	
	std::vector<float> c_dot(3,0.0);	//combined with c_dot_2 used to get c_ddot
	std::vector<float> c_dot_2(3,0.0);	//combined with c_dot used to get c_ddot
	std::vector<float> c_ddot(3,0.0);
	
	std::vector<float> c_1(3,0.0);
	std::vector<float> c_2(3,0.0);
	std::vector<float> c_3(3,0.0);
	
	float c_dot_norm = 0.0;
	std::vector<float> c_dot_cross_ddot(3,0.0);
	float c_dot_cross_ddot_norm = 0.0;
	float cross_i_hold = 0.0;
	float cross_j_hold = 0.0;
	float cross_k_hold = 0.0;
		
	c_1 = evaluate_path_point(i_u);
	c_2 = evaluate_path_point(i_u+dt);
	c_3 = evaluate_path_point((i_u+dt)+dt);
	for(int i = 0; i < 3; i++)
	{
		c_dot[i] = c_2[i] - c_1[i];
		c_dot_2[i] = c_3[i] - c_2[i];
		c_ddot[i] = c_dot_2[i] - c_dot[i];
	}
	ROS_INFO("motion info: @ %f, c_dot= (%f, %f) and c_ddot= (%f, %f)...", i_u, c_dot[0], c_dot[1], c_ddot[0], c_ddot[0]);	//debug
	
	//Evaluate c_dot cross c_ddot
	c_dot_cross_ddot[0] = ( (c_dot[1]*c_ddot[2]) - (c_dot[2]*c_ddot[1]) );
	c_dot_cross_ddot[1] = -( (c_dot[0]*c_ddot[2]) - (c_dot[2]*c_ddot[0]) );
	c_dot_cross_ddot[2] = ( (c_dot[0]*c_ddot[1]) - (c_dot[1]*c_ddot[0]) );
	
	//nessesary values
	c_dot_norm = sqrt( pow(c_dot[0],2) + pow(c_dot[1],2) + pow(c_dot[2],2));
	c_dot_cross_ddot_norm = sqrt( pow(c_dot_cross_ddot[0],2) + pow(c_dot_cross_ddot[1],2) + pow(c_dot_cross_ddot[2],2) );
	ROS_INFO("motion info: @ %f, c_dot_norm= %f and c_dot_cross_ddot_norm= %f...", i_u, c_dot_norm, c_dot_cross_ddot_norm);	//debug
	
	//unit tangent vector t
	t[0] = c_dot[0] / c_dot_norm;
	t[1] = c_dot[1] / c_dot_norm;
	t[2] = c_dot[2] / c_dot_norm;
	
	//unit binomial vector b
	b[0] = c_dot_cross_ddot[0] / c_dot_cross_ddot_norm;
	b[1] = c_dot_cross_ddot[1] / c_dot_cross_ddot_norm;
	b[2] = c_dot_cross_ddot[2] / c_dot_cross_ddot_norm;
	
	//unit normal
	n[0] = ( (b[1]*t[2]) - (b[2]*t[1]) );
	n[1] = -( (b[0]*t[2]) - (b[2]*t[0]) );
	n[2] = ( (b[0]*t[1]) - (b[1]*t[0]) );
	
	ROS_INFO("motion info: @ %f, t= (%f, %f, %f) and b= (%f, %f, %f) and n= (%f, %f, %f)...", i_u, t[0], t[1], t[2], b[0], b[1], b[2], n[0], n[1], n[2]);	//debug
	
	//use these to get info on velocity and acceleration, maybe use absolute 
	k = c_dot_cross_ddot_norm / pow(c_dot_norm, 3);
	rho = 1.0 / k;
	
	ROS_INFO("motion info: @ %f, k= %f, rho= %f...", k, rho); //debug
	
	ROS_INFO("motion_info completed..."); //debug
	
	return 0;
}

int obstacles_Callback()//const laser_obstacle_id::ObstaclesConstPtr& edges_xy_)
{
	ROS_INFO("obstacles_Callback() initiated...");	
	
	//Grid is variable controlled by d_hold::map_size (at begining in namespace def)
	float initial_position[2] = {6.7,4.2}; 
	float final_position[2] = {93.1,67.6};
	
	//Check if these points are within grid size
	if(initial_position[0] >= float (d_hold::map_size) || initial_position[0] >= float (d_hold::map_size))
		return -1; //Error: current position or goal position is on or out of grid boundary
	if(final_position[0] >= float (d_hold::map_size) || final_position[0] >= float (d_hold::map_size))
		return -1; //Error: current position or goal position is on or out of grid boundary
		
	d_hold::initial_final_positions[0][0] = initial_position[0];
	d_hold::initial_final_positions[0][1] = initial_position[1];
	d_hold::initial_final_positions[1][0] = final_position[0];
	d_hold::initial_final_positions[1][1] = final_position[1];
	
	//costmap_2d::Costmap2DROS obstacle_map;
	//From the incoming edges_xy_ create an obstacle map (edges_xy_ not yet implemented)
	std::vector<std::vector<int> > obstacle_map = generate_map(initial_position, final_position);
	ROS_INFO("Map generated...");
		
	//Displays  map in 1st quadrant of standard xy plane
/*	for(int k=d_hold::map_size-1; k >= 0; k--)
	{
		for(int i=0; i < d_hold::map_size; i++)
		{
			std::cout << obstacle_map[i][k] << " ";
		}
		std::cout << "\n";
	}
	ROS_INFO("Map displayed...");
*/

	//Pass the obstacle map, current pos, and goal position to functions which will 
	//effetively plan the path using our method. Result is an array of control points for all of the b-spline
	//segments from start to goal.
	std::vector<std::vector<float> > control_points = plan_path(obstacle_map, initial_position, final_position);
	
	//place the CP matrix in global d_hold::
	d_hold::final_CP = control_points;
	
	//To demonstrate result pass the array of control points to a function which will properly display
	//graph showing the obstacles and a plot of the curve.
//	plot_path(control_points);

	//perform curve dynamics analysis
	motion_info();
	
	return 1; //Succesful Completion
}

void laser_Callback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle node_handle;
	
	log_file.open("/home/marin/ros_workspace_electric/motion_planning/path_planner/log_file.txt");
	
	int sub_program_status = 0;
	
	laser_obstacle_id::Obstacles obstacles;
	
	ros::Publisher obstacles_pub = node_handle.advertise<laser_obstacle_id::Obstacles>("/laser_obstacles", 1000);	
//	ros::Subscriber obstacles_sub = node_handle.subscribe("/laser_obstacles", 500, obstacles_Callback);
//	ros::Subscriber laser_sub = node_handle.subscribe("/laser_scan", 500, laser_Callback);
	
	sub_program_status = obstacles_Callback();
	if(sub_program_status == 1)
		ROS_INFO("obstacles_Callback...Completed Succesfully...");
	if(sub_program_status == -1)
		ROS_INFO("obstacles_Callback...Failed!...");
	
	
	log_file.close();
	
	ROS_INFO("Program End Succesful.");

return 0;
}
