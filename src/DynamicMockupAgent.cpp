#include "../include/SingleAgent.h"
#include <iostream>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include "yaml-cpp/yaml.h"
#include<dqrobotics/utils/DQ_Geometry.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>

double min_dist = 1000;
double ave_dist = 0;
double ave_num = 0;
MatrixXd moving_object_circle;
MatrixXd moving_object_line;
MatrixXd moving_object_plane;
MatrixXd moving_object_cubic;
MatrixXd moving_object_cylinder;
int i_plane = 0;
int i_line= 0;
int i_cubic= 0;
int i_circle= 0;
int i_cylinder= 0;

int moving_steps = 0;
int moving_interval = 0;

void processFiles(const std::string& folderPath) {
    DIR* dir; // Directory pointer
    struct dirent* entry; // Represents a directory entry


    dir = opendir(folderPath.c_str());

	std::cout<< "folderPath: " << folderPath << std::endl;
    if (!dir) {
        std::cerr << "Unable to open directory: " << folderPath << std::endl;
        return;
    }
    std::cout << "Processing files in: " << folderPath << std::endl;

    // Read files in the directory
    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;

        // Skip "." and ".."
        if (fileName == "." || fileName == "..") {
            continue;
        }

        // Check if the file name contains "circle"
        if (fileName.find("circle_moving") != std::string::npos) {
			i_circle = i_circle + 1;
            // std::cout << "Found a file containing 'circle': " << fileName << std::endl;
        }

		if (fileName.find("line_moving") != std::string::npos) {
			i_line = i_line + 1;
            // std::cout << "Found a file containing 'line': " << fileName << std::endl;
        }

		if (fileName.find("plane_moving") != std::string::npos) {
			i_plane = i_plane + 1;
            // std::cout << "Found a file containing 'plane': " << fileName << std::endl;
        }

		if (fileName.find("cubic_moving") != std::string::npos) {
			i_cubic = i_cubic + 1;
            // std::cout << "Found a file containing 'cubic': " << fileName << std::endl;
        }

		if (fileName.find("cylinder_moving") != std::string::npos) {
			i_cylinder = i_cylinder + 1;
            // std::cout << "Found a file containing 'cylinder': " << fileName << std::endl;
        }
    }
	// std::cout << "999999999999999999999" << std::endl;
	moving_object_circle.resize(moving_steps, 3*(i_circle));
	for (size_t j = 0; j < i_circle; j++) {
		// std::cout << "1111111111111" << std::endl;
		std::string path_file = folderPath + "/circle_movingobject_" + std::to_string(j) + ".csv";
		// std::cout << "22222222222222222222" << std::endl;
		// std::cout << "path_file: " << path_file << std::endl;

		MatrixXd matrix_obj = load_csv3(path_file);
		// std::cout << "333333333333333333333" << std::endl;

		moving_object_circle.block(0, j*3, moving_steps, 3) = matrix_obj.transpose();
	}

	// std::cout << "qqqqqqqqqqqqqqqqq" << std::endl;

	moving_object_line.resize(moving_steps, 3*(i_line));
	for (size_t j = 0; j < i_line; j++) {
		std::string path_file = folderPath + "/line_movingobject_" + std::to_string(j) + ".csv";
		MatrixXd matrix_obj = load_csv3(path_file);
		// std::cout <<  "matrix_obj" << matrix_obj << std::endl;
		moving_object_line.block(0, j*3, moving_steps, 3) = matrix_obj.transpose();
	}
	moving_object_plane.resize(moving_steps, 3*(i_plane));
	for (size_t j = 0; j < i_plane; j++) {
		std::string path_file = folderPath + "/plane_movingobject_" + std::to_string(j) + ".csv";
		MatrixXd matrix_obj = load_csv3(path_file);
		moving_object_plane.block(0, j*3, moving_steps, 3) = matrix_obj.transpose();
	}
	moving_object_cubic.resize(moving_steps, 3*(i_cubic));
	for (size_t j = 0; j < i_cubic; j++) {
		std::string path_file = folderPath + "/cubic_movingobject_" + std::to_string(j) + ".csv";
		MatrixXd matrix_obj = load_csv3(path_file);
		moving_object_cubic.block(0, j*3, moving_steps, 3) = matrix_obj.transpose();
	}
	moving_object_cylinder.resize(moving_steps, 3*(i_cylinder));
	for (size_t j = 0; j < i_cylinder; j++) {
		std::string path_file = folderPath + "/cylinder_movingobject_" + std::to_string(j) + ".csv";
		MatrixXd matrix_obj = load_csv3(path_file);
		moving_object_cylinder.block(0, j*3, moving_steps, 3) = matrix_obj.transpose();
	}

    closedir(dir);
    std::cout << "All files processed." << std::endl;
}


int main()
{    
	bool fail = false;
	std::string file_name = "/home/yuhe/Desktop/project/3-GeometricPrimitives/cpp/config/TuningParams.yml";
	YAML::Node params_config = YAML::LoadFile(file_name);
	for (size_t iiiii = 1; iiiii < params_config["mockup_num"].as<double>(); iiiii++)
	{
			
		fail = false;	
		min_dist = 1000;
		ave_dist = 0;
		ave_num = 0;

		i_plane = 0;
		i_line= 0;
		i_cubic= 0;
		i_circle= 0;
		i_cylinder= 0;

		double dt_object_moving = 0.01 * 0.1; 

		moving_steps = params_config["moving_steps"].as<int>();
		moving_interval = params_config["moving_interval"].as<int>();

		// YAML file
		
		file_name = params_config["path"].as<std::string>() + std::to_string(iiiii) + "/GeomObstacles.yaml";
		YAML::Node obstacle_config = YAML::LoadFile(file_name);
		file_name = "/home/yuhe/Desktop/project/3-GeometricPrimitives/cpp/config/RobotPos.yml";
		YAML::Node robot_config = YAML::LoadFile(file_name);
		std::string order;

		// worksapce boundary
		double worksapce_rad = obstacle_config["worksapce_boundary"]["rad"].as<double>();
		std::array<double, 3> worksapce_boundary_array = obstacle_config["worksapce_boundary"]["pos"].as<std::array<double, 3>>();
		Eigen::Map<Eigen::Vector3d> worksapce_boundary(worksapce_boundary_array.data());
		VectorXd workspace_bound_info(4);
		workspace_bound_info.block(0,0,3,1) = worksapce_boundary;
		workspace_bound_info(3) = worksapce_rad;

		std::string save_path = params_config["path"].as<std::string>() + std::to_string(iiiii);

		// sphere obstacles
		int numCircles = obstacle_config["circle"].size();
		MatrixXd circle_object(3,numCircles);
		MatrixXd circle_rad(1,numCircles);

		for (size_t i = 0; i < numCircles; i++) {

			order = "circle" + std::to_string(i+1);
			std::array<double, 3> circle_array = obstacle_config["circle"][order]["pos"].as<std::array<double, 3>>();
			circle_object.col(i) = Eigen::Map<Eigen::VectorXd>(circle_array.data(), circle_array.size());
			std::array<double, 1> obs_rad_array = obstacle_config["circle"][order]["rad"].as<std::array<double, 1>>();
			circle_rad.col(i) = Eigen::Map<Eigen::VectorXd>(obs_rad_array.data(), obs_rad_array.size());
		}
		


		// line obstacles
		int numLines = obstacle_config["line"].size();
		MatrixXd line_object_start(3,numLines);
		MatrixXd line_object_end(3,numLines);
		for (size_t i = 0; i < numLines; i++) {

			order = "line" + std::to_string(i+1);
			std::array<double, 3> line_start_array = obstacle_config["line"][order]["start"].as<std::array<double, 3>>();
			line_object_start.col(i) = Eigen::Map<Eigen::VectorXd>(line_start_array.data(), line_start_array.size());  
			std::array<double, 3> line_end_array = obstacle_config["line"][order]["end"].as<std::array<double, 3>>();
			line_object_end.col(i) = Eigen::Map<Eigen::VectorXd>(line_end_array.data(), line_end_array.size()); 
		}

		// plane obstacles
		int numPlane = obstacle_config["plane"].size();
		MatrixXd plane_object_point1(3,numPlane);
		MatrixXd plane_object_point2(3,numPlane);
		MatrixXd plane_object_point3(3,numPlane);
		MatrixXd plane_object_point4(3,numPlane);
		for (size_t i = 0; i < numPlane; i++) {

			order = "plane" + std::to_string(i+1);
			std::array<double, 3> plane_point1_array = obstacle_config["plane"][order]["point1"].as<std::array<double, 3>>();
			plane_object_point1.col(i) = Eigen::Map<Eigen::VectorXd>(plane_point1_array.data(), plane_point1_array.size());  
			std::array<double, 3> plane_point2_array = obstacle_config["plane"][order]["point2"].as<std::array<double, 3>>();
			plane_object_point2.col(i) = Eigen::Map<Eigen::VectorXd>(plane_point2_array.data(), plane_point2_array.size()); 
			std::array<double, 3> plane_point3_array = obstacle_config["plane"][order]["point3"].as<std::array<double, 3>>();
			plane_object_point3.col(i) = Eigen::Map<Eigen::VectorXd>(plane_point3_array.data(), plane_point3_array.size()); 
			std::array<double, 3> plane_point4_array = obstacle_config["plane"][order]["point4"].as<std::array<double, 3>>();
			plane_object_point4.col(i) = Eigen::Map<Eigen::VectorXd>(plane_point4_array.data(), plane_point4_array.size());
		}

		// cubic obstacles
		int numCubic = obstacle_config["cubic"].size();
		MatrixXd cubic_object_point1(3,numCubic);
		MatrixXd cubic_object_point2(3,numCubic);
		MatrixXd cubic_object_point3(3,numCubic);
		MatrixXd cubic_object_point4(3,numCubic);
		MatrixXd cubic_object_point5(3,numCubic);
		MatrixXd cubic_object_point6(3,numCubic);
		MatrixXd cubic_object_point7(3,numCubic);
		MatrixXd cubic_object_point8(3,numCubic);
		for (size_t i = 0; i < numCubic; i++) {

			order = "cubic" + std::to_string(i+1);
			std::array<double, 3> cubic_point1_array = obstacle_config["cubic"][order]["point1"].as<std::array<double, 3>>();
			cubic_object_point1.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point1_array.data(), cubic_point1_array.size());  
			std::array<double, 3> cubic_point2_array = obstacle_config["cubic"][order]["point2"].as<std::array<double, 3>>();
			cubic_object_point2.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point2_array.data(), cubic_point2_array.size()); 
			std::array<double, 3> cubic_point3_array = obstacle_config["cubic"][order]["point3"].as<std::array<double, 3>>();
			cubic_object_point3.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point3_array.data(), cubic_point3_array.size()); 
			std::array<double, 3> cubic_point4_array = obstacle_config["cubic"][order]["point4"].as<std::array<double, 3>>();
			cubic_object_point4.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point4_array.data(), cubic_point4_array.size());
			std::array<double, 3> cubic_point5_array = obstacle_config["cubic"][order]["point5"].as<std::array<double, 3>>();
			cubic_object_point5.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point5_array.data(), cubic_point5_array.size());
			std::array<double, 3> cubic_point6_array = obstacle_config["cubic"][order]["point6"].as<std::array<double, 3>>();
			cubic_object_point6.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point6_array.data(), cubic_point6_array.size());
			std::array<double, 3> cubic_point7_array = obstacle_config["cubic"][order]["point7"].as<std::array<double, 3>>();
			cubic_object_point7.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point7_array.data(), cubic_point7_array.size());
			std::array<double, 3> cubic_point8_array = obstacle_config["cubic"][order]["point8"].as<std::array<double, 3>>();
			cubic_object_point8.col(i) = Eigen::Map<Eigen::VectorXd>(cubic_point8_array.data(), cubic_point8_array.size());
		}

		int numCylinder = obstacle_config["cylinder"].size();
		MatrixXd cylinder_object_start(3,numCylinder);
		MatrixXd cylinder_object_end(3,numCylinder);
		MatrixXd cylinder_object_rad(1,numCylinder);
		for (size_t i = 0; i < numCylinder; i++) {

			order = "cylinder" + std::to_string(i+1);
			std::array<double, 3> cylinder_object_start_array = obstacle_config["cylinder"][order]["obs_start"].as<std::array<double, 3>>();
			cylinder_object_start.col(i) = Eigen::Map<Eigen::VectorXd>(cylinder_object_start_array.data(), cylinder_object_start_array.size());  
			std::array<double, 3> cylinder_object_end_array = obstacle_config["cylinder"][order]["obs_end"].as<std::array<double, 3>>();
			cylinder_object_end.col(i) = Eigen::Map<Eigen::VectorXd>(cylinder_object_end_array.data(), cylinder_object_end_array.size()); 
			std::array<double, 1> cylinder_object_rad_array = obstacle_config["cylinder"][order]["obs_rad"].as<std::array<double, 1>>();
			cylinder_object_rad.col(i) = Eigen::Map<Eigen::VectorXd>(cylinder_object_rad_array.data(), cylinder_object_rad_array.size()); 
		}

		// load_csv
		// std::string folderPath = "./myFolder"; // Specify the folder path
		std::string moving_path = save_path + "/moving";
    	processFiles(moving_path);
		// abort();


		// initialization
		std::array<double, 3> start_position_array = robot_config["start"].as<std::array<double, 3>>();
		Eigen::Map<Eigen::Vector3d> start_position(start_position_array.data());

		std::array<double, 3> goal_position_array = robot_config["goal"].as<std::array<double, 3>>();
		Eigen::Map<Eigen::Vector3d> goal_position(goal_position_array.data());
		Vector3d robot_pos = start_position;
		Vector3d robot_vel(0, 0, 0);
		Vector3d acc(0,0,0);
		double robot_mass = 1.0;
		double dt = 0.01;
		double v_max = 0.04;

		// tuning parameters: k values
		VectorXd k_attr(1);
		k_attr << params_config["k_attr"].as<double>();
		VectorXd k_boundary(1);
		k_boundary << params_config["k_boundary"].as<double>();
		VectorXd k_circ(1);
		k_circ << params_config["k_circ"].as<double>();
		VectorXd k_line(1);
		k_line << params_config["k_line"].as<double>();
		VectorXd k_plane(1);
		k_plane << params_config["k_plane"].as<double>();
		VectorXd k_cubic(1);
		k_cubic << params_config["k_cubic"].as<double>();
		VectorXd k_cylinder(1);
		k_cylinder << params_config["k_cylinder"].as<double>();
		
		//VectorXd k_cube(0.1);
		//VectorXd k_cylinder(0.1);
		Vector3d force(0,0,0);
		Vector3d att_force(0,0,0);
		Vector3d circ_force(0,0,0);
		Vector3d lin_force(0,0,0);
		MatrixXd robot_pos_store;


		double e = 1111;
		double path_len = 0;
		Vector3d robot_pos_last;
		robot_pos_last = robot_pos;
		

		MatrixXd moving_circle;
		moving_circle = circle_object;
		moving_circle.setZero();
		MatrixXd moving_line;
		moving_line = line_object_start;
		moving_line.setZero();
		MatrixXd moving_plane;
		moving_plane = plane_object_point1;
		moving_plane.setZero();
		MatrixXd moving_cubic;
		moving_cubic = cubic_object_point1;
		moving_cubic.setZero();
		MatrixXd moving_cylinder;
		moving_cylinder = cylinder_object_start;
		moving_cylinder.setZero();


		// std::cout << "1111111111111" << std::endl;

		auto start = std::chrono::high_resolution_clock::now();
		int count = 0;
		while (e > 0.01) {

			force.setZero();

			// goal attraction force
			att_force = attr_force(goal_position, robot_pos, k_attr(0));
			force = force + att_force;
			
			// workspace boundary repulsive force
			force = force + boundary_force(worksapce_boundary, worksapce_rad, robot_pos, robot_vel, goal_position, k_boundary(0));
		// std::cout << "22222222222222222" << std::endl;

			// sphere repulsive force
			for (size_t i = 0; i < circle_object.cols(); i++) {
					// std::cout << "9999999999" << std::endl;

				if (i < i_circle)
				{
					// std::cout << "8888888888888888888" << std::endl;

					if (count < moving_steps*moving_interval && count % moving_interval == 0)
					{

						moving_circle.col(i) += moving_object_circle.block(count/moving_interval, i*3, 1, 3).transpose() * dt_object_moving;

					}
					
				}
					// std::cout << "777777777777777" << std::endl;

		
				force = force + sphere_force(circle_object.col(i) + moving_circle.col(i), circle_rad.col(i).value(), robot_pos, robot_vel, goal_position, k_circ(0));
			}

		// 	// // line repulsive force
			for (size_t i = 0; i < line_object_start.cols(); i++) {
				if (i < i_line)
				{

					if (count < moving_steps*moving_interval && count % moving_interval == 0)
					{
						moving_line.col(i) += moving_object_line.block(count/moving_interval, i*3, 1, 3).transpose() * dt_object_moving;
					}
				}	

				force = force + line_force(line_object_start.col(i) + moving_line.col(i), line_object_end.col(i) + moving_line.col(i), robot_pos, robot_vel, goal_position, k_line(0));
			} 

			// plane repulsive force
			for (size_t i = 0; i < plane_object_point1.cols(); i++) {

				if (i < i_plane)
				{
					if (count < moving_steps*moving_interval && count % moving_interval == 0)
					{
						moving_plane.col(i) += moving_object_plane.block(count/moving_interval, i*3, 1, 3).transpose() * dt_object_moving;
					}
				}

				force = force + plane_force(plane_object_point1.col(i) + moving_plane.col(i), plane_object_point2.col(i) + moving_plane.col(i), plane_object_point3.col(i) + moving_plane.col(i), 
										plane_object_point4.col(i) + moving_plane.col(i), robot_pos, robot_vel, goal_position, k_plane(0));
			} 

			// cubic repulsive force
			for (size_t i = 0; i < cubic_object_point1.cols(); i++) {

				if (i < i_cubic)
				{
					if (count < moving_steps*moving_interval && count % moving_interval == 0)
					{
						moving_cubic.col(i) += moving_object_cubic.block(count/moving_interval, i*3, 1, 3).transpose() * dt_object_moving;
					}
				}

				force = force + cubic_force(cubic_object_point1.col(i) + moving_cubic.col(i), cubic_object_point2.col(i) + moving_cubic.col(i), cubic_object_point3.col(i) + moving_cubic.col(i), 
										cubic_object_point4.col(i) + moving_cubic.col(i), cubic_object_point5.col(i) + moving_cubic.col(i), cubic_object_point6.col(i) + moving_cubic.col(i), 
										cubic_object_point7.col(i) + moving_cubic.col(i), cubic_object_point8.col(i) + moving_cubic.col(i), 
										robot_pos, robot_vel, goal_position, k_cubic(0));
			} 

			//  cylinder repulsive force
			for (size_t i = 0; i < cylinder_object_start.cols(); i++) {


				if (i < i_cylinder)
				{
					if (count < moving_steps*moving_interval && count % moving_interval == 0)
					{
						moving_cylinder.col(i) += moving_object_cylinder.block(count/moving_interval, i*3, 1, 3).transpose() * dt_object_moving;
					}
				}

				force = force + cylinder_force(cylinder_object_start.col(i) + moving_cylinder.col(i), cylinder_object_end.col(i)+ moving_cylinder.col(i), cylinder_object_rad.col(i).value(), robot_pos, robot_vel, goal_position, k_cylinder(0));
			} 



			// position update
			acc = force/robot_mass;
			robot_vel = robot_vel + acc * dt;  
			if (robot_vel.norm() > v_max) {

				robot_vel = robot_vel * v_max/robot_vel.norm();
			}
			if (e < 0.1) {

				robot_vel = robot_vel * (10*e);
			}
			
			robot_pos_last = robot_pos;
			robot_pos = robot_pos + 0.5 * acc * dt * dt + robot_vel * dt;
			path_len = path_len + (robot_pos - robot_pos_last).norm();

			// stop criteria
			e = (goal_position - robot_pos).norm();
			
			
			// trajectory store
			count = count + 1;
			robot_pos_store.conservativeResize(3, count);
			robot_pos_store.col(robot_pos_store.cols()-1) = robot_pos;
			if (count > params_config["limit_iter"].as<double>() || min_dist < 1.e-8)
			{
				e = -1;
			}

			// abort();
				
		}

		if (e == -1)
		{
			std::cout << "training failed" << std::endl;
			fail = true;
		}


		
		

		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

		std::cout << "execution time: " << duration << " seconds" << std::endl;
		std::cout << "count: " << count << std::endl;
		std::cout << "path_length: " << path_len << std::endl;
		std::cout << "trial: " << iiiii << std::endl;
		std::cout << "min_dist: " << min_dist << std::endl;
		std::cout << "ave_dist: " << ave_dist/ave_num << std::endl;
		std::cout << "moving object: " << i_circle + i_line + i_plane + i_cylinder + i_cubic << std::endl;

		


		VectorXd evaluation(8);
		evaluation(0) = duration;
		evaluation(1) = count;
		evaluation(2) = path_len;
		evaluation(3) = min_dist;
		evaluation(4) = ave_dist/ave_num;
		evaluation(5) = static_cast<int>(fail);
		evaluation(6) = numCircles + numLines + numPlane + numCubic + numCylinder;
		evaluation(7) = i_circle + i_line + i_plane + i_cylinder + i_cubic;


		std::cout << "numCircles: " << numCircles << std::endl;
		std::cout << "numLines : " << numLines  << std::endl;
		std::cout << "numPlane : " << numPlane  << std::endl;
		std::cout << "numCubic : " << numCubic  << std::endl;
		std::cout << "numCylinder : " << numCylinder  << std::endl;







		
		std::cout << "object numbeer: " << numCircles + numLines + numPlane + numCubic + numCylinder << std::endl;

		
		write2csv(save_path + "/trajectory.csv", robot_pos_store);
		write2csv(save_path + "/workspaceboundary.csv", workspace_bound_info);
		write2csv(save_path + "/sphere.csv", circle_object);
		write2csv(save_path + "/sphererad.csv", circle_rad);
		write2csv(save_path + "/linestart.csv", line_object_start);
		write2csv(save_path + "/lineend.csv", line_object_end);
		write2csv(save_path + "/planepoint1.csv", plane_object_point1);
		write2csv(save_path + "/planepoint2.csv", plane_object_point2);
		write2csv(save_path + "/planepoint3.csv", plane_object_point3);
		write2csv(save_path + "/planepoint4.csv", plane_object_point4);
		write2csv(save_path + "/cubicpoint1.csv", cubic_object_point1);
		write2csv(save_path + "/cubicpoint2.csv", cubic_object_point2);
		write2csv(save_path + "/cubicpoint3.csv", cubic_object_point3);
		write2csv(save_path + "/cubicpoint4.csv", cubic_object_point4);
		write2csv(save_path + "/cubicpoint5.csv", cubic_object_point5);
		write2csv(save_path + "/cubicpoint6.csv", cubic_object_point6);
		write2csv(save_path + "/cubicpoint7.csv", cubic_object_point7);
		write2csv(save_path + "/cubicpoint8.csv", cubic_object_point8);
		write2csv(save_path + "/cylinderstart.csv", cylinder_object_start);
		write2csv(save_path + "/cylinderend.csv", cylinder_object_end);
		write2csv(save_path + "/cylinderrad.csv", cylinder_object_rad);
		write2csv(save_path + "/evaluation.csv", evaluation);


		if (fail)
		{
			std::cout << "Training Fail" << std::endl;
		}
		else{
			std::cout << "Training Success" << std::endl;
		}

		std::cout << "-------------------------------------------------------------" << std::endl;
		
		
	}
	return 0;

}
