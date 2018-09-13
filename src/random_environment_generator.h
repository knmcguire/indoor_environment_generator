/*
 * random_environment_generator.h
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_
#define ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_


//OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>    // std::rotate
#include <vector>       // std::vector
#include <stdlib.h>     /* srand, rand */
#include <stdio.h>
#include <ctime>
#include <random>

#include <fstream>

//#include <chrono>


struct grid_element_status_t {
  bool is_agent_present;
  std::vector<std::vector<int> > circ_action;
  bool is_corridor_present;
};

class RandomEnvironmentGenerator
{
public:
  RandomEnvironmentGenerator();
  void Init(float arena_size_X, float arena_size_Y, float* pos_bot_x, float* pos_bot_y, int size_pos_bot, float* pos_tower);
  virtual void Reset();

  void initializeGrid();
  void initializeAgents();
  void findAgents();
  void decideNextAction(std::vector<int> current_bot_position);
  void setNextLocation(std::vector<int> current_bot_position);
  float getCorridorPercentage();
  void makeBinaryImageCorridors();
  void checkConnectivity();
  void checkConnectivityOpenCV();

  void makeBoundariesCorridors();
  void makeRooms();
  void makeRandomOpenings();
  void putBlocksInEnvironment();
  void putLinesInEnvironment();
  void generateEnvironment();
  void generateEnvironmentFromFile(std::string file_name);
  void getRobotPositions( float* pos_bot_x, float* pos_bot_y, int size_pos_bot, float* pos_tower);
  void dfs(int x, int y, int current_label);

  void RSSIMap();
  float getRSSITower(float x, float y, float heading);
  void visualize_grid(bool show_agents);


private:
  std::vector<std::vector<grid_element_status_t> > environment_grid;
  int environment_width;
  int environment_height;
  int grid_width;
  int grid_height;
  std::vector<float> tower_position;
  std::vector<std::vector<int> > initial_bot_positions;
  std::vector<std::vector<int> > current_agent_positions;
  float change_agent_gostraight;
  float wanted_corridor_percentage;
  cv::Mat bin_corridor_img;
  cv::Mat bin_corridor_img_large;
  cv::Mat corridor_contours_img;
  cv::Mat corridor_contours_img_save;
  cv::Mat  corridor_contours_img_dilate;

  float room_percentage;
  int total_boxes_generated;
  int amount_of_openings;
  bool environment_accepted;
  cv::RNG rng;
  std::vector<std::vector<int> > connectivity_labels;

  int it_box;

  bool corridors_are_connected;

  std::vector<std::vector<float> > RSSI_map;

  int corridor_size_meters;




};


#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_ */
