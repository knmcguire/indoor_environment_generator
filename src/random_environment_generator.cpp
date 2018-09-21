/*
 * random_environment_generator.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#include "random_environment_generator.h"
//#include <chrono>


//#define ACCEPT_ENVIRONMENT

using namespace std;
using namespace cv;

#define EFFICIENT_ENVIRONMENT true
#define ACCEPT_ENVIRONMENT true

RandomEnvironmentGenerator::RandomEnvironmentGenerator() :
										  environment_width(10),
										  environment_height(10),
										  change_agent_gostraight(0.8f),
										  wanted_corridor_percentage(0.35f),
										  room_percentage(0.4f),
										  total_boxes_generated(0),
										  amount_of_openings(5),
										  environment_accepted(false),
										  corridor_size_meters(2),
										  is_initialized(false){}



void RandomEnvironmentGenerator::visualize_grid(bool show_agents)
{
	for(int it_x = 0; it_x<grid_width; it_x++)
	{
		for (int it_y = 0; it_y<grid_height; it_y++)
		{
			if(show_agents)
				cout<<environment_grid.at(it_x).at(it_y).is_agent_present<<" ";
			else
				cout<<environment_grid.at(it_x).at(it_y).is_corridor_present<<" ";

		}
		cout<<" "<<endl;
	}
}

void RandomEnvironmentGenerator::getRobotPositions( float* pos_bot_x, float* pos_bot_y, int size_pos_bot, float* pos_tower)
{
	cout<<"...Robot positions"<<endl;



	// put corridor agents on start position of robots in grid coordinates
	for (int i = 0; i<size_pos_bot;i++)
	{
		int pos_bot_x_grid = (int)(roundf(pos_bot_x[i]/(float)corridor_size_meters)) + (grid_width-1)/2;
		int pos_bot_y_grid = (int)(roundf(pos_bot_y[i]/(float)corridor_size_meters)) + (grid_height-1)/2;

		cout<<"pos grid "<<pos_bot_x_grid<<" "<<pos_bot_y_grid<<endl;


		vector<int> initial_bot_position{0,0};
		initial_bot_position.at(0)=pos_bot_x_grid;
		initial_bot_position.at(1)=pos_bot_y_grid;
		initial_bot_positions.push_back(initial_bot_position);
	}

	// put on corridor agent on tower position
	vector<int> initial_tower_position{0,0};

    // save tower position for later use
	tower_position.at(0) = pos_tower[0];
	tower_position.at(1) = pos_tower[1];

	// translate tower coordinates to grid coordinates
	int pos_tower_x_grid = pos_tower[0]/corridor_size_meters + (grid_width-1)/2;
	int pos_tower_y_grid = pos_tower[1]/corridor_size_meters + (grid_height-1)/2;

	initial_tower_position.at(0)=pos_tower_x_grid;
	initial_tower_position.at(1)=pos_tower_y_grid;
	initial_bot_positions.push_back(initial_tower_position);

	is_initialized = true;
}

void RandomEnvironmentGenerator::Init(float arena_size_X, float arena_size_Y, float* pos_bot_x, float* pos_bot_y, int size_pos_bot, float* pos_tower)
{
	cout<<"Init..."<<endl;


	environment_accepted =false;

	// Save size arena in meters
	environment_width = (int)(arena_size_X);
	environment_height=(int)(arena_size_Y);

	// get grid width with corridor size in meters (default 2)
	grid_width = environment_width/corridor_size_meters+1;
	grid_height = environment_height/corridor_size_meters+1;

	// Initialize some parameters
	it_box = 0;
	tower_position.resize(2);

	// Initialize the generator
	getRobotPositions( pos_bot_x,  pos_bot_y,  size_pos_bot,  pos_tower);
	initializeGrid();
	initializeAgents();

	bin_corridor_img = Mat::zeros(environment_width, environment_height, CV_8UC1);
	corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);


	RSSI_map.resize(10*environment_width);
	for (int it = 0; it < 10*environment_width; it++) {
		RSSI_map[it].resize(10*environment_height);
	}
}



void RandomEnvironmentGenerator::Reset()
{
	generateEnvironment();
}


void RandomEnvironmentGenerator::generateEnvironment(void)
{

	std::cout<<"Start generate environment!"<<std::endl;

	// Boolean for connectivity check
	corridors_are_connected = false;

	// Random seed for corridor agents
	rng = cv::getTickCount();


	// corridor itrations in total
	int corridor_iterations = 100;

	visualize_grid(true);


	while(!environment_accepted){
		while (!corridors_are_connected) {

			initializeGrid();
			initializeAgents();

				// For a number of iterations
			for (int it_total = 0; it_total < corridor_iterations; it_total++) {

				// Find agent position
				findAgents();

				// At all agent positions size, decide what to do next next

				for (int it = 0; it < current_agent_positions.size(); it++) {

					decideNextAction(current_agent_positions.at(it));
					setNextLocation(current_agent_positions.at(it));
				}

				// if corridor percentage is reached, terminate
				cout<<getCorridorPercentage()<<endl;
				if (getCorridorPercentage() > wanted_corridor_percentage) {
					break;
				}

			}

			// Check the connectivity of the grid
			checkConnectivity();
			if(!corridors_are_connected)
				{
				cout<<"corridors are not connected!!"<<endl;
				rng = cv::getTickCount();

				}

		}

		visualize_grid(false);


		bin_corridor_img = Mat::zeros(grid_width, grid_height, CV_8UC1);

		makeBinaryImageCorridors();

		bin_corridor_img_large = Mat::zeros(environment_width * 10, environment_height * 10, CV_8UC1);
		corridor_contours_img = Mat::zeros(bin_corridor_img_large.size(), CV_8UC1);

		makeBoundariesCorridors();


		makeRooms();
		makeRandomOpenings();



		cv::Rect border(cv::Point(0, 0), corridor_contours_img.size());

		rectangle(corridor_contours_img, border, Scalar(255), 3);
		cv::imwrite("environment.png",corridor_contours_img);



#ifdef ACCEPT_ENVIRONMENT
		Mat corridor_contours_img_extra;

		cvtColor(corridor_contours_img.clone(),corridor_contours_img_extra,COLOR_GRAY2RGB);

		for(int it = 0;it<initial_bot_positions.size();it++)
		{
			Point tower_pos {(int)(10*corridor_size_meters*initial_bot_positions.at(it).at(0) ),(int)(10*corridor_size_meters*initial_bot_positions.at(it).at(1))};

			if(it==initial_bot_positions.size()-1)
			{

				circle(corridor_contours_img_extra,tower_pos,3,(0,0,255),-1);
			}
			else
			{
				circle(corridor_contours_img_extra,tower_pos,3,Scalar(0,255,0),-1);

			}
		}
		namedWindow( "Environment",WINDOW_NORMAL );// Create a window for display.
		imshow( "Environment", corridor_contours_img_extra );                   // Show our image inside it.
		char key = (char)waitKey(0);
		if(key=='y')
		{
			environment_accepted=true;
			break;
		}else
		{
			rng = cv::getTickCount();
			corridors_are_connected = false;
			environment_accepted=false;

		}
#else
		environment_accepted=true;
#endif

	}


	RSSIMap();


#if EFFICIENT_ENVIRONMENT
	putLinesInEnvironment();
	putBlocksInEnvironment();
#else
	putBlocksInEnvironment();
#endif
}



void RandomEnvironmentGenerator::initializeGrid(void)
{
	cout<<"...initialize Grid"<<endl;

	vector<vector<int> > circ_action_init {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
	//Resizing environment grid
	environment_grid.resize(grid_width);
	for (int it = 0; it < grid_width; it++) {
		environment_grid[it].resize(grid_height);
	}

	//TODO: get this like trajectory_loop_function does
	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			environment_grid.at(itx).at(ity).is_corridor_present = false;
			environment_grid.at(itx).at(ity).is_agent_present = false;
			environment_grid.at(itx).at(ity).circ_action = circ_action_init;
		}
	}
}

void RandomEnvironmentGenerator::initializeAgents(void)
{
	cout<<"...initialize Agents"<<endl;

	// Initialize some parameters
	vector<vector<int> > circ_action_init{{0, 1}, {1, 0}, {0, -1}, { -1, 0}};
	current_agent_positions.resize(2);

	// initial robot positions, place agent where they are
	for (int it = 0; it < initial_bot_positions.size(); it++) {
		environment_grid.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0)).is_agent_present = true;

		std::rotate(circ_action_init.begin(), circ_action_init.begin() + std::rand()%4, circ_action_init.end());
		environment_grid.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0)).circ_action = circ_action_init;

	}
}

void RandomEnvironmentGenerator::findAgents(void)
{


	current_agent_positions.clear();

	int k = 0;
	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			if (environment_grid.at(itx).at(ity).is_agent_present) {
				current_agent_positions.resize(k + 1);
				current_agent_positions.at(k).resize(2);
				current_agent_positions.at(k).at(0) = itx;
				current_agent_positions.at(k).at(1) = ity;
				k++;
			}
		}
	}

}

void RandomEnvironmentGenerator::decideNextAction(std::vector<int> current_bot_position)
{
	float random_percentage = rng.uniform(0.0f,1.0f);
	float percentage_rest = 1.0f - change_agent_gostraight;

	vector<vector<int> >circ_action_temp = environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action;

	string state;
	if (random_percentage <= change_agent_gostraight) {
		state = "GO_STRAIGHT";
	} else if (random_percentage > change_agent_gostraight &&
			random_percentage <= change_agent_gostraight + percentage_rest / 2.0f) {
		state = "GO_LEFT";
		std::rotate(circ_action_temp.begin(), circ_action_temp.begin() + 1, circ_action_temp.end());

	} else if (random_percentage > change_agent_gostraight + percentage_rest / 2.0f &&
			random_percentage <= 1) {
		state = "GO_RIGHT";
		std::rotate(circ_action_temp.rbegin(), circ_action_temp.rbegin() + 1, circ_action_temp.rend());
	}

	environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action = circ_action_temp;


}

int mod(int a, int b)
{ return (a % b + b) % b; }

void RandomEnvironmentGenerator::setNextLocation(std::vector<int> current_bot_position)
{

	vector<vector<int> >circ_action_temp = environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action;
	vector<int> next_location{current_bot_position.at(0) + circ_action_temp.at(0).at(0), current_bot_position.at(1) + circ_action_temp.at(0).at(1)};




	vector<int> next_location_corrected {mod(next_location.at(0), grid_width), mod(next_location.at(1), grid_height)};


	environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).is_agent_present = false;
	environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).is_corridor_present = true;
	environment_grid.at(next_location_corrected.at(0)).at(next_location_corrected.at(1)).is_agent_present = true;
	environment_grid.at(next_location_corrected.at(0)).at(next_location_corrected.at(1)).circ_action = circ_action_temp;



}
float RandomEnvironmentGenerator::getCorridorPercentage()
{

	int count_corridor = 0;
	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			if (environment_grid.at(itx).at(ity).is_corridor_present) {
				count_corridor++;
			}
		}
	}

	return (float)count_corridor / (float)(grid_width * grid_height);
}

void RandomEnvironmentGenerator::makeBinaryImageCorridors()
{

	cout<<"Make image of grid..."<<endl;
	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			// cout << environment_grid.at(itx).at(ity).is_corridor_present << " ";
			if (environment_grid.at(itx).at(ity).is_corridor_present) {
				bin_corridor_img.at<uchar>(itx, ity) = 255;
			}
		}
	}
}


void RandomEnvironmentGenerator::checkConnectivityOpenCV()
{
	Mat labels;
	connectedComponents(bin_corridor_img, labels, 4, CV_16U);
	ushort label_at_first_location = labels.at<ushort>(initial_bot_positions.at(0).at(0), initial_bot_positions.at(0).at(1));

	for (int it = 1; it < initial_bot_positions.size(); it++) {
		ushort label_at_second_location = labels.at<ushort>(initial_bot_positions.at(it).at(0), initial_bot_positions.at(it).at(1));
		if (label_at_first_location == label_at_second_location) {
			corridors_are_connected = true;
		} else {
			corridors_are_connected = false;
			break;
		}
	}

}


const int dx[] = {+1, 0, -1, 0};
const int dy[] = {0, +1, 0, -1};
void RandomEnvironmentGenerator::dfs(int x, int y, int current_label) {
	if (x < 0 || x == grid_width) return; // out of bounds
	if (y < 0 || y == grid_height) return; // out of bounds
	if (connectivity_labels.at(x).at(y) || !environment_grid.at(x).at(y).is_corridor_present) return; // already labeled or not marked with 1 in m

	// mark the current cell
	connectivity_labels.at(x).at(y) = current_label;

	// recursively mark the neighbors
	for (int direction = 0; direction < 4; ++direction)
		dfs(x + dx[direction], y + dy[direction], current_label);
}

void RandomEnvironmentGenerator::checkConnectivity()
{



	cout<<"Check Connectivity..."<<endl;


	vector<int> linked;
	//Resizing connectivity_labels grid
	connectivity_labels.resize(grid_width);
	for (int it = 0; it < grid_width; it++) {
		connectivity_labels[it].resize(grid_height);
	}

	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			connectivity_labels.at(itx).at(ity)=0;
		}
	}


	vector<int> neighbors_labels;
	int next_label = 0;

	int component = 0;
	for (int itx = 0; itx < grid_width; itx++) {
		for (int ity = 0; ity < grid_height; ity++) {
			if (!connectivity_labels.at(itx).at(ity)&&environment_grid.at(itx).at(ity).is_corridor_present)
			{
				dfs(itx, ity, ++component);
			}
		}
	}

	/*
  for (int itx = 0; itx < environment_width; itx++) {
     for (int ity = 0; ity < environment_height; ity++) {
         cout<<connectivity_labels.at(itx).at(ity)<<" ";
     }
     cout<<" "<<endl;
   }
	 */

	int label_at_first_location = connectivity_labels.at(initial_bot_positions.at(0).at(1)).at(initial_bot_positions.at(0).at(0));

	for (int it = 1; it < initial_bot_positions.size(); it++) {
		int label_at_second_location = connectivity_labels.at(initial_bot_positions.at(it).at(1)).at(initial_bot_positions.at(it).at(0));
		if (label_at_first_location == label_at_second_location) {
			corridors_are_connected = true;
		} else {
			corridors_are_connected = false;
			break;
		}
	}
}

void RandomEnvironmentGenerator::makeBoundariesCorridors()
{

	cout<<"Turn corridors into walls"<<endl;

	// Resize Corridor image to fit environment size * resolution (default = 10 per m)
	resize(bin_corridor_img, bin_corridor_img_large, bin_corridor_img_large.size(), 0, 0, INTER_NEAREST);

    // Dilate corridors image (??)
	dilate(bin_corridor_img_large, bin_corridor_img_large, Mat(), Point(-1, -1), 2, 1, 1);
	//dilate(bin_corridor_img_large, bin_corridor_img_large, Mat(), Point(-1, -1), 2, 1, 1);



	// Create a large block around the robot's first position.
	for(int it =0; it<initial_bot_positions.size();it++)
		//Scalar(it*50+50)
		rectangle(bin_corridor_img_large, Point(initial_bot_positions.at(it).at(0)*10*corridor_size_meters - 20, initial_bot_positions.at(it).at(1)*10*corridor_size_meters- 20),
				Point(initial_bot_positions.at(it).at(0)*10*corridor_size_meters + 20, initial_bot_positions.at(it).at(1)*10*corridor_size_meters + 20), Scalar(255), CV_FILLED, 8, 0);

	// Find contours around corridors
	vector<vector<Point> > contours_coordinates;
	Mat hierarchy;

	findContours(bin_corridor_img_large, contours_coordinates, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	Scalar color = Scalar(255, 255, 255);

	// draw corridor contours
	for (int i = 0; i < contours_coordinates.size(); i++) {

		drawContours(corridor_contours_img, contours_coordinates, i, color, 1  , cv::LINE_4, hierarchy, 0);
	}

#if !EFFICIENT_ENVIRONMENT
	dilate(corridor_contours_img, corridor_contours_img, Mat(), Point(-1, -1), 2, 1, 1);
#endif

}

void RandomEnvironmentGenerator::makeRooms()
{

	cout<<"make rooms"<<endl;
	Mat bin_corridor_img_large_inv = Mat::zeros(environment_width * 10, environment_height * 10, CV_8UC1);
	bitwise_not(bin_corridor_img_large,bin_corridor_img_large_inv);
	corridor_contours_img.copyTo(corridor_contours_img_save);
	vector<vector<Point> > contours_coordinates;
	Mat hierarchy;

	findContours(bin_corridor_img_large_inv, contours_coordinates, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	for(int i = 0; i <contours_coordinates.size();i++ )
	{
		Rect boundRect=boundingRect(contours_coordinates.at(i));
		for(int itx = boundRect.x; itx<(boundRect.x + boundRect.width); itx++)
		{
			for(int ity = boundRect.y; ity<(boundRect.y + boundRect.height); ity++)
			{

				vector<int> coord_mod_rooms {(itx-boundRect.x) % (int)(boundRect.width /2), (ity-boundRect.y) % (int)(boundRect.height /2)};
				if( boundRect.width<(float)environment_width * 10 *room_percentage)
					coord_mod_rooms.at(0) = 1;
				if(boundRect.height<(float)environment_height * 10 *room_percentage)
					coord_mod_rooms.at(1) = 1;

				if ((coord_mod_rooms.at(0) == 0 || coord_mod_rooms.at(1) == 0))
					if (bin_corridor_img_large.at<uchar>(ity, itx) == 0) {
#if EFFICIENT_ENVIRONMENT
						rectangle(corridor_contours_img, Point(itx, ity ), Point(itx , ity ), Scalar(255), 1, 8, 0);

#else
						rectangle(corridor_contours_img, Point(itx-1, ity-1 ), Point(itx+1 , ity+1 ), Scalar(255), 1, 8, 0);

#endif
					}
			}
		}
	}


}

void RandomEnvironmentGenerator::makeRandomOpenings()
{

	cout<<"make openings"<<endl;

	RNG rng(cv::getTickCount());
	int half_size_openings = 13;
	int erosion_size = 1;
	Mat element = getStructuringElement(cv::MORPH_CROSS,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size) );
	Mat corridor_contours_img_temp = Mat::zeros(corridor_contours_img.size(), CV_8UC1);


#if EFFICIENT_ENVIRONMENT
	corridor_contours_img_save.copyTo(corridor_contours_img_temp);
#else
	erode(corridor_contours_img_save, corridor_contours_img_temp, element, Point(-1, -1), 2, 1, 1);
#endif

	for (int itx = 0; itx < environment_width * 10; itx++) {
		for (int ity = 0; ity < environment_height * 10; ity++) {
			if(corridor_contours_img_temp.at<uchar>(ity, itx)==255&&  rng.uniform(1, 1000)<amount_of_openings
					&&itx!=0&&itx!=environment_width* 20
					&&itx!=0&&itx!=environment_height* 20)
			{
				corridor_contours_img.at<uchar>(ity, itx)==0;
				rectangle(corridor_contours_img, Point(itx - half_size_openings, ity - half_size_openings), Point(itx + half_size_openings, ity + half_size_openings), Scalar(0), CV_FILLED, 8, 0);

			}
		}
	}


}

void RandomEnvironmentGenerator::putBlocksInEnvironment()
{

// ToDo, see if this is still necessary for gazebo

}


void RandomEnvironmentGenerator::putLinesInEnvironment()
{

     cout<<"put lines in model"<<endl;

   //Open sdf file for environment and add starting lines
  ofstream myfile;
  myfile.open ("/home/knmcguire/Software/catkin_ws/src/indoor_environment_generator/models/random_generated_environment/test_model.sdf");
  myfile << "<?xml version='1.0'?>\n <sdf version='1.4'> \n <model name='crazy_maze'> \n <static>1</static>\n";


  ofstream lines_file;
  lines_file.open("environment_lines.txt");


  // Initialize box_name string
  std::ostringstream box_name;

  // indicate number of paramid levels for the hough line detection
  int num_of_iterations = 3;

  // Go throught multiple levels of hougline detection
  for (int it_total= 0; it_total<num_of_iterations;it_total++)
  {
	  // Resize the environment image per layer
	  Mat corridor_contours_img_lines = Mat::zeros(environment_width * 10*(it_total+1), environment_height * 10*(it_total+1), CV_8UC1);
	  resize(corridor_contours_img, corridor_contours_img_lines, corridor_contours_img_lines.size(), 0, 0, INTER_NEAREST);

	  // Detect houghlines in image
	  vector<Vec4i> lines;
	  HoughLinesP(corridor_contours_img_lines, lines, 1, CV_PI/180*45, 20, num_of_iterations, 0 );

	  // Remove lines from original image to see how many of the walls were handled (for debugging)
	  Mat img_lines = corridor_contours_img.clone();

	  for( size_t i = 0; i < lines.size(); i++ )
	  {

		  Vec4i l = lines[i];
		  int line_point_x[2] = {l[0]/(it_total+1),l[2]/(it_total+1)};
		  int line_point_y[2] = {l[1]/(it_total+1),l[3]/(it_total+1)};

		  line( img_lines, Point(line_point_x[0], line_point_y[0]), Point(line_point_x[1],line_point_y[1]), Scalar(100,100,100), 3, CV_AA);

		  // Remove lines from corridor contour image to later save for blocks placement
		  line(corridor_contours_img,Point(line_point_x[0], line_point_y[0]), Point(line_point_x[1],line_point_y[1]), Scalar(0,0,0), 1, CV_AA);
	  }

		/*namedWindow( "Environment",WINDOW_NORMAL );
		imshow( "Environment", corridor_contours_img );
		char key = (char)waitKey(0);
	*/

	  // Put the detected lines in .sdf file for gazebo
	  for( size_t i = 0; i < lines.size(); i++ )
	  {

		  // give the box a unique name
		  box_name.str("");
		  box_name << "box" << (it_box);

		  // Transform the hough line coordinates to gazebo coordinates
		  Vec4i l = lines[i];
		  double gazebo_coordinates_x = (double)((l[0]+l[2])/(2*(it_total+1)) - ((double)environment_width * 10.0f / 2.0f)) / (10.0f);
		  double gazebo_coordinates_y = (double)((l[1]+l[3])/(2*(it_total+1)) - ((double)environment_height * 10.0f / 2.0f)) / (10.0f);
		  vector<double> gazebo_coordinates{gazebo_coordinates_x,gazebo_coordinates_y };

		  // Determine the box length and orientation
		  double box_lenght = (sqrt(pow((double)(l[2]-l[0]),2.0f)+pow((double)(l[3]-l[1]),2.0f))+2)/(10.0f*(it_total+1));
		  double box_orientation = (atan2(l[3]-l[1],l[2]-l[0]));

		  lines_file<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<"\n";

		  // Write box entities in environment file
		  myfile<<" <link name='"<<box_name.str()<<"'>\n";
		  myfile<<"   <pose>"<<to_string(gazebo_coordinates.at(0))<<" "<<to_string(gazebo_coordinates.at(1))<<" 0 0 0 "<<to_string(box_orientation)<<" </pose>\n";
		  myfile<<"   <collision name='"<<box_name.str()<<"_Collision'>\n";
		  myfile<<"     <pose>0 0 0.25 0 0 0</pose>\n";
		  myfile<<"   	<geometry>\n";
		  myfile<<"      		<box>\n";
		  myfile<<"         		<size>"<<to_string(box_lenght)<<" 0.1 1.0</size>\n";
		  myfile<<"      		</box>\n";
		  myfile<<"   	</geometry>\n";
		  myfile<<"   </collision>\n";
		  myfile<<"   <visual name='"<<box_name.str()<<"_Visual'>\n";
		  myfile<<"     <pose>0 0 0.25 0 0 0</pose>\n";
		  myfile<<"   	<geometry>\n";
		  myfile<<"      		<box>\n";
		  myfile<<"         		<size>"<<to_string(box_lenght)<<" 0.1 1.0</size>\n";
		  myfile<<"      		</box>\n";
		  myfile<<"   	</geometry>\n";
		  myfile<<"   	<material>\n";
		  myfile<<"   		<script>\n";
		  myfile<<"       		<uri>file://media/materials/scripts/gazebo.material</uri>\n";
		  myfile<<"           	 <name>Gazebo/Grey</name>\n";
		  myfile<<"			</script>\n";
		  myfile<<"		</material>\n";
		  myfile<<" 	</visual>\n";
		  myfile<<"</link>";
		  myfile<<"\n\n";

		  // count how many boxes were entered
		  it_box++;

	  }
  }


  // Save the environment file and save how many elements it took
  myfile<<"  </model> \n </sdf>";
  myfile.close();
  total_boxes_generated=it_box-1;
  lines_file.close();
}

static float wraptopi(float number)
{

	if(number>(float)M_PI)
		return (number-(float)(2*M_PI));
	else if(number< (float)(-1*M_PI))
		return (number+(float)(2*M_PI));
	else
		return (number);

}

void RandomEnvironmentGenerator::RSSIMap()
{
	cout<<"make rssimap"<<endl;

	//float p[3] = {-0.08685 ,5.205e-17,1.329};

	// Prepare the check image for debugging
	Mat check_slice_rssi_map = Mat::zeros(corridor_contours_img.size(), CV_8UC1);


	// Retrieve position tower
	int index_tower = initial_bot_positions.size()-1;
	Point tower_pos_img {(int)(10*initial_bot_positions.at(index_tower).at(0)),(int)(10*initial_bot_positions.at(index_tower).at(1))};

	// Initialize look up table for rssi measurements


	// Dilate original corridor image
	//Mat corridor_contours_img_dilate;
	int dilation_size = 5;
	Mat kernel = getStructuringElement(  MORPH_ELLIPSE,
			Size( 2*dilation_size + 1, 2*dilation_size+1 ),
			Point( dilation_size, dilation_size ) );
	dilate(corridor_contours_img,corridor_contours_img_dilate,kernel,Point(-1,-1),2,1,1);


	float heading[120];
	for(int it = 0; it<120;it++)
		heading[it] = -M_PI + (float)it*(2*M_PI)/120;

	int debug_heading = 0;

	// go through each location in the map
	for(int it_x = 0;it_x<10*environment_width;it_x++)
	{
		for(int it_y = 0;it_y<10*environment_height;it_y++)
		{


			// Calculate distance to beacon
			//float distance = (float)sqrt(pow(tower_pos_img.x-it_x,2)+pow(tower_pos_img.y-it_y,2))/10.0f;


			// Go to bearing difference
				// calculate the bearing and the adjusted distance because of it
				//float bearing = wraptopi((float)atan2((float)(tower_pos_img.y-it_y)/10.0f,(float)(tower_pos_img.x-it_x)/10.0f)-heading[it_w]);
				//float distance_bearing = distance*(bearing*bearing*p[0]+bearing*p[1]+p[2]);

				//cout<<"check "<<it_x<<"  "<<it_y<<endl;
				// If location is not near a obstacle, then just input distance
				if(corridor_contours_img_dilate.at<uchar>(it_y,it_x) < 200)
				{
					// small std in freespace
					std::random_device rd;
					std::mt19937 e2(rd());
					std::normal_distribution<float> dist(0.0, 0.20f);
					float noise = dist(e2);

					//RSSI_map.at(it_x).at(it_y) = distance_bearing;
					RSSI_map.at(it_x).at(it_y) = noise;


				}else
				{
					// Normal distribution with a high std around obstacles
					std::random_device rd;
					std::mt19937 e2(rd());
					std::normal_distribution<float> dist(0, 2.0f);
					float noise = dist(e2);
						RSSI_map.at(it_x).at(it_y) = noise;
				}

				// Save a slice of the rssi map for debugging
				//if(it_w == debug_heading)
					check_slice_rssi_map.at<uchar>(it_x,it_y)=(uchar)RSSI_map.at(it_x).at(it_y)*10;




		}
	}


	cout<<"make copy"<<endl;
    std::fstream of("Map.txt", std::ios::out);
    if (of.is_open())
     {
        of<<10*environment_width<<" "<<10*environment_height<<"\n";

    	   for (int it_x = 0; it_x < 10*environment_width; ++it_x)
    	    {
    	        for (int it_y = 0; it_y < 10*environment_height; ++it_y)
    	        {
    	        	of << RSSI_map.at(it_x).at(it_y)<<" ";
    	        }
    	        of<<"\n";
    	    }
         of.close();
     }




	/*addWeighted(check_slice_rssi_map,0.8,corridor_contours_img,0.2,0.0,check_slice_rssi_map);
	namedWindow("display",WINDOW_NORMAL);// Create a window for display.

	//normalize(check_slice_rssi_map,check_slice_rssi_map,255,0,NORM_MINMAX);
	imshow("display", check_slice_rssi_map );                   // Show our image inside it.
	waitKey(0);*/


	//display slice for debugging

}

float RandomEnvironmentGenerator::getRSSITower(float x, float y, float heading)
{

	if(!is_initialized)
	{
	ifstream myReadFile;
	    myReadFile.open("Map.txt");

	    std::vector<std::vector<float> > RSSI_map_temp;


		cout<<"get copy"<<endl;


		float tmpValue;
		int map_width, map_height;
	    myReadFile  >> tmpValue;
	    map_width = (int)tmpValue;
	    myReadFile  >> tmpValue;
	    map_height = (int)tmpValue;

	    environment_width = map_width/10;
	    environment_height = map_height/10;

	    cout<<"size "<<environment_width<<" "<<environment_height<<endl;



	    RSSI_map.resize(map_width);
		for (int it = 0; it < map_width; it++) {
			RSSI_map[it].resize(map_height);
		}


		if (!myReadFile.eof()) {
		    for(int it_x = 0; it_x < map_width; it_x++){
		        float tmpValueF;

		        for (int it_y = 0; it_y < map_height; it_y++){
		            myReadFile  >> tmpValueF;
		            RSSI_map.at(it_x).at(it_y)= (float)tmpValueF;
		        }
		    }
		}

		tower_position.resize(2);

		tower_position.at(0)=4;
		tower_position.at(1)=4;

		is_initialized = true;

		cout<<"got copy"<<endl;
	}



   int x_to_image = (int)(roundf((x+environment_width/2)*10));
   int y_to_image = (int)(roundf((y+environment_height/2)*10));

/*Mat check_slice_rssi_map = Mat::zeros(corridor_contours_img.size(), CV_8UC1);

	for(int it_x = 0;it_x<10*environment_width;it_x++)
	{
		for(int it_y = 0;it_y<10*environment_height;it_y++)
		{
	check_slice_rssi_map.at<uchar>(it_x,it_y)=(uchar)RSSI_map.at(it_x).at(it_y)*10;

		}
	}

	circle(check_slice_rssi_map, Point(x_to_image,y_to_image),2,Scalar(100),2,8,0);
	check_slice_rssi_map.at<uchar>(x_to_image,y_to_image) = (uchar)255;

	namedWindow( "display",WINDOW_NORMAL );// Create a window for display.
	imshow("display", check_slice_rssi_map );                   // Show our image inside it.
	waitKey(1);*/

	float p[3] = {-0.08685 ,5.205e-17,1.329};


	float distance = (float)sqrt(pow(tower_position.at(0)-x,2)+pow(tower_position.at(1)-y,2));
	float noisy_distance = distance + RSSI_map.at(x_to_image).at(y_to_image);

	float bearing = wraptopi((float)atan2((float)(tower_position.at(1)-y),(float)(tower_position.at(0)-x))-heading);
	float noisy_distance_bearing = noisy_distance*(bearing*bearing*p[0]+bearing*p[1]+p[2]);
	//cout<<"distancwe bearing "<<distance_bearing<<endl;


	//cout<<"coordinates  "<<x<<"  "<<y<<"    "<<(roundf((x+environment_width)*10))<<"   "<<(roundf((-x+environment_height)*10))<<endl;

	//RSSI = Pn - 10*gamma*log10(distance)
	float Pn = -47.0f;
	float gamma_rssi = 4.0f;

	float noisy_RSSI = Pn - 10*gamma_rssi*log10(noisy_distance_bearing);



	return noisy_RSSI;//RSSI_map.at((int)x).at((int)y).at((int)(heading * 180.0 / (M_PI *3)));

}

