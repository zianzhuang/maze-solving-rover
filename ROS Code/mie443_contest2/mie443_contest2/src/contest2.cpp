#include <boxes.h>
#include <robot_pose.h>
#include <navigation.h>
#include <imagePipeline.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <utility>
#include <string>
#include <regex>
#include <cmath>
#include <chrono>
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

// GLOBAL VARAIBLES ===============================================================================
Boxes boxes;
RobotPose robotPose(0, 0, 0);
std::vector<std::pair<int, int>> results_vector;
float startPose_x = 0.0, startPose_y = 0.0, startPose_z = 0.0;
float blockDist = 0.5;
int goal = -1;

// FUNCTIONS ======================================================================================
float calcDistance(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void update_coords()
{ // update coords [0-2] to navigatable coords [3-5]
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        boxes.coords[i].push_back(boxes.coords[i][0] + blockDist * cos(boxes.coords[i][2])); // goal x
        boxes.coords[i].push_back(boxes.coords[i][1] + blockDist * sin(boxes.coords[i][2])); // goal y
        if (boxes.coords[i][2] < 0.0)                                                        // goal phi
        {
            boxes.coords[i].push_back(boxes.coords[i][2] + M_PI);
        }
        else
        {
            boxes.coords[i].push_back(boxes.coords[i][2] - M_PI);
        }
        boxes.coords[i].push_back(0); // 0/1 indicate F(not examed)/T(examed)
        std::cout << "Goal coordinates for box " << i << ": x_goal: " << boxes.coords[i][3] << " y_goal: " << boxes.coords[i][4] << " phi_goal: " << boxes.coords[i][5] <<  std::endl;
    }
}

void find_closestBox()
{ // find closest box with respective to current pose
    float minDist = std::numeric_limits<float>::infinity();
    float boxDist = 0.0;
    goal = -1;
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        if (boxes.coords[i][6] != 1)
        { // for unexamed box
            boxDist = calcDistance(robotPose.x, robotPose.y, boxes.coords[i][3], boxes.coords[i][4]);
            if (minDist > boxDist)
            {
                minDist = boxDist;
                goal = i;
            }
        }
    }
}

void goto_Goal()
{
    if (goal == -1)
    { // go to startPose if no unexamed box
        std::cout << "ERROR: no unexamed box --> go to starting coordinates" << std::endl;
        Navigation::moveToGoal(startPose_x, startPose_y, startPose_z);
    }
    else
    {
        std::cout << "Going to Closest box:" << goal << std::endl;
        Navigation::moveToGoal(boxes.coords[goal][3], boxes.coords[goal][4], boxes.coords[goal][5]);
        // ros::spinOnce();
        // if (calcDistance(robotPose.x, robotPose.y, boxes.coords[goal][3], boxes.coords[goal][4]) > 0.2)
        // {
        //     Navigation::moveToGoal(startPose_x, startPose_y, startPose_z);
        //     Navigation::moveToGoal(boxes.coords[goal][3], boxes.coords[goal][4], boxes.coords[goal][5]);
        // }
    }
}

std::string trim(const std::string &str)
{
    const auto strBegin = str.find_first_not_of(" \t");
    if (strBegin == std::string::npos)
        return ""; // no content
    const auto strEnd = str.find_last_not_of(" \t");
    const auto strRange = strEnd - strBegin + 1;
    return str.substr(strBegin, strRange);
}

void writeResults(std::vector<std::pair<int, int>> results_vector)
{
    std::fstream templates_file;
    std::ofstream results_file("/home/janicezhou/Desktop/results.txt"); //// update result file address
    for (auto it = results_vector.begin(); it != results_vector.end(); it++)
    {
        results_file << "Template: ";
        std::string template_name;
        if (it->first == -1)
        {
            template_name = "no tag";
        }
        else
        {
            int i = 1;
            templates_file.open("/home/janicezhou/catkin_ws/src/mie443_contest2/boxes_database/templates.xml"); //// update result file address
            switch (it->first)
            {
            case 0:
                while (std::getline(templates_file, template_name))
                {
                    if (i == 4)
                    {
                        break;
                    }
                    i++;
                }
                templates_file.close();
                break;
            case 1:
                while (std::getline(templates_file, template_name))
                {
                    if (i == 5)
                    {
                        break;
                    }
                    i++;
                }
                templates_file.close();
                break;
            case 2:
                while (std::getline(templates_file, template_name))
                {
                    if (i == 6)
                    {
                        break;
                    }
                    i++;
                }
                templates_file.close();
                break;
            default:
                std::cout << "Error in switch statement\n";
                templates_file.close();
                break;
            }
        }
        std::string template_name_trimmed;
        template_name_trimmed = trim(template_name);
        results_file << template_name_trimmed;
        results_file << " was found at: X = ";
        results_file << boxes.coords[it->second][0];
        results_file << ", Y = ";
        results_file << boxes.coords[it->second][1];
        results_file << ", Phi = ";
        results_file << boxes.coords[it->second][2];
        results_file << "\n";
    }
    templates_file.close();
    results_file.close();
}

int main(int argc, char **argv)
{
    bool all_checked = true;
    uint64_t duration = 0;
    std::chrono::time_point<std::chrono::system_clock> start;
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // wait to establish position for robot for 5 seconds
    start = std::chrono::system_clock::now();
    while ((startPose_x == 0.0 || startPose_y == 0.0 || startPose_z == 0.0) && duration < 5)
    { // wait for robot to get pose info or 5 seconds
        ros::spinOnce();
        startPose_x = robotPose.x;
        startPose_y = robotPose.y;
        startPose_z = robotPose.phi;
        duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    }
    std::cout << "Starting coordinates (OLD):" << " X: " << startPose_x << " Y: " << startPose_y << " Phi: " << startPose_z << std::endl;

    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // load boxes
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    std::cout << "Boxes loaded" << std::endl;
    /*
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: "
                  << boxes.coords[i][2] << std::endl;
    }
    */

    // set up imagePipeline
    ImagePipeline imagePipeline(n);
    imagePipeline.loadObjects(boxes);
    int templateID = -1;

    // add to boxes x y and phi of approriate robot position
    update_coords();

    // look at what the first image is
    ros::spinOnce();
    templateID = imagePipeline.getTemplateID(boxes);

    // save starting position
    startPose_x = robotPose.x;
    startPose_y = robotPose.y;
    startPose_z = robotPose.phi;
    std::cout << "Starting coordinates (NEW):" << " X: " << startPose_x << " Y: " << startPose_y << " Phi: " << startPose_z << std::endl;

    while (ros::ok())
    {
        ros::spinOnce();
    
        find_closestBox();

        if (boxes.coords[goal][6] != 1)
        { // if box is unexamined
            goto_Goal();
            // identify image
            ros::spinOnce();
            templateID = imagePipeline.getTemplateID(boxes);
            // push back image and box info
            results_vector.push_back(std::make_pair(templateID, goal));
            std::cout << "Template ID Found: " << templateID << std::endl;
            boxes.coords[goal][6] = 1; // box is now examined
        }
        all_checked = true;
        // check if all boxes have been examined
        for (int i = 0; i < boxes.coords.size(); ++i)
        {
            if (boxes.coords[i][6] != 1)
            {
                all_checked = false;
            }
        }
        if (all_checked)
        {
            break;
        }
        ros::Duration(0.01).sleep();
    }
    std::cout << "All boxes have been examined --> going to starting coordinates" << std::endl;
    ros::spinOnce();
    std::cout << "Current coordinates:" << " X: " << robotPose.x << " Y: " << robotPose.y << " Phi: " << robotPose.phi << std::endl;
    std::cout << "Starting coordinates:" << " X: " << startPose_x << " Y: " << startPose_y << " Phi: " << startPose_z << std::endl;
    std::cout << "Moving back to starting coordinates now..." << std::endl;

    Navigation::moveToGoal(startPose_x, startPose_y, startPose_z);
    ros::spinOnce();
    Navigation::moveToGoal(startPose_x, startPose_y, startPose_z);

    ros::spinOnce();
    std::cout << "Ending coordinates:" << " X: " << robotPose.x << " Y: " << robotPose.y << " Phi: " << robotPose.phi << std::endl;

    writeResults(results_vector);
    std::cout << "Contest 2 is completed. Results.txt saved to Desktop." << std::endl;
    return 0;
}
