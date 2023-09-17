/*
ImagePipeline.cpp should be fully functional, REMEMBER TO USE MY imagePipeline.h file as well
Once you call getTemplateID(), it will return -1 (for blank/noise), 0, 1, 2 for image IDs

In the contest2.cpp file, every time you get to a box, the following process should be followed:
1. Specify a window of time when you are actively calling getTemplateID(), save all results in array/vector
2. Take the mode of the array/vector --> this is our templateID guess
3. We know which box # (boxID) we are at, so make a pair --> std::pair <int,int> new_pair = std::make_pair(templateID, boxID)
    3a. #include <utility> is needed for pair
4. push_back the new_pair into a vector (at the end of the contest, this vector should have 5 pairs)
5. Let's call this vector --> results_vector

the function to write to a file is below, probably needs testing. This requires you to have
a global varaible in your contest2.cpp file which is a vector of pairs (1st element = templateID, 2nd element = boxID)
as was described in step 5 above
the only thing I'm not clear about is the file paths but i'm sure a TA can help
*/

#include <utility>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <regex>

void writeResults(std::vector<std::pair<int, int>>);
std::string trim(const std::string&);

int main(int argc, char *argv[])
{
    // make an example vector and push_back some pairs to it
    std::vector<std::pair<int, int>> results_vector;
    std::pair<int, int> new_pair1 = std::make_pair(-1, 0); // box 0 is blank
    std::pair<int, int> new_pair2 = std::make_pair(2, 1);  // box 1 is ID 2
    std::pair<int, int> new_pair3 = std::make_pair(2, 2);  // box 2 is ID 2 --> duplicate
    std::pair<int, int> new_pair4 = std::make_pair(1, 3);  // box 3 is ID 1
    std::pair<int, int> new_pair5 = std::make_pair(0, 4);  // box 4 is ID 0
    results_vector.push_back(new_pair1);
    results_vector.push_back(new_pair2);
    results_vector.push_back(new_pair3);
    results_vector.push_back(new_pair4);
    results_vector.push_back(new_pair5);

    // test print to make sure vector is constructed correctly
    /*
    for (auto it = results_vector.begin(); it != results_vector.end(); it++)
    {
        std::cout << it->first << " " << it->second << std::endl;
    }
    */

    // write the results of the vector to file
    writeResults(results_vector);

}

void writeResults(std::vector<std::pair<int, int>> results_vector)
{
    // I moved "coords.xml" and "templates.xml" to my current directory
    // for actual competition, you'll probably have to access the files through
    // "./mie443_contest2\boxes_database", where "./" is the entire preceding directory

    // declare templates file
    std::fstream templates_file;

    // make the file we will be writing to
    std::ofstream results_file("results.txt");

    // iterate through the results_vector and write to results_file accordinly
    for (auto it = results_vector.begin(); it != results_vector.end(); it++)
    {
        results_file << "Template: ";
        std::string template_name;
        
        // the picture is blank
        if (it->first == -1)
        {
            // std::cout << "blank\n";
            template_name = "blank";
        }

        // the picture is not blank, go to templates.xml and read the corresponding line
        // template 0 -> line 4 in xml file, template 1 -> line 5 in xml file, template 2 -> line 6 in xml file
        else 
        {
            // use i to count to lines 4, 5, and 6, etc
            int i = 1;
            templates_file.open("templates.xml");
            switch(it->first)
            {
                case 0:
                    // std::cout << "0\n";
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
                    // std::cout << "1\n";
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
                    // std::cout << "2\n";
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
        // trim all leading whitespace from template_name, make a new string template_name_trimmed
        std::string template_name_trimmed;
        template_name_trimmed = trim(template_name);
        // write the template name to file
        std::cout << template_name_trimmed << std::endl;
        results_file << template_name_trimmed;

        // write in the location information. it->second is the boxID, we use this to go into our
        // global boxes class, and retrieve the x, y, and phi values to write
        results_file << " was found at: X = ";
        // results_file << boxes.coords[it->second][0];
        results_file << ", Y = ";
        // results_file << boxes.coords[it->second][1];
        results_file << ", Phi = ";
        // results_file << boxes.coords[it->second][2];
        results_file << "\n";
    }

    // close the files, all done!
    templates_file.close();
    results_file.close();
}

// function to trim strings by removing whitespaces
std::string trim(const std::string& str)
{
    const auto strBegin = str.find_first_not_of(" \t");
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(" \t");
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}