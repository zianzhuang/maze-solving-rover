#include <imagePipeline.h>
#include <string>
#include <algorithm>
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;

// vector of matrices -> descriptors of each of the 3 objects
std::vector<cv::Mat> descriptors_objects;

ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

// new function to load descriptors for each object, only need to call once each trial
void ImagePipeline::loadObjects(Boxes boxes)
{
    // iterate through each template in boxes.templates, and create descriptor matrices for each
    for (auto it = boxes.templates.begin(); it != boxes.templates.end(); it++)
    {
        // dereference it to img_object
        cv::Mat img_object = *it;
        
        // error checking
        if (!img_object.data)
        {
            std::cout << " --(!) Error reading images " << std::endl;
            return;
        }

        // Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 400;
        cv::Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints_object;
        cv::Mat descriptors_object;
        detector->detectAndCompute(img_object, noArray(), keypoints_object, descriptors_object);
        
        // pushback to the descriptors_objects vector
        descriptors_objects.push_back(descriptors_object);
    }
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes)
{
    // initialize template_id to be -1
    int template_id = -1;
    
    // error checking
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {
        cv::Mat img_scene = img;
        std::vector<float> match_points; // vector to store "scores" for each object based on closeness of match to scene

        // iterate through each template in descriptors_objects vector, and match with img
        for (auto it = descriptors_objects.begin(); it != descriptors_objects.end(); it++)
        {
            // error checking
            if (!img_scene.data)
            {
                std::cout << " --(!) Error reading images " << std::endl;
                return -1;
            }

            //-- Step 1: For scene imgage, detect the keypoints using SURF Detector, compute the descriptors
            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);
            std::vector<KeyPoint> keypoints_scene, keypoints_object;
            cv::Mat descriptors_scene;
            cv::Mat descriptors_object;
            descriptors_object = *it; // dereference it to get descriptors_object
            detector->detectAndCompute(img_scene, noArray(), keypoints_scene, descriptors_scene);

            //-- Step 2: Matching descriptor vectors with a FLANN based matcher
            // Since SURF is a floating-point descriptor NORM_L2 is used
            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector<std::vector<DMatch>> knn_matches;
            matcher->knnMatch(descriptors_scene, descriptors_object, knn_matches, 2);

            //-- Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.7f;
            std::vector<DMatch> good_matches;

            // iterate through total matches, and only add to good_matches if within threshold
            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }

            // count number of "good_matches" and append to match_points vector
            match_points.push_back(good_matches.size());
        }

        // pick max value from match_points --> use the index of max value
        // if max value from match_points < threshold, assume no no tag at box --> index of max value to -1
        // template_id = index of max value
        std::cout << "Array: ";
        for (auto it = match_points.begin(); it != match_points.end(); it++)
        {
            std::cout << *it << " ";
        }
          // in Gazebo simulation, the all black box produces (0, x, 0)
        if ((match_points[0] == 0) && (match_points[2] == 0))
        {
            cv::imshow("view", img);
            cv::waitKey(10);
            return template_id;
        }

        int lower_threshold = 20;
        int upper_threshold = 90;
        int max_index = std::max_element(match_points.begin(), match_points.end()) - match_points.begin();

        // only consider it a match, if the points are between the lower and upper thresholds
        if ((match_points[max_index] >= lower_threshold) && (match_points[max_index] <= upper_threshold))
        {
            template_id = max_index;
        }
    
        //std::cout << std::endl << "ID: " << template_id;
        cv::imshow("view", img);
        cv::waitKey(10);
        // std::cout << "\n----------------------------\n";
    }
    return template_id;
}

cv::Mat ImagePipeline::getImg()
{
    return img;
}
