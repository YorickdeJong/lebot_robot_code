#include "draw_shapes.h"
#include "frame_analysis.h"
#include "overlap.h"
#include "std_msgs/Bool.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
class ObjectDetection
{
    public:
        ObjectDetection();
        ~ObjectDetection() = default;

        void CameraCallBack(const sensor_msgs::ImageConstPtr& msg);
        void DataAnalyses(cv::Mat& frame);
        bool Validate();
        void Detection();

        // SUPPORT FUNCTIONS
        bool DetectionCircle();
        void PublishOverlap(bool checkBlue);

    private:
        ros::NodeHandle m_node;
        ros::Publisher m_pubOverlap;
        image_transport::Subscriber m_subCamera; 

        FrameAnalysis m_frameAnalysis;
        Draw m_draw;
        Overlap m_overlap;

        float m_radiusContour, m_radiusCircle;
        cv::Point2f m_centerContour, m_centerCicle;
        cv::Mat m_frame;
};