#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "quirc.h"
#include <memory>
#include "../../magicNumbers/magicNumbers.h"

/* Used QUIRC library as openCV gave a linking error for objdetect.h. 
 * QUIRC is written in C, hence the usage of raw pointers
 */

class QRCodeDetection
{
    public:
        QRCodeDetection();
        ~QRCodeDetection();
        void Display(cv::Mat& frame);
        void CheckQRCode(cv::Mat& frame);
        void CreateQuirc();
        void ResizeQuirc();
        cv::Mat ResizeFrame(cv::Mat& frame);
        uint8_t* ConvertFrameToBuf(cv::Mat& frame, uint8_t*& buf);
        void DecodeQR(int num_codes);
    
    //getters
        struct quirc_data GetData(){return m_data;};
        bool GetCheckQRCode(){return m_checkQRCode;};

    private:
        cv::Mat m_bbox, m_rectifiedImage; 
        struct quirc* m_quirc;
        struct quirc_data m_data;
        bool m_checkQRCode; 
        int m_w, m_h;
};