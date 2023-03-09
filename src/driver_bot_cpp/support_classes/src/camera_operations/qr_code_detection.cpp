#include "qr_code_detection.h"



QRCodeDetection::QRCodeDetection()
/*Class detects QR code and returns true if found, else returns false*/
: m_w{WIDTH}, m_h{HEIGHT}
{

}

QRCodeDetection::~QRCodeDetection()
{

}
//------------MAIN FUNCTIONALITY --------------------//
void QRCodeDetection::CheckQRCode(cv::Mat& frame)
{
    /*Detects QR code
    Args:
        frame: raw image frame from 'cam_pub' topic
    Returns:
        true if QR code is detected, false if not*/
    m_checkQRCode = false;

    CreateQuirc();
    ResizeQuirc();
    uint8_t *buf = quirc_begin(m_quirc, &m_w, &m_h);

    frame = ResizeFrame(frame); //resizes frame to w and h
    buf = ConvertFrameToBuf(frame, buf);
    quirc_end(m_quirc);

    int num_codes; //#number of qr codes detected
    
    num_codes = quirc_count(m_quirc);
    DecodeQR(num_codes);
    quirc_destroy(m_quirc);
}

//------------SUPPORT FUNCTIONS--------------------//
void QRCodeDetection::Display(cv::Mat& frame)
{
    /*If QR code is detected, a box around the QR code is displayed
    Args:
        frame: raw image from 'cam_pub' topic
        bbox: optional output array of vertices of the found QR code quadrangle. Will be empty if not found.    
    */
        
    int n = m_bbox.rows;
    for(int i = 0 ; i < n ; i++)
    {
        cv::line(frame, cv::Point2i(m_bbox.at<float>(i,0),m_bbox.at<float>(i,1)), cv::Point2i(m_bbox.at<float>((i+1) % n,0), m_bbox.at<float>((i+1) % n,1)), cv::Scalar(255,0,0), 3);
    }
    cv::imshow("Result", m_bbox);   
}

void QRCodeDetection::CreateQuirc()
{
    /*Allocates data on the heap for quirc object. Checks if data
    is succesfully allocated*/
    m_quirc = quirc_new();
    if (!m_quirc)
    {
        std::cout << "Failed to allocate memory" << std::endl;
        std::exit(EXIT_FAILURE); 
    }
}

void QRCodeDetection::ResizeQuirc()
{
    /*Resizes allocated heap memory to the desired amount. 
    m_w and m_h are set in general numbers file*/
    if (quirc_resize(m_quirc, m_w, m_h) < 0) 
    {
        std::cout << "Failed to allocate video memory" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

cv::Mat QRCodeDetection::ResizeFrame(cv::Mat& frame)
{
    /*Resizes received frame to be the same size as quirc object
    Args:
        frame: received from 'cam_pub' topic containing frame data in uint8_t
    */
    cv::resize(frame, frame, cv::Size(m_w, m_h));
    assert(frame.cols == m_w);
    assert(frame.rows == m_h);
    return frame; 
}

uint8_t* QRCodeDetection::ConvertFrameToBuf(cv::Mat& frame, uint8_t*& buf)
{
    /* convert frame into buf 
    Args:
        frame: frame: received from 'cam_pub' topic containing frame data in uint8_t
        buf: frame buffer -> grayscales data from frame to be used in quirc object
    */
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY, 0);
    for (int y = 0; y < gray.rows; y++) 
    {
        for (int x = 0; x < gray.cols; x++) 
        {
            buf[(y * m_w + x)] = gray.at<uint8_t>(y, x);
        }
    }
    return buf; 
}

void QRCodeDetection::DecodeQR(int num_codes)
{
    /*Decodes QR code
    Args:
        num_codes: number of detected QR codes on screen
    Returns:
        Decoded QR code if QR codes are detected
    */
    if (num_codes == 0) 
    {
        std::cout << "NO QR CODE DETECTED IN QR CODE DETECTION" << std::endl;
        m_checkQRCode = false;

    }

    for (int i{}; i < num_codes; i++) 
    {
        struct quirc_code code;
        quirc_decode_error_t err;

        quirc_extract(m_quirc, i, &code);

        /* Decoding stage */
        err = quirc_decode(&code, &m_data);
        std::cout << "check" << std::endl;
        if (err)
        {
            std::cout << "DECODE FAILED IN QR CODE DETECTION: " << quirc_strerror(err) << std::endl;  
            m_checkQRCode = false;
        }
        else
        {
            std::cout << "Data IN QR CODE DETECTION: " << m_data.payload << std::endl;
            m_checkQRCode = true;
        }
    }

}