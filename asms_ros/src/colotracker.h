#ifndef COLOTRACKER_H
#define COLOTRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <region.h>
#include <histogram.h>
#include <iostream>


//#define SHOWDEBUGWIN

#define BIN_1 16
#define BIN_2 16
#define BIN_3 16

struct asms_cfg {
    int img_scale_target_diagonal = 100;
};

class ColorTracker
{
private:
    BBox lastPosition_;
    cv::Mat im1_;
    cv::Mat im2_;
    cv::Mat im3_;

    cv::Mat im1_old_;
    cv::Mat im2_old_;
    cv::Mat im3_old_;

    Histogram q_hist_;
    Histogram q_orig_hist_;
    Histogram b_hist_;

    asms_cfg cfg;
    double scaledWidth_;
    double scaledHeight_;
    double img_scale_factor_;

    double wAvgBg_;
    double bound1_;
    double bound2_;
    double accuracy_;

    cv::Point histMeanShift(double x1, double y1, double x2, double y2);
    cv::Point histMeanShiftIsotropicScale(double x1, double y1, double x2, double y2, double * scale, int * msIter = NULL);
    cv::Point histMeanShiftAnisotropicScale(double x1, double y1, double x2, double y2, double * width, double * height);

    inline void preprocessImage(cv::Mat & img);
    void extractBackgroundHistogram(int x1, int y1, int x2, int y2, Histogram &hist);
    void extractForegroundHistogram(int x1, int y1, int x2, int y2, Histogram &hist);

    inline double kernelProfile_Epanechnikov(double x)
        { return (x <= 1) ? (2.0/3.14)*(1-x) : 0; }
        //{ return (x <= 1) ? (1-x) : 0; }
    inline double kernelProfile_EpanechnikovDeriv(double x)
        { return (x <= 1) ? (-2.0/3.14) : 0; }
        //{ return (x <= 1) ? -1 : 0; }
public:
    int frame;
    int sumIter;

	// Init methods
    void init(cv::Mat & img_raw, int x1, int y1, int x2, int y2);

    // Set last object position - starting position for next tracking step
    inline void setLastBBox(int x1, int y1, int x2, int y2)
	{
        lastPosition_.setBBox(x1, y1, x2-x1, y2-y1, 1, 1);
    }

    inline BBox * getBBox()
    {
        BBox * bbox = new BBox();
        *bbox = lastPosition_;
        return bbox;
    }

	// frame-to-frame object tracking
    BBox * track(cv::Mat & img_raw, double x1, double y1, double x2, double y2);
    inline BBox * track(cv::Mat & img)
    {
        return track(img, lastPosition_.x, lastPosition_.y, lastPosition_.x + lastPosition_.width, lastPosition_.y + lastPosition_.height);
    }
};

#endif // COLOTRACKER_H
