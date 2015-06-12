//
//  Created by Yuya Hanai, https://github.com/hanasaan/
//
#pragma once

#include "ofxCv.h"
#include "ofxCvCameraUtilSimple.h"

namespace ofxCvCameraUtil
{
    static inline void setCamera(const cv::Mat& camera_matrix, float w, float h, ofCamera& outcam)
    {
        const float cx = camera_matrix.at<double>(0, 2);
        const float cy = camera_matrix.at<double>(1, 2);
        const float fy = camera_matrix.at<double>(1, 1);
        setCamera(cx, cy, fy, w, h, outcam);
    }
    
    static inline void setCamera(const ofxCv::Intrinsics& intrinsics, ofCamera& outcam)
    {
        setCamera(intrinsics.getCameraMatrix(), intrinsics.getImageSize().width,
                  intrinsics.getImageSize().height, outcam);
    }
    
    static inline cv::Mat createCameraMatrix(const ofCamera& cam, float width, float height) {
        Intr intr = getIntrinsics(cam, width, height);
        
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        
        camera_matrix.ptr<double>()[0] = intr.f;//fx
        camera_matrix.ptr<double>()[4] = intr.f;//fy
        camera_matrix.ptr<double>()[2] = intr.cx;//cx
        camera_matrix.ptr<double>()[5] = intr.cy;//cy
        camera_matrix.ptr<double>()[8] = 1.0;
        
        return camera_matrix;
    }
};