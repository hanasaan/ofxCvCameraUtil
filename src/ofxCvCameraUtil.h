//
//  Created by Yuya Hanai, https://github.com/hanasaan/
//
#pragma once

#include "ofxCv.h"

namespace ofxCvCameraUtil
{
    static inline void setCamera(const cv::Mat& camera_matrix, float w, float h, ofCamera& outcam)
    {
        const float cx = camera_matrix.at<double>(0, 2);
        const float cy = camera_matrix.at<double>(1, 2);
        const float fy = camera_matrix.at<double>(1, 1);
        float fov = ofRadToDeg(2.0f * atanf((h * 0.5f) / fy));
        
        outcam.setFov(fov);
        
        float lx = -2.0 * (cx - w * 0.5 - 0.5) / w;
        float ly = 2.0 * (cy - h * 0.5 - 0.5) / h;
        
        outcam.setLensOffset(ofVec2f(lx, ly));
    }
    
    static inline void setCamera(const ofxCv::Intrinsics& intrinsics, ofCamera& outcam)
    {
        setCamera(intrinsics.getCameraMatrix(), intrinsics.getImageSize().width,
                  intrinsics.getImageSize().height, outcam);
    }
    
    static inline cv::Mat createCameraMatrix(const ofCamera& cam, float width, float height) {
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        
        ofxCv::Intrinsics intr;
        float focal = 0.5 * height / tanf(ofDegToRad(0.5 * cam.getFov()));
        camera_matrix.ptr<double>()[0] = focal;//fx
        camera_matrix.ptr<double>()[4] = focal;//fy
        camera_matrix.ptr<double>()[2] = 0.5 * width - 0.5;//cx
        camera_matrix.ptr<double>()[5] = 0.5 * height - 0.5;//cy
        camera_matrix.ptr<double>()[8] = 1.0;
        
        return camera_matrix;
    }
};