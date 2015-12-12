//
//  Created by Yuya Hanai, https://github.com/hanasaan/
//
#pragma once

#include "ofxCv.h"
#include "ofxCvCameraUtilSimple.h"

namespace ofxCvCameraUtil
{
    static inline ofxCvCameraUtil::Intr toOfxCvCameraUtil(const ofxCv::Intrinsics& src)
    {
        ofxCvCameraUtil::Intr dst;
        dst.f = src.getFocalLength();
        dst.cx = src.getPrincipalPoint().x;
        dst.cy = src.getPrincipalPoint().y;
        dst.w = src.getImageSize().width;
        dst.h = src.getImageSize().height;
        return dst;
    }

    static inline cv::Mat createCameraMatrix(float cx, float cy, float f);
    static inline void toOfxCv(const ofxCvCameraUtil::Intr& src, ofxCv::Intrinsics& dst)
    {
        dst.setup(createCameraMatrix(src.cx, src.cy, src.f), cv::Size(src.w, src.h));
        return dst;
    }
    
    static inline DistCoeffs toOfxCvCameraUtil(const cv::Mat& dist_coeffs) {
        int sz = dist_coeffs.cols * dist_coeffs.rows;
        DistCoeffs dc;
        dc.resize(sz);
        for (int i=0; i<sz; ++i) {
            dc[i] = dist_coeffs.ptr<double>()[i];
        }
        return dc;
    }
    
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
    
    static inline cv::Mat createCameraMatrix(float cx, float cy, float f) {
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        
        camera_matrix.ptr<double>()[0] = f;
        camera_matrix.ptr<double>()[4] = f;
        camera_matrix.ptr<double>()[2] = cx;
        camera_matrix.ptr<double>()[5] = cy;
        camera_matrix.ptr<double>()[8] = 1.0;
        
        return camera_matrix;
    }
    
    static inline cv::Mat createDistCoeffs(float k1, float k2, float p1, float p2, float k3) {
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        dist_coeffs.ptr<double>()[0] = k1;
        dist_coeffs.ptr<double>()[1] = k2;
        dist_coeffs.ptr<double>()[2] = p1;
        dist_coeffs.ptr<double>()[3] = p2;
        dist_coeffs.ptr<double>()[4] = k3;
        return dist_coeffs;
    }
    
    static inline cv::Mat createCameraMatrix(const ofCamera& cam, float width, float height) {
        Intr intr = getIntrinsics(cam, width, height);
        return createCameraMatrix(intr.cx, intr.cy, intr.f);
    }
    
    static inline void createIntrinsics(const ofCamera& cam, float width, float height, ofxCv::Intrinsics& out_intrinsics)
    {
        out_intrinsics.setup(createCameraMatrix(cam, width, height), cv::Size(width, height));
    }
};