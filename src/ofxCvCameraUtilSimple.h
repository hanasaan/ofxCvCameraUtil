//
//  Created by Yuya Hanai, https://github.com/hanasaan/
//
#pragma once

#include "ofMain.h"

// using without ofxCv.
namespace ofxCvCameraUtil
{
    struct Intr
    {
        float cx;
        float cy;
        float f;
        float w;
        float h;
        Intr() : cx(0), cy(0), f(0), w(0), h(0) {}
        Intr(float cx, float cy, float f, float w, float h)
            : cx(cx), cy(cy), f(f), w(w), h(h) {}
    };
    
    inline ostream& operator<<(ostream& os, const Intr& intr) {
        int w = 8;
        int p = 8;
        os << fixed << setw(w) << setprecision(p) << intr.cx << std::endl;
        os << intr.cy << std::endl;
        os << intr.f << std::endl;
        os << intr.w << std::endl;
        os << intr.h << std::endl;
        
        return os;
    }
    
    inline istream& operator>>(istream& is, Intr& intr) {
        is >> intr.cx; is.ignore(1);
        is >> intr.cy; is.ignore(1);
        is >> intr.f; is.ignore(1);
        is >> intr.w; is.ignore(1);
        is >> intr.h;
        return is;
    }
    
    typedef vector<float> DistCoeffs;
    
    inline ostream& operator<<(ostream& os, const DistCoeffs& coeffs) {
        int w = 8;
        int p = 12;
        os << fixed << setw(w) << setprecision(p);
        for (auto& p : coeffs) {
            os << p << std::endl;
        }
        return os;
    }
    
    inline istream& operator>>(istream& is, DistCoeffs& coeffs) {
        coeffs.clear();
        while (!is.eof()) {
            float f = 0;
            is >> f; is.ignore(1);
            coeffs.push_back(f);
        }
        return is;
    }
    
    static inline void setCamera(const Intr& intr, ofCamera& outcam)
    {
        float fov = ofRadToDeg(2.0f * atanf((intr.h * 0.5f) / intr.f));
        
        outcam.setFov(fov);
        
        float lx = -2.0 * (intr.cx - (intr.w * 0.5 - 0.5)) / intr.w;
        float ly = 2.0 * (intr.cy - (intr.h * 0.5 - 0.5)) / intr.h;
        
        outcam.setLensOffset(ofVec2f(lx, ly));
    }

    static inline void setCamera(float cx, float cy, float fy, float w, float h, ofCamera& outcam)
    {
        setCamera(Intr(cx, cy, fy, w, h), outcam);
    }
    
    static inline Intr getIntrinsics(const ofCamera& cam, float width, float height) {
        float lx = cam.getLensOffset().x;
        float ly = -cam.getLensOffset().y;
        
        float focal = 0.5 * height / tanf(ofDegToRad(0.5 * cam.getFov()));
        float cx = 0.5 * width * (1.0 - lx) - 0.5;
        float cy = 0.5 * height * (1.0 - ly) - 0.5;
        
        return Intr(cx, cy, focal, width, height);
    }
    
    static void saveIntr(string path, const Intr& intr)
    {
        ofFile file(path, ofFile::WriteOnly);
        file << "#ofxCvCameraUtil::Intr" << endl;
        file << intr << endl;
        file.close();
    }
    
    static void saveIntr(string path, const ofCamera& cam, float width, float height)
    {
        saveIntr(path, getIntrinsics(cam, width, height));
    }
    
    static Intr loadIntr(string path)
    {
        Intr intr;
        ofFile file(path);
        if (!file.exists()) return Intr();
        string tmp;
        file >> tmp;
        file >> intr;
        return intr;
    }
    
    static void loadIntrToCamera(string path, ofCamera& outcam)
    {
        setCamera(loadIntr(path), outcam);
    }
    
    static void saveDistCoeffs(string path, const DistCoeffs& dc)
    {
        ofFile file(path, ofFile::WriteOnly);
        file << "#ofxCvCameraUtil::DistCoeffs" << endl;
        file << dc;
        file << endl;
        file.close();
    }
    
    static DistCoeffs loadDistCoeffs(string path)
    {
        DistCoeffs dc;
        ofFile file(path);
        if (!file.exists()) return DistCoeffs();
        string tmp;
        file >> tmp;
        file >> dc;
        return dc;
    }
    
    // some utils
    static void saveTransform(string path, const ofMatrix4x4& mat, string tag = "#transform")
    {
        ofFile file(path, ofFile::WriteOnly);
        file << tag << endl;
        file << mat;
        file << endl;
        file.close();
    }
    
    static ofMatrix4x4 loadTransform(string path)
    {
        ofMatrix4x4 mat;
        ofFile file(path);
        if (!file.exists()) return ofMatrix4x4();
        string tmp;
        file >> tmp;
        file >> mat;
        return mat;
    }
};