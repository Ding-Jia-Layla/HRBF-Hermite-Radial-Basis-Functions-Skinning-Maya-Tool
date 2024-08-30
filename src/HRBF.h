#ifndef HRBF_H
#define HRBF_H

#include <vector>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MMatrixArray.h>
#include <maya/MItGeometry.h>
#include "hrbfCore.h"
#include <maya/MGlobal.h>
#include <memory>
class HRBF{
public:
    HRBF(MMatrix &bPreMatrix);
    ~HRBF();
    std::vector<HRBF*> m_childrens;
    HRBF* m_parent;
    MPoint m_boneChildPos;
    std::vector<MPoint> m_bonesChildrenPos;
    std::unique_ptr<HRBF_fit> hrbf_core;
    MPoint m_bonePositionWS;
    MPoint m_bonePositionLS;
    MVector m_bone;
    std::vector<float> m_finalVals;
    std::vector<MVector> m_finalGrads;
    void setBonesPN();
    float compactSupport(float& d, float& r);
    void cullingVertex(MPoint& pos, MVector& norm);
    void compute();
    std::vector<MPoint> m_finalPos;
    std::vector<MVector> m_finalNorm;
    float m_r;
    float m_ret;
    std::vector<float> m_originalRet;
    MVector m_grad;
    MMatrix  m_bPreMatrix;
    
    
};
#endif
