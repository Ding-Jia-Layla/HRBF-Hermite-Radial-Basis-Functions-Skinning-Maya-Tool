#ifndef HRBFMANAGER_H
#define HRBFMANAGER_H
#include <vector>
#include <maya/MItGeometry.h>
#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPxSkinCluster.h> 
#include <maya/MPoint.h>
#include "HRBF.h"
#include <vector>
#include "hrbfCore.h"
#include <utility> 

class HRBFManager {
public:
    HRBFManager();
    ~HRBFManager();
    std::vector<std::tuple<float, MVector, float>> maxfirstHRBFValuesAndGradients;
    std::vector<std::tuple<float, MVector, float>> maxsecondHRBFValuesAndGradients;
    void printCorrection(int&vertexIndex,MVector& correction);
    void printMaxFirstHRBFValuesAndGradients();
    std::vector<std::unique_ptr<HRBF>> m_HRBFs;
    //float angleCheck(float&a, float& b,float& x, float& theta);
    void adjustVertices(MItGeometry& iter);
    void setBonesRelationship(std::vector<int>& bonesList);
    void buildHRBFs(std::vector<int>& jointList,MMatrixArray& binds, MMatrixArray& transforms,MItGeometry& iter);
    void unionFirstField(MMatrixArray& binds);
    void unionSeconeField(MMatrixArray& transforms);
    float Angle = 55.0f;
};
#endif
