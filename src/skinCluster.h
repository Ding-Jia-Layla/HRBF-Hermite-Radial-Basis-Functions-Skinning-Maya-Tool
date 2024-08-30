#ifndef SKINCLUSTER_H
#define SKINCLUSTER_H
#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h> 
#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>
#include <maya/MPxSkinCluster.h> 
#include <maya/MItGeometry.h>
#include <maya/MPoint.h>
#include <maya/MFnMatrixData.h>
#include "HRBFManager.h"
#include <memory>
class myHRBFSkinCluster : public MPxSkinCluster
{
public:
    static  void* creator();
    static  MStatus initialize();
    std::unique_ptr<HRBFManager> hrbfManager;
    MMatrixArray bPreMatrices;
    virtual MStatus deform(MDataBlock& block,
        MItGeometry& iter,
        const MMatrix& mat,
        unsigned int multiIndex);

	MStatus skinLBS(MMatrixArray&  transforms,
		int numTransforms,
		MArrayDataHandle& weightListHandle,
		MItGeometry&   iter);
    static const MTypeId id;
    bool build;
};
const MTypeId myHRBFSkinCluster::id(0x00112212);
#endif