#include "skinCluster.h"
#include <maya/MGlobal.h>

void* myHRBFSkinCluster::creator()
{
    myHRBFSkinCluster* cluster = new myHRBFSkinCluster();
    cluster->hrbfManager = std::make_unique<HRBFManager>();
    return cluster;
}

MStatus myHRBFSkinCluster::initialize()
{
    return MStatus::kSuccess;
}
void printIntegerValueskinCluster(int value) {
    MString output = "numTransforms value is: ";
    output += value; // 将整数转换为MString并附加到输出字符串上
    MGlobal::displayWarning(output);
}
MStatus myHRBFSkinCluster::deform( MDataBlock& block,
                      MItGeometry& iter,
                      const MMatrix& m,
                      unsigned int multiIndex)
{
    MStatus returnStatus;
    MArrayDataHandle transformsHandle = block.inputArrayValue( matrix );
    int numTransforms = transformsHandle.elementCount();
    printIntegerValueskinCluster(numTransforms);
    if ( numTransforms == 0 ) {
        return MS::kSuccess;
    }

    MMatrixArray transforms;
    for ( int i=0; i<numTransforms; ++i ) 
    {
        transforms.append( MFnMatrixData( transformsHandle.inputValue().data() ).matrix() );
        transformsHandle.next();
    }
  
    MArrayDataHandle bindHandle = block.inputArrayValue( bindPreMatrix );

    if (bindHandle.elementCount() > 0)
    {
        for (int i = 0; i < numTransforms; ++i)
        {
            // bindTFs = bPreMatrices
            MMatrix bind = MFnMatrixData(bindHandle.inputValue().data()).matrix();
            transforms[i] = bind * transforms[i];
            bPreMatrices.append(bind);
            bindHandle.next();
        }
    }
    MArrayDataHandle weightListHandle = block.inputArrayValue( weightList );
    MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
    int nb_weights = weightsHandle.elementCount();


 
    if (!build)
    {
        std::vector <int> jointList;
        for (int i = 0; i < numTransforms; i++) {
            jointList.push_back(i);
        }
        hrbfManager->buildHRBFs(jointList,bPreMatrices, transforms,iter);
        build = true;
    }
    returnStatus = skinLBS(transforms, numTransforms, weightListHandle, iter);
    iter.reset();
    hrbfManager->unionSeconeField(transforms);
    iter.reset();
    hrbfManager->adjustVertices(iter);
    return returnStatus;
}
MStatus myHRBFSkinCluster::skinLBS(MMatrixArray&  transforms,
	int numTransforms,
	MArrayDataHandle& weightListHandle,
	MItGeometry& iter) {
	MStatus returnStatus;

	// Iterate through each point in the geometry.
	//
	for (; !iter.isDone(); iter.next()) {
		MPoint pt = iter.position();
		MPoint skinned;
		// get the weights for this point -> must be dependent on the iterator somehow
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
		// compute the skinning -> TODO: what's the order that the weights are given in? Appears to just be maya list relatives order.
		for (int i = 0; i<numTransforms; ++i) {
			if (MS::kSuccess == weightsHandle.jumpToElement(i)) {
				skinned += (pt * transforms[i]) * weightsHandle.inputValue().asDouble();
			}
		}

		// Set the final position.
		iter.setPosition(skinned);
		// advance the weight list handle
		weightListHandle.next();
	}
	return returnStatus;
}
MStatus initializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj, PLUGIN_COMPANY, "3.0", "Any");
    result = plugin.registerNode(
        "myHRBFSkinCluster",
        myHRBFSkinCluster::id ,
        &myHRBFSkinCluster::creator ,
        &myHRBFSkinCluster::initialize ,
        MPxNode::kSkinCluster
        );

    return result;
}

MStatus uninitializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj );
    result = plugin.deregisterNode( myHRBFSkinCluster::id );

    return result;
}




