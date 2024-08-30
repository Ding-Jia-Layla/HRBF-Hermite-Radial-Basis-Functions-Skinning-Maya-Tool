#include "HRBF.h"
HRBF::HRBF(MMatrix& bPreMatrix)
{
	m_bPreMatrix = bPreMatrix;
	MMatrix m_boneWMat = m_bPreMatrix.inverse();
	m_bonePositionWS = MVector(m_boneWMat(3,0), m_boneWMat(3, 1), m_boneWMat(3, 2));
	m_bonePositionLS = m_bonePositionWS * bPreMatrix;
}
HRBF::~HRBF()
{

}
void HRBF::setBonesPN()
{
	for (int i = 0; i < m_childrens.size(); i++)
	{
		m_boneChildPos = m_childrens[i]->m_bonePositionLS;
		m_bonesChildrenPos.push_back(m_boneChildPos);
	}
}
float HRBF::compactSupport(float& d, float& r)
{
	if (r == 0) {
        return 0.5f; 
    }
	float newd = d / r;
	if (d < -r)
	{
		return 1.0f;
	}
	else if (d > r)
	{
		return 0.0f;
	}
	else
	{
		return (-3.0f / 16.0f) * newd * newd * newd * newd * newd + (5.0f / 8.0f) * (newd * newd * newd) - (15.0f / 16.0f) * newd + 0.5f;
	}

}
void HRBF::cullingVertex(MPoint& pos, MVector& norm)
{
	bool cull = false;
	MVector posLocal = pos * m_bPreMatrix;
	for (int i = 0; i < m_childrens.size(); i++)
	{
		MVector distance_bone_dir = m_bonesChildrenPos[i] - m_bonePositionLS;
		float bone_square = pow(distance_bone_dir.x, 2) + pow(distance_bone_dir.y, 2) + pow(distance_bone_dir.z, 2);
		MVector distance_mesh_dir = posLocal - m_bonePositionLS;
		float check = (distance_mesh_dir * distance_bone_dir) / bone_square;
		if (0.05f > check || check > 0.95f) {
			cull = true;
			break;
		}
		
	}
	if (!cull) {
		m_finalPos.push_back(posLocal);
		m_finalNorm.push_back(norm * m_bPreMatrix);

	}
}
void printIntegerValueHRBF(float value1) 
{
    MString output1 = "new vector m_originmalHrbf in individual value is: ";
    output1 += value1; 
    MGlobal::displayWarning(output1);

}
void calculateR()
{
}
void HRBF::compute() {
	MGlobal::displayWarning("compute calling!!!!!!");
	m_r = 0.0f;
	// // we set r to the distance between the bone and the farthest sampling point used for reconstruction
	for (auto& p : m_finalPos)
	{
		m_r = std::max(m_r, (float)(p - m_bonePositionLS).length());
	}
	// 5 2 5
	//printIntegerValueHRBF(m_r);
	// //get val,gradient
	std::vector<MVector> positions;
	for (int i = 0; i < m_finalPos.size(); i++) 
	{
		positions.push_back(m_finalPos[i]); // push back as MVector
	}
	//printIntegerValueHRBF(positions.size(),m_finalNorm.size());

	hrbf_core = std::make_unique<HRBF_fit>(positions, m_finalNorm);
	for (auto& p : m_finalPos)
	{
		hrbf_core->query(p,m_ret,m_grad);
		m_ret = compactSupport(m_ret, m_r);
		//printIntegerValueHRBF(m_ret);
		m_originalRet.push_back(m_ret);
		
	}
	// for (auto& ret : m_originalRet) {
    //     MGlobal::displayInfo(MString("Stored val: ") + ret);
    // }
	
}

// Warning: hrbf value is: 0.500001
// Warning: hrbf value is: 0.5
// Warning: hrbf value is: 0.500002
// Warning: hrbf value is: 0.500018
// Warning: hrbf value is: 0.500003
// Warning: hrbf value is: 0.500002
// Warning: hrbf value is: 0.499996
// Warning: hrbf value is: 0.500014
// Warning: hrbf value is: 0.500006
// Warning: hrbf value is: 0.499993
