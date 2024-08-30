#include "HRBFManager.h"
#include <maya/MGlobal.h>
HRBFManager::HRBFManager()
{
}
HRBFManager::~HRBFManager()
{
}
void HRBFManager::setBonesRelationship(std::vector<int>& bonesList)
{
    for (int bIndex = 1; bIndex < bonesList.size(); bIndex++)
    {
        m_HRBFs[bIndex]->m_parent = m_HRBFs[bIndex-1].get();
        m_HRBFs[bIndex-1]->m_childrens.push_back(m_HRBFs[bIndex].get());

    }
}
void HRBFManager::printMaxFirstHRBFValuesAndGradients()
{
    MString info;
    for (int i = 0; i < maxfirstHRBFValuesAndGradients.size(); i++) 
	{
        float hrbfValue = std::get<0>(maxfirstHRBFValuesAndGradients[i]);
		float _radius = std::get<2>(maxfirstHRBFValuesAndGradients[i]);
        info += "Index ";
        info += i;
        info += ": HRBF Value = ";
        info += hrbfValue;
		info += "radius = ";
		info += _radius;
        info += "\n";
    }
    MGlobal::displayInfo(info);
}
//transform is local
void HRBFManager::buildHRBFs(std::vector<int>& jointList,
	MMatrixArray& binds, MMatrixArray& transforms,
	MItGeometry& iter) 
{
	int m_numJoints = jointList.size();
	iter.reset();
    std::vector<int> m_isoVals;
	MPoint pos;
	MVector norm;
	//printIntegerValueHRBFManager(m_numJoints);
	for (int i = 0; i < m_numJoints; i++) {
		m_HRBFs.push_back(std::make_unique<HRBF>(binds[i]));
	}
	setBonesRelationship(jointList);
	for (; !iter.isDone(); iter.next()) {
		pos = iter.position();
        norm = iter.normal();
		for (int i = 0; i < m_numJoints; i++) 
		{
			m_HRBFs[i]->setBonesPN();
			m_HRBFs[i]->cullingVertex(pos, norm);
			//里面需要算t_r(d_i(x))+ compact support
			
		}
	}
	for (int i = 0; i < m_numJoints; i++) 
	{
		m_HRBFs[i]->compute();
	}
	unionFirstField(binds);
}
//传进来的是转local的
void HRBFManager::unionFirstField(MMatrixArray& binds)
{
	int m_numJoints = binds.length();
	maxfirstHRBFValuesAndGradients.resize(m_HRBFs[0]->m_finalPos.size(), std::make_tuple(0.0f, MVector(0.0f, 0.0f, 0.0f), 0.0f)); // 假设所有HRBF控制的顶点数量相同
	for (int i = 0; i < m_numJoints; i++) {
        float Grad_length = 0.0f;
        float currVal;
        MVector currGrad;
		for (size_t j = 0; j < m_HRBFs[i]->m_finalPos.size(); j++) {
            MPoint localSample =m_HRBFs[i]->m_finalPos[j];
			//MPoint float MVector
            m_HRBFs[i]->hrbf_core->query(localSample, currVal, currGrad);

			float& maxVal = std::get<0>(maxfirstHRBFValuesAndGradients[j]);
            MVector& maxGrad = std::get<1>(maxfirstHRBFValuesAndGradients[j]);
            float& maxR = std::get<2>(maxfirstHRBFValuesAndGradients[j]);
            float currGrad_length = currGrad.length();
			if (currVal > maxVal)
			{
                maxVal = currVal;
				maxR = m_HRBFs[i]->m_r;
                
            }
			if (currGrad.length() > maxGrad.length())
			{
                maxGrad = currGrad;
            }
		}
	
	}
	 for (auto& item : maxfirstHRBFValuesAndGradients) {
	 //because it's reference, so change it means change that
        float& val = std::get<0>(item);
        float r = std::get<2>(item);
		if(r == 0.0f)
		{
		val = 0.5f;
		}
		else{
		val = m_HRBFs[0]->compactSupport(val, r);
		}
    }

	printMaxFirstHRBFValuesAndGradients();
}

void HRBFManager::unionSeconeField(MMatrixArray& transforms)
{
	int m_numJoints = transforms.length();
	maxsecondHRBFValuesAndGradients.resize(m_HRBFs[0]->m_finalPos.size(),std::make_tuple(0.0f, MVector(0.0f, 0.0f, 0.0f), 0.0f)); 
		for (int i = 0; i < m_numJoints; i++) 
		{
        float Grad_length = 0.0f;
        float currVal;
        MVector currGrad;
		for (size_t j = 0; j < m_HRBFs[i]->m_finalPos.size(); j++) {
            MPoint localSample =m_HRBFs[i]->m_finalPos[j]* transforms[i].inverse();
			//MPoint float MVector
            m_HRBFs[i]->hrbf_core->query(localSample, currVal, currGrad);
			float& maxVal = std::get<0>(maxsecondHRBFValuesAndGradients[j]);
            MVector& maxGrad = std::get<1>(maxsecondHRBFValuesAndGradients[j]);
            float& maxR = std::get<2>(maxsecondHRBFValuesAndGradients[j]);
            float currGrad_length = currGrad.length();
			if (currVal > maxVal)
			{
                maxVal = currVal;
                maxR = m_HRBFs[i]->m_r;
            }
			if (currGrad.length() > maxGrad.length())
			{
                maxGrad = currGrad;
            }
		}
	
	}
	for (auto& item : maxsecondHRBFValuesAndGradients) {
	 //because it's reference, so change it means change that
        float& val = std::get<0>(item);
        float r = std::get<2>(item);
		if(r == 0.0f)
		{
		val = 0.5f;
		}
		else{
		val = m_HRBFs[0]->compactSupport(val, r);
		}
        
    }
}
void HRBFManager::printCorrection(int&vertexIndex,MVector& correction)
{
	MString msg = "the correction in ";
	msg += vertexIndex;
	msg += " is: (";
	msg += correction.x;
	msg += ", ";
	msg += correction.y;
	msg += ", ";
	msg += correction.z;
	msg += ")";
	MGlobal::displayInfo(msg);
}

void HRBFManager::adjustVertices(MItGeometry& iter) {
    MPoint pos;
    MVector norm;
    MVector correction;
    int vertexIndex = 0;

    for (iter.reset(); !iter.isDone(); iter.next(), vertexIndex++) {
        pos = iter.position();
        norm = iter.normal();
        norm.normalize();

        float initialHRBF = std::get<0>(maxfirstHRBFValuesAndGradients[vertexIndex]);
        float updatedHRBF = std::get<0>(maxsecondHRBFValuesAndGradients[vertexIndex]);
        MVector updatedGrad =std::get<1>(maxsecondHRBFValuesAndGradients[vertexIndex]);

        // 计算HRBF值的差异
        float deltaHRBF = updatedHRBF - initialHRBF;

        // 计算梯度长度
        float gradLength = updatedGrad.length();
        if (gradLength > 0.0001) {  // 避免除以零
            correction = 0.35f * deltaHRBF * (updatedGrad / (gradLength * gradLength));
            pos += correction;  // 应用位置修正
			printCorrection(vertexIndex,correction);
        }

        iter.setPosition(pos);  // 更新顶点位置
    }
}



