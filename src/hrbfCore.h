#ifndef HRBF_FIT
#define HRBF_FIT
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <vector>
#include <maya/MVector.h>

class HRBF_fit
{
public:
    typedef Eigen::Matrix<double, 3, 1>                         Vector;
    typedef Eigen::Matrix<double, 3, Eigen::Dynamic>            MatrixDX;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1>              VectorX; 
    HRBF_fit(std::vector<MVector>& points,
        std::vector<MVector>& normals);
    ~HRBF_fit();
    float eval(float x, float y, float z);
    MVector grad(float x, float y, float z);
    void query(MPoint& point, float& value, float& gradX,float& gradY, float& gradZ) ;
    void query(MPoint& point, float& value, MVector& _grad) ;
    /// Each column represents p_i:  VectorX pi = _node_centers.col(i);
    MatrixDX  _node_centers;
    /// Vector of scalar values alpha
    VectorX   _alphas;
    /// Each column represents beta_i:  VectorX bi = _betas.col(i);
    MatrixDX  _betas;


};

#endif 

