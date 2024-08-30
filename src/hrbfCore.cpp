#include "hrbfCore.h"

//https://rodolphe-vaillant.fr/entry/12/recipe-for-implicit-surface-reconstruction-with-hrbf


HRBF_fit::HRBF_fit(std::vector<MVector>& points, std::vector<MVector>& normals)
{

    int nb_points = points.size();
    int nb_hrbf_constraints = 4 * nb_points;
    int nb_coeffs = nb_hrbf_constraints;

    _node_centers.resize(3, nb_points);
    _betas.resize(3, nb_points);
    _alphas.resize(nb_points);

    MatrixXX  D(nb_hrbf_constraints, nb_coeffs);
    VectorX   f(nb_hrbf_constraints);
    VectorX   x(nb_coeffs);
    
    for (int i = 0; i < nb_points; ++i)
    {
        _node_centers.col(i)[0] = points[i].x;
        _node_centers.col(i)[1] = points[i].y;
        _node_centers.col(i)[2] = points[i].z;
    }
    Vector p;
    Vector n;

    for (int i = 0; i < nb_points; ++i)
    {
        p[0] = points[i].x;
        p[1] = points[i].y;
        p[2] = points[i].z;
        n[0] = normals[i].x;
        n[1] = normals[i].y;
        n[2] = normals[i].z;
        int io = 4 * i;
        f(io) = 0;
        f.template segment<3>(io + 1) = n;
        
        for (int j = 0; j < nb_points; ++j)
        {

            int jo = 4 * j;
            Vector diff = p - _node_centers.col(j);
            double l = diff.norm();
            if (l == 0)
            {
                D.template block<4, 4>(io, jo).setZero();

            }
            else
            {
                double w = l*l*l;
                double dw_l = 3.0 * l;;
                double ddw = 6.0 * l;
                Vector g = diff * dw_l;
                D(io, jo) = w;
                D.row(io).template segment<3>(jo + 1) = g.transpose();
                D.col(jo).template segment<3>(io + 1) = g;
                D.template block<3, 3>(io + 1, jo + 1) = (ddw - dw_l) / (l * l) * (diff * diff.transpose());
                D.template block<3, 3>(io + 1, jo + 1).diagonal().array() += dw_l;
            }
        }
    }
    x = D.lu().solve(f);
    Eigen::Map< Eigen::Matrix<double, 4, Eigen::Dynamic> > mx(x.data(), 4, nb_points);
    _alphas = mx.row(0);
    _betas = mx.template bottomRows<3>();
    }

HRBF_fit::~HRBF_fit() 
{
}

float HRBF_fit::eval(float x, float y, float z) 
{

    float ret = 0.0f;
    Vector p;
	p[0] = x;
	p[1] = y;
	p[2] = z;

    int nb_nodes = _node_centers.cols();

    for (int i = 0; i < nb_nodes; ++i)
    {
        Vector diff = p - _node_centers.col(i);
        double l = diff.norm();

        if (l > 0)
        {
            ret += _alphas(i) * l * l * l; 
			ret += _betas.col(i).dot(diff) * 3.0 * l;
        }
    }
    return ret;
} 

MVector HRBF_fit::grad(float x, float y, float z) 
{   
    Vector p;
	p[0] = x;
	p[1] = y;
	p[2] = z;

    Vector grad = Vector::Zero();
    int nb_nodes = _node_centers.cols();
    for (int i = 0; i < nb_nodes; i++)
    {
        Vector node = _node_centers.col(i);
        Vector beta = _betas.col(i);
        double  alpha = _alphas(i);
        Vector diff = p - node;
        Vector diffNormalized = diff;
        double l = diff.norm();

        if (l > 0.00001f)
        {
            diffNormalized.normalize();
            double dphi = 3.0 * l * l;
            double ddphi = 6.0 * l;
            double alpha_dphi = alpha * dphi;
            double bDotd_l = beta.dot(diff) / l;
            double squared_l = diff.squaredNorm();
            grad += alpha_dphi * diffNormalized;
            grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l;
        }
    }
    return MVector(grad[0], grad[1], grad[2]);
}

//MPoint float MVector
void HRBF_fit::query(MPoint& point, float& value, float& gradX, float& gradY, float& gradZ) {
    Vector p(point.x, point.y, point.z); // 使用构造函数直接创建
    value = eval(p(0), p(1), p(2));
    MVector gradient = grad(p(0), p(1), p(2));
    gradX = static_cast<float>(gradient(0));
    gradY = static_cast<float>(gradient(1));
    gradZ = static_cast<float>(gradient(2));

}
void HRBF_fit::query(MPoint& point, float& value, MVector& _grad)  {
    Vector p;
    p[0] = point.x;
	p[1] = point.y;
	p[2] = point.z;
    value = eval(p[0], p[1], p[2]);
    _grad = grad(p[0], p[1], p[2]);
}
