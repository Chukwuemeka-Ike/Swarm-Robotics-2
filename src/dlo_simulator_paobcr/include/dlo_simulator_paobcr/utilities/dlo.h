/*
 * Author: Burak Aksoy
 */

#ifndef DLO_H
#define DLO_H

#include <math.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_map> 
#include <algorithm>    // std::sort

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <scipy/spatial.h>

#include <string>
#include <iostream>

#include <float.h>

#define USE_DOUBLE // comment out if you would like to use float.

#ifdef USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif

namespace pbd_object
{

struct MeshDLO
{
    std::string name;
    std::vector<Eigen::Matrix<Real,3,1>> vertices;
    std::vector<Eigen::Quaternion<Real>> quaternions;
};

class Dlo
{
public:
    Dlo();
    Dlo(const MeshDLO &mesh, 
        const Real &stretching_compliance, 
        const Real &shearing_compliance_x, 
        const Real &shearing_compliance_y, 
        const Real &twisting_compliance, 
        const Real &bending_compliance_x, 
        const Real &bending_compliance_y, 
        const Real &density,
        const Real &radius);
    ~Dlo();

    void preSolve(const Real &dt, const Eigen::Matrix<Real,3,1> &gravity);
    void solve(const Real &dt);
    void postSolve(const Real &dt);

    void hangFromCorners(const int &num_corners);

    /*
    int attachNearest(const Eigen::Matrix<Real,3,1> &pos);
    void updateAttachedPose(const int &id, const Eigen::Matrix<Real,3,1> &pos);

    void resetForces();
    */

    Eigen::Matrix2Xi *getStretchShearIdsPtr();
    Eigen::Matrix2Xi *getBendTwistIdsPtr();

    std::vector<Eigen::Matrix<Real,3,1>> *getPosPtr();
    Eigen::Matrix<Real,3,Eigen::Dynamic> *getVelPtr();
    Eigen::Matrix<Real,3,Eigen::Dynamic> *getForPtr();

    std::vector<int> *getAttachedIdsPtr();    

private:
    // Functions
    void setStretchShearConstraints();
    void setBendTwistConstraints();

    void setMasses();

    /*
    int findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, const Eigen::Matrix<Real,3,1>& pos);
    */
   
    void solveStretchShearConstraints(const Real &dt);
    void solveBendTwistConstraints(const Real &dt);

    // Variables
    MeshDLO mesh_;
    
    Real density_; // dlo mass per meter cube (kg/m^3)
    Real radius_; // radius of dlo assuming it's cylndrical

    int num_particles_;
    int num_quaternions_;

    // Particle Data
    std::vector<Eigen::Matrix<Real,3,1>> pos_;
    std::vector<Eigen::Matrix<Real,3,1>> prev_pos_;
    std::vector<Eigen::Matrix<Real,3,1>> rest_pos_;
    Eigen::Matrix<Real,3,Eigen::Dynamic> vel_;
    Eigen::Matrix<Real,3,Eigen::Dynamic> for_;
    std::vector<Real> inv_mass_;

    // Orientation Data
    std::vector<Eigen::Quaternion<Real>> ori_;
    std::vector<Eigen::Quaternion<Real>> prev_ori_;
    std::vector<Eigen::Quaternion<Real>> rest_ori_;
    Eigen::Matrix<Real,3,Eigen::Dynamic> omega_;
    Eigen::Matrix<Real,3,Eigen::Dynamic> tor_;
    std::vector<Eigen::Matrix<Real,3,3>> inv_iner_;
    std::vector<Eigen::Matrix<Real,3,3>> iner_;

    Eigen::Matrix2Xi stretchShear_ids_;
    std::vector<Real> stretchShear_restLengths_;

    Eigen::Matrix2Xi bendTwist_ids_;
    std::vector<Eigen::Quaternion<Real>> bendTwist_restDarbouxVectors_;

    Real stretching_compliance_;
    Real shearing_compliance_x_;
    Real shearing_compliance_y_;
    Real twisting_compliance_;
    Real bending_compliance_x_;
    Real bending_compliance_y_;

    std::vector<int> attached_ids_; // ids of robot attached particles
    // int grab_id_;
    // Real grab_inv_mass_;
};

} // namespace pbd_object

#endif /* !DLO_H */