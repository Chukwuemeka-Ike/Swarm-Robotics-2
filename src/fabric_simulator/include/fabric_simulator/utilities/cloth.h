/*
 * Author: Burak Aksoy
 */

#ifndef CLOTH_H
#define CLOTH_H

#include <math.h>
#include <numeric>
#include <vector>
#include <unordered_map> 
#include <algorithm>    // std::sort

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <scipy/spatial.h>

#include <string>
#include <iostream>

namespace pbd_object
{

struct Mesh
{
    std::string name;
    Eigen::MatrixX3d vertices;
    Eigen::MatrixX3i face_tri_ids;
};

class Cloth
{
public:
    Cloth();
    Cloth(const Mesh &mesh, const double &bending_compliance, const double &density);
    ~Cloth();

    void preSolve(const double &dt, const Eigen::RowVector3d &gravity);
    void solve(const double &dt);
    void postSolve(const double &dt);

    int attachNearest(const Eigen::RowVector3d &pos);
    void updateAttachedPose(const int &id, const Eigen::RowVector3d &pos);

    // Eigen::MatrixX2i getStretchingIds();
    // Eigen::MatrixX4i getBendingIds();

    // Eigen::RowVectorXd getStretchingLengths();
    // Eigen::RowVectorXd getBendingLengths();

    // Eigen::MatrixX3d getPos();
    // Eigen::MatrixX3d getVel();

    Eigen::MatrixX2i *getStretchingIdsPtr();
    Eigen::MatrixX4i *getBendingIdsPtr();

    Eigen::RowVectorXd *getStretchingLengthsPtr();
    Eigen::RowVectorXd *getBendingLengthsPtr();

    Eigen::MatrixX3d *getPosPtr();
    Eigen::MatrixX3d *getVelPtr();

private:
    // Functions
    void initPhysics(const Eigen::MatrixX3i &face_tri_ids);
    Eigen::RowVectorXi findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids);

    int findNearestPositionVectorId(const Eigen::MatrixXd& matrix, const Eigen::Vector3d& pos);
    void solveStretching(const double &compliance, const double &dt);
    void solveBending(const double &compliance, const double &dt);

    void hangFromCorners();

    // Variables
    Mesh mesh_;
    int num_particles_;

    Eigen::MatrixX3d pos_;
    Eigen::MatrixX3d prev_pos_;
    Eigen::MatrixX3d rest_pos_;
    Eigen::MatrixX3d vel_;
    
    Eigen::RowVectorXd inv_mass_;
    double density_; // fabric mass per meter square (kg/m^2)
    Eigen::RowVector3d grads_;

    Eigen::MatrixX2i stretching_ids_;
    Eigen::MatrixX4i bending_ids_;
    Eigen::RowVectorXd stretching_lengths_;
    Eigen::RowVectorXd bending_lengths_;

    double stretching_compliance_;
    double bending_compliance_;

    // May not be necessary?
    // Eigen::RowVectorXi attached_ids_; // ids of robot attached particles
    // int grab_id_;
    // double grab_inv_mass_;
    
    // to debug:
    // int only_once;
};

} // namespace pbd_object

#endif /* !CLOTH_H */