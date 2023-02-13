/*
 * Author: Burak Aksoy
 */

#include "fabric_simulator/utilities/cloth.h"

using namespace pbd_object;

Cloth::Cloth(){

}

Cloth::Cloth(const Mesh &mesh, const double &bending_compliance, const double &density):
    mesh_(mesh),
    bending_compliance_(bending_compliance),
    density_(density)
{
    stretching_compliance_ = 1.0/100000 ; //0.0;

    num_particles_ = mesh_.vertices.rows();
    
    pos_ = mesh_.vertices;
    prev_pos_ = mesh_.vertices;
    rest_pos_ = mesh_.vertices;
    vel_ = Eigen::MatrixX3d::Zero(num_particles_,3);
    inv_mass_ = Eigen::RowVectorXd::Zero(num_particles_);
    
    grads_ = Eigen::RowVector3d::Zero();

    // std::cout << "pos_:\n" << pos_ << std::endl;
    // std::cout << "prev_pos_:\n" << prev_pos_ << std::endl;
    // std::cout << "vel_:\n" << vel_ << std::endl;
    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;

    
    // Set stretching and bending constraints
    Eigen::RowVectorXi neighbors = findTriNeighbors(mesh_.face_tri_ids);
    int num_tris = mesh_.face_tri_ids.rows();

    std::vector<Eigen::RowVector2i> edge_ids;
    std::vector<Eigen::RowVector4i> tri_pair_ids;
    for (int i = 0; i < num_tris; i++) {
        for (int j = 0; j < 3; j++) {
            int global_edge_nr = 3 * i + j;
            // ids of particle creating that global_edge:
            int id0 = mesh_.face_tri_ids(i,j);
            int id1 = mesh_.face_tri_ids(i,(j + 1) % 3);

            // Each edge only once
            int n = neighbors[global_edge_nr]; // returns -1 if open edge, or positive global edge number of its pair
            if (n < 0 || id0 < id1) {
                Eigen::RowVector2i ids(id0, id1);
                edge_ids.push_back(ids);
            }

            // Tri pair
            if (n >= 0 && id0 < id1) {
                int ni = n / 3;
                int nj = n % 3;
                int id2 = mesh_.face_tri_ids(i,(j + 2) % 3);
                int id3 = mesh_.face_tri_ids(ni,(nj + 2) % 3);
                Eigen::RowVector4i ids(id0, id1, id2, id3);
                tri_pair_ids.push_back(ids); // simple bending constraint needs only id2 and id3, but id0 and id1 also added for a "future" implementation
            }
        }
    }

    // Eigen::Map<Eigen::MatrixX2i> edge_ids_mat((int *)edge_ids.data(), edge_ids.size(), 2);
    Eigen::MatrixXi edge_ids_mat(edge_ids.size(),2);
    for (int i = 0; i < edge_ids.size(); i++) {
        edge_ids_mat.row(i) = edge_ids[i];
    }

    // Eigen::Map<Eigen::MatrixX4i> tri_pair_ids_mat((int *)tri_pair_ids.data(), tri_pair_ids.size(), 4);
    Eigen::MatrixXi tri_pair_ids_mat(tri_pair_ids.size(),4);
    for (int i = 0; i < tri_pair_ids.size(); i++) {
        tri_pair_ids_mat.row(i) = tri_pair_ids[i];
    }

    stretching_ids_ = edge_ids_mat;
    bending_ids_ = tri_pair_ids_mat;

    std::cout << "stretching_ids_:\n" << stretching_ids_ << std::endl;
    std::cout << "bending_ids_:\n" << bending_ids_ << std::endl;

    stretching_lengths_ = Eigen::RowVectorXd::Zero(stretching_ids_.rows()); //assigned at initPhysics
    bending_lengths_ = Eigen::RowVectorXd::Zero(bending_ids_.rows()); //assigned at initPhysics

    // Not necessary? 
    // attached_ids_
    // only_once 

    initPhysics(mesh_.face_tri_ids);
}

Cloth::~Cloth(){

}

void Cloth::initPhysics(const Eigen::MatrixX3i &face_tri_ids){
    int num_tris = face_tri_ids.rows();

    for (int i = 0; i < num_tris; i++) {
        int id0 = face_tri_ids(i,0);
        int id1 = face_tri_ids(i,1);
        int id2 = face_tri_ids(i,2);

        Eigen::RowVector3d e0 = pos_.row(id1) - pos_.row(id0);
        Eigen::RowVector3d e1 = pos_.row(id2) - pos_.row(id0);
        Eigen::RowVector3d c = e0.cross(e1);

        double A = 0.5 * c.norm();  // area
        double mass = A * density_;
    
        if (mass > 0.0) {
            double p_mass = (mass / 3.0); // divide by 3 because we have 3 vertices per triangle

            double p_0_mass = ( inv_mass_(id0) > 0.0 ) ? 1.0/inv_mass_(id0) : 0.0;
            double p_1_mass = ( inv_mass_(id1) > 0.0 ) ? 1.0/inv_mass_(id1) : 0.0;
            double p_2_mass = ( inv_mass_(id2) > 0.0 ) ? 1.0/inv_mass_(id2) : 0.0;

            p_0_mass += p_mass;
            p_1_mass += p_mass;
            p_2_mass += p_mass;

            inv_mass_(id0) = 1.0/p_0_mass;
            inv_mass_(id1) = 1.0/p_1_mass;
            inv_mass_(id2) = 1.0/p_2_mass;
        }
    }

    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;
    std::cout << "Total fabric mass:\n" << inv_mass_.cwiseInverse().sum() << " kg." << std::endl;

    

    for (int i = 0; i < stretching_lengths_.size(); i++){
        int id0 = stretching_ids_(i,0);
        int id1 = stretching_ids_(i,1);
        stretching_lengths_(i) = (pos_.row(id1)-pos_.row(id0)).norm();
    }

    for (int i = 0; i < bending_lengths_.size(); i++){
        int id0 = bending_ids_(i,2);
        int id1 = bending_ids_(i,3);
        bending_lengths_(i) = (pos_.row(id1)-pos_.row(id0)).norm();
    }

    // std::cout << "stretching_lengths_:\n" << stretching_lengths_ << std::endl;
    // std::cout << "bending_lengths_:\n" << bending_lengths_ << std::endl;

    hangFromCorners();

    // std::cout << "inv_mass_ after hang:\n" << inv_mass_ << std::endl;
}

Eigen::RowVectorXi Cloth::findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids){
    int num_tris = face_tri_ids.rows();

    // create common edges
    std::vector<std::unordered_map<std::string, int>> edges;
    for (int i = 0; i < num_tris; i++) {
        for (int j = 0; j < 3; j++) {
            int id0 = face_tri_ids(i,j);
            int id1 = face_tri_ids(i,(j + 1) % 3);
            edges.push_back({
                {"id0", std::min(id0, id1)},
                {"id1", std::max(id0, id1)},
                {"edgeNr", 3 * i + j}
            });
        }
    }

    // sort so common edges are next to each other
    std::sort(edges.begin(), edges.end(), [](const std::unordered_map<std::string, int> &a, const std::unordered_map<std::string, int> &b) {
        return a.at("id0") < b.at("id0") || (a.at("id0") == b.at("id0") && a.at("id1") < b.at("id1"));
    });

    // find matching edges
    const int fill_value = -1;
    Eigen::RowVectorXi neighbors = Eigen::RowVectorXi::Constant(3*num_tris, fill_value);

    int nr = 0;
    while (nr < edges.size()) {
        std::unordered_map<std::string, int> e0 = edges[nr]; // unordered_map
        nr++;
        if (nr < edges.size()) {
            std::unordered_map<std::string, int> e1 = edges[nr]; // unordered_map
            if (e0.at("id0") == e1.at("id0") && e0.at("id1") == e1.at("id1")) {
                neighbors(e0.at("edgeNr")) = e1.at("edgeNr");
                neighbors(e1.at("edgeNr")) = e0.at("edgeNr");
            }
        }
    }
    
    return neighbors;
}


// Find the nearest 3D position vector row id in the given matrix
int Cloth::findNearestPositionVectorId(const Eigen::MatrixXd& matrix, const Eigen::Vector3d& pos) {
  int nearestId = -1;
  double minDistance = std::numeric_limits<double>::max();
  for (int i = 0; i < matrix.rows(); ++i) {
    Eigen::Vector3d currentPos = matrix.row(i);
    double currentDistance = (currentPos - pos).norm();
    if (currentDistance < minDistance) {
      nearestId = i;
      minDistance = currentDistance;
    }
  }
  return nearestId;
}

void Cloth::solveStretching(const double &compliance, const double &dt){
    double alpha = compliance / (dt*dt);

    for (int i = 0; i < stretching_lengths_.size(); i++){
        int id0 = stretching_ids_(i,0);
        int id1 = stretching_ids_(i,1);

        double w0 = inv_mass_(id0);
        double w1 = inv_mass_(id1);
        double w = w0 + w1;

        if (w == 0){
            continue;
        }

        grads_ = pos_.row(id0) - pos_.row(id1);

        double len = grads_.norm();
        if (len == 0){
            continue;
        }

        grads_ = grads_ / len;

        // std::cout << "grads_: " << grads_ << std::endl;

        double rest_len = stretching_lengths_(i);
        double C = len - rest_len;
        double s = -C / (w+alpha);

        pos_.row(id0) += s*w0*grads_;
        pos_.row(id1) += -s*w1*grads_;
    }
}

void Cloth::solveBending(const double &compliance, const double &dt){
    double alpha = compliance / (dt*dt);

    for (int i = 0; i < bending_lengths_.size(); i++){
        int id0 = bending_ids_(i,2);
        int id1 = bending_ids_(i,3);

        double w0 = inv_mass_(id0);
        double w1 = inv_mass_(id1);
        double w = w0 + w1;

        if (w == 0){
            continue;
        }

        grads_ = pos_.row(id0) - pos_.row(id1);

        double len = grads_.norm();
        if (len == 0){
            continue;
        }

        grads_ = grads_ / len;

        double rest_len = bending_lengths_(i);
        double C = len - rest_len;
        double s = -C / (w+alpha);

        pos_.row(id0) += s*w0*grads_;
        pos_.row(id1) += -s*w1*grads_;
    }
}

void Cloth::hangFromCorners(){
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < num_particles_; i++) {
        min_x = std::min(min_x, pos_(i,0));
        max_x = std::max(max_x, pos_(i,0));
        max_y = std::max(max_y, pos_(i,1));
    }

    double eps = 0.0001;

    for (int i = 0; i < num_particles_; i++) {
        double x = pos_(i,0);
        double y = pos_(i,1);
        if (y > max_y - eps && (x < min_x + eps || x > max_x - eps)) {
            inv_mass_(i) = 0.0;
        }
    }
}

void Cloth::preSolve(const double &dt, const Eigen::RowVector3d &gravity){
    for (int i = 0; i< num_particles_; i++){
        if (inv_mass_(i) > 0){
            // std::cout << "i :" << i << std::endl;
            // std::cout << "pos_.row(i): " << pos_.row(i) << std::endl;
            // std::cout << "prev_pos_.row(i): " << prev_pos_.row(i) << std::endl;
            // std::cout << "vel_.row(i): " << vel_.row(i) << std::endl;

            vel_.row(i) += gravity*dt;
            prev_pos_.row(i) = pos_.row(i);
            pos_.row(i) += vel_.row(i)*dt;

            // Prevent going below ground
            double z = pos_(i,2);
            if (z < 0.){
                pos_.row(i) = prev_pos_.row(i) ;
                pos_(i,2) = 0.0;
            }

            
            // std::cout << "i :" << i << std::endl;
            // std::cout << "pos_.row(i): " << pos_.row(i) << std::endl;
            // std::cout << "prev_pos_.row(i): " << prev_pos_.row(i) << std::endl;
            // std::cout << "vel_.row(i): " << vel_.row(i) << std::endl;

            // std::string s;
            // std::cin >> s;
        }
    }
}

void Cloth::solve(const double &dt){
    solveStretching(stretching_compliance_,dt);
    solveBending(bending_compliance_,dt);
}

void Cloth::postSolve(const double &dt){
    for (int i = 0; i< num_particles_; i++){
        if (inv_mass_(i) != 0){
            vel_.row(i) = (pos_.row(i) - prev_pos_.row(i))/dt;
        }
    }
}

int Cloth::attachNearest(const Eigen::RowVector3d &pos){
    int id = findNearestPositionVectorId(pos_,pos);
    // Make that particle stationary
    if (id >= 0){
        inv_mass_(id) = 0.0;
    }
    return id;
}

void Cloth::updateAttachedPose(const int &id, const Eigen::RowVector3d &pos){
    pos_.row(id) = pos;
}

Eigen::MatrixX3d *Cloth::getPosPtr(){
    return &pos_;
}

Eigen::MatrixX3d *Cloth::getVelPtr(){
    return &vel_;
}

Eigen::RowVectorXd *Cloth::getStretchingLengthsPtr(){
    return &stretching_lengths_;
}

Eigen::RowVectorXd *Cloth::getBendingLengthsPtr(){
    return &bending_lengths_;
}

Eigen::MatrixX2i *Cloth::getStretchingIdsPtr(){
    return &stretching_ids_;
}

Eigen::MatrixX4i *Cloth::getBendingIdsPtr(){
    return &bending_ids_;
}