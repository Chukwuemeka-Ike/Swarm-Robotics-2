/*
 * Author: Burak Aksoy
 */

#include "dlo_simulator_paobcr/utilities/dlo.h"

using namespace pbd_object;

const Real eps_ = static_cast<Real>(1e-6);

Dlo::Dlo(){

}

Dlo::Dlo(const MeshDLO &mesh, 
        const Real &stretching_compliance, 
        const Real &shearing_compliance_x, 
        const Real &shearing_compliance_y, 
        const Real &twisting_compliance, 
        const Real &bending_compliance_x, 
        const Real &bending_compliance_y, 
        const Real &density,
        const Real &radius):
    mesh_(mesh),
    stretching_compliance_(stretching_compliance),
    shearing_compliance_x_(shearing_compliance_x),
    shearing_compliance_y_(shearing_compliance_y),
    twisting_compliance_(twisting_compliance),
    bending_compliance_x_(bending_compliance_x),
    bending_compliance_y_(bending_compliance_y),
    density_(density),
    radius_(radius)
{
    num_particles_ = mesh_.vertices.size();
    num_quaternions_ = mesh_.quaternions.size();

    std::cout << "num particles: " << num_particles_ << std::endl;
    std::cout << "num quaternions: " << num_quaternions_ << std::endl;
    
    // Particle data
    pos_ = mesh_.vertices;
    prev_pos_ = mesh_.vertices;
    rest_pos_ = mesh_.vertices;
    vel_ = Eigen::Matrix<Real,3,Eigen::Dynamic>::Zero(3,num_particles_); // Create a velocity vector of particles filled with zeros 
    for_ = Eigen::Matrix<Real,3,Eigen::Dynamic>::Zero(3,num_particles_);
    inv_mass_.assign(num_particles_, 0.0);

    // std::cout << "pos_:\n" << pos_ << std::endl;
    // std::cout << "prev_pos_:\n" << prev_pos_ << std::endl;
    // std::cout << "vel_:\n" << vel_ << std::endl;
    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;
    
    // Orientation Data
    ori_ = mesh_.quaternions;
    prev_ori_ = mesh_.quaternions;
    rest_ori_ = mesh_.quaternions;
    omega_ = Eigen::Matrix<Real,3,Eigen::Dynamic>::Zero(3,num_quaternions_); // Create a angular velocity vector of orientations filled with zeros 
    tor_ = Eigen::Matrix<Real,3,Eigen::Dynamic>::Zero(3,num_quaternions_);
    inv_iner_.assign(num_quaternions_,Eigen::Matrix<Real,3,3>::Zero());
    iner_.assign(num_quaternions_,Eigen::Matrix<Real,3,3>::Zero());

    // std::cout << "ori_:\n" << ori_ << std::endl;
    // std::cout << "prev_ori_:\n" << prev_ori_ << std::endl;
    // std::cout << "omega_:\n" << omega_ << std::endl;
    // std::cout << "inv_iner_:\n" << inv_iner_ << std::endl;
    // std::cout << "iner_:\n" << iner_ << std::endl;

    setStretchShearConstraints();


    setBendTwistConstraints();


    setMasses();


    // attached_ids_  //Initially empty vector of integers to store the ids of attached (fixed) particles.
}

Dlo::~Dlo(){

}

void Dlo::setStretchShearConstraints(){
    /* 
    Specifies:
    - stretchShear_ids_ (from DLO mesh)
    - stretchShear_restLengths_

    Note:
    - stretching_compliance
    - shearing_compliance_x
    - shearing_compliance_y are also part of this constraint, however they are already set in the beginning of the constructor.
    */

    std::vector<Eigen::Vector2i> edge_ids;
    int num_edges = num_particles_-1;
    for(int id = 0; id < num_edges; id++){
        Eigen::Vector2i ids(id, id+1); // edge ids
        edge_ids.push_back(ids);
    }


    Eigen::MatrixXi edge_ids_mat(2,edge_ids.size());
    for (int i = 0; i < edge_ids.size(); i++) {
        edge_ids_mat.col(i) = edge_ids[i];
    }


    stretchShear_ids_ = edge_ids_mat;
    
    for (int i = 0; i < stretchShear_ids_.cols(); i++){
        int id0 = stretchShear_ids_(0,i);
        int id1 = stretchShear_ids_(1,i);

        Real rest_length = (pos_[id1]-pos_[id0]).norm();
        stretchShear_restLengths_.push_back(rest_length);
    }


    std::cout << "num edges: " << stretchShear_ids_.cols() << std::endl;
    // std::cout << "stretchShear_restLengths_:\n" << stretchShear_restLengths_ << std::endl;
}

void Dlo::setBendTwistConstraints(){
    /* 
    Specifies:
    - bendTwist_ids_ (from DLO mesh)
    - bendTwist_restDarbouxVectors_

    Note:
    - twisting_compliance
    - bending_compliance_x
    - bending_compliance_y are also part of this constraint, however they are already set in the beginning of the constructor.
    */

    std::vector<Eigen::Vector2i> quaternion_ids;
    for(unsigned int id = 0; id < num_quaternions_-1; id++){
        Eigen::Vector2i ids(id, id+1); // quaternion ids
        quaternion_ids.push_back(ids);
    }

    Eigen::MatrixXi quaternion_ids_mat(2,quaternion_ids.size());
    for (int i = 0; i < quaternion_ids.size(); i++) {
        quaternion_ids_mat.col(i) = quaternion_ids[i];
    }

    bendTwist_ids_ = quaternion_ids_mat;

    bendTwist_restDarbouxVectors_ ;

    for (int i = 0; i < bendTwist_ids_.cols(); i++){
        int id0 = bendTwist_ids_(0,i);
        int id1 = bendTwist_ids_(1,i);

        Eigen::Quaternion<Real> rest_darboux_vect = ori_[id0].conjugate() * ori_[id1]; 

        Eigen::Quaternion<Real> omega_plus, omega_minus;
        omega_plus.coeffs()  = rest_darboux_vect.coeffs() + Eigen::Quaternion<Real>(1, 0, 0, 0).coeffs();
        omega_minus.coeffs() = rest_darboux_vect.coeffs() - Eigen::Quaternion<Real>(1, 0, 0, 0).coeffs();
        if (omega_minus.squaredNorm() > omega_plus.squaredNorm())
            rest_darboux_vect.coeffs() *= -1.0;

        bendTwist_restDarbouxVectors_.push_back(rest_darboux_vect);
    }
}

void Dlo::setMasses(){
    /* 
    Specifies:
    - inv_mass_, using the edge rest lengths, DLO length, radius and density information.
    - inv_iner_,
    - iner_
    */

    for (int i = 0; i < stretchShear_restLengths_.size(); i++) {
        int id0 = stretchShear_ids_(0,i);
        int id1 = stretchShear_ids_(1,i);

        Real l = stretchShear_restLengths_[i]; // edge length
        Real V = M_PI*(radius_*radius_)*l;  // volume
        Real mass = V * density_; // edge mass
    
        if (mass > 0.0) {
            Real p_mass = (mass / 2.0); // divide by 2 because we have 2 particles per edge

            Real p_0_mass = ( inv_mass_[id0] > 0.0 ) ? 1.0/inv_mass_[id0] : 0.0;
            Real p_1_mass = ( inv_mass_[id1] > 0.0 ) ? 1.0/inv_mass_[id1] : 0.0;

            Real q_0_inertia_xx = ( inv_iner_[id0](0,0) > 0.0 ) ? 1.0/inv_iner_[id0](0,0) : 0.0;
            Real q_0_inertia_yy = ( inv_iner_[id0](1,1) > 0.0 ) ? 1.0/inv_iner_[id0](1,1) : 0.0;
            Real q_0_inertia_zz = ( inv_iner_[id0](2,2) > 0.0 ) ? 1.0/inv_iner_[id0](2,2) : 0.0;

            p_0_mass += p_mass;
            p_1_mass += p_mass;
            
            q_0_inertia_xx += (1./4.)*mass*(radius_*radius_) + (1./12.)*mass*(l*l);
            q_0_inertia_yy += (1./4.)*mass*(radius_*radius_) + (1./12.)*mass*(l*l);
            q_0_inertia_zz += 0.5*mass*(radius_*radius_);

            inv_mass_[id0] = 1.0/p_0_mass;
            inv_mass_[id1] = 1.0/p_1_mass;
            
            inv_iner_[id0](0,0) = 1.0/q_0_inertia_xx;
            inv_iner_[id0](1,1) = 1.0/q_0_inertia_yy;
            inv_iner_[id0](2,2) = 1.0/q_0_inertia_zz;

            iner_[id0](0,0) = q_0_inertia_xx;
            iner_[id0](1,1) = q_0_inertia_yy;
            iner_[id0](2,2) = q_0_inertia_zz;
        }
    }

    // To debug
    Eigen::Matrix<Real,1,Eigen::Dynamic> inv_mass_eigen(num_particles_);
    for (int i = 0; i < inv_mass_.size(); i++) {
        inv_mass_eigen(i) = inv_mass_[i];
    }
    // std::cout << "inv_mass_:\n" << inv_mass_eigen << std::endl;
    std::cout << "particle masses:\n" << inv_mass_eigen.cwiseInverse() << " kg." << std::endl;
    std::cout << "Total dlo mass:\n" << inv_mass_eigen.cwiseInverse().sum() << " kg." << std::endl;
}

void Dlo::hangFromCorners(const int &num_corners){
    // if num_corners = 0: Do not fix any corners, free fall
    // if num_corners = 1: Fix from 1 corners
    // if num_corners = 2: Fix from 2 (all) corners
    // if num_corners = else: Fix all corners


    Real min_x = std::numeric_limits<Real>::infinity();
    Real max_x = -std::numeric_limits<Real>::infinity();
    Real min_y = std::numeric_limits<Real>::infinity();
    Real max_y = -std::numeric_limits<Real>::infinity();

    for (int i = 0; i < num_particles_-1; i++) {
        min_x = std::min(min_x, pos_[i](0));
        max_x = std::max(max_x, pos_[i](0));
        min_y = std::min(min_y, pos_[i](1));
        max_y = std::max(max_y, pos_[i](1));
    }

    Real eps = 0.0001;

    for (int i = 0; i < num_particles_-1; i++) {
        Real x = pos_[i](0);
        Real y = pos_[i](1);

        switch(num_corners) {
            case 0:
                std::cout << "Did not virtually hang from any corners." << std::endl;
                return;
                // break;
            case 1:
                if (y > max_y - eps && x > max_x - eps) {
                    std::cout << "id: " << i << " is virtually hang as corner 1." << std::endl;
                    inv_mass_[i] = 0.0;
                    inv_mass_[i+1] = 0.0;
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            case 2:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    inv_mass_[i] = 0.0;
                    inv_mass_[i+1] = 0.0;
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            default:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    inv_mass_[i] = 0.0;
                    inv_mass_[i+1] = 0.0;
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
        }
    }
}

void Dlo::preSolve(const Real &dt, const Eigen::Matrix<Real,3,1> &gravity){


    // Semi implicit euler (position)
    for (int i = 0; i< num_particles_; i++){
        if (inv_mass_[i] > 0){
            vel_.col(i) += gravity*dt;
            prev_pos_[i] = pos_[i];
            pos_[i] += vel_.col(i)*dt;

            // Prevent going below ground
            Real z = pos_[i](2);
            if (z < 0.){
                pos_[i] = prev_pos_[i] ;
                pos_[i](2) = 0.0;
            }
        }
    }

     
    // Semi implicit euler (rotation)
    for (int i = 0; i< num_quaternions_; i++){
        if (!inv_iner_[i].isZero(0)){
            //assume zero external torque.
            Eigen::Matrix<Real,3,1> torque = Eigen::Matrix<Real,3,1>::Zero(); 
            

            // integration 
            omega_.col(i) += dt * inv_iner_[i] * (torque - (omega_.col(i).cross(iner_[i]*omega_.col(i))));


            Eigen::Quaternion<Real> angVelQ(0.0, omega_.col(i)(0), omega_.col(i)(1), omega_.col(i)(2));

            prev_ori_[i] = ori_[i];
            
            ori_[i].coeffs() += dt * 0.5 * (angVelQ * ori_[i]).coeffs();
            ori_[i].normalize();
        }
    }
}

void Dlo::solve(const Real &dt){
    solveStretchShearConstraints(dt);
    solveBendTwistConstraints(dt);
}

void Dlo::solveStretchShearConstraints(const Real &dt){
    Eigen::Matrix<Real,3,1> stretchingAndShearingKs;
    stretchingAndShearingKs[0] = 1.0/(shearing_compliance_x_+eps_);
    stretchingAndShearingKs[1] = 1.0/(shearing_compliance_y_+eps_);
    stretchingAndShearingKs[2] = 1.0/(stretching_compliance_+eps_);

    int i = 0;
    const int n = stretchShear_restLengths_.size(); // num. of constraints
    for (int itr = 0; itr < n; itr++){
        // Bilateral interleaving order
        if (n % 2 == 0) {
            (itr % 2 == 0) ? i = itr :  i = n-itr;
        } else {
            (itr % 2 == 0) ? i = itr :  i = n-itr-1;
        }

        // Correction vectors place holders
        Eigen::Matrix<Real,3,1> dp0;
        Eigen::Matrix<Real,3,1> dp1;
        Eigen::Quaternion<Real> dq0;

        
        // IDs corresponding to the edge
        const int& id0 = stretchShear_ids_(0,i);
        const int& id1 = stretchShear_ids_(1,i);
        const int& idq0 = id0;
        

        // Inverse masses of particles and orientation
        const Real& w0 = inv_mass_[id0];
        const Real& w1 = inv_mass_[id1];
        
        const Real& wq0 = (static_cast<Real>(1.0)/static_cast<Real>(3.0))*(inv_iner_[idq0](0,0) + inv_iner_[idq0](1,1) + inv_iner_[idq0](2,2)); 
        // Take avr inverse inertia for now. 


        // Current positions and orientation
        Eigen::Matrix<Real,3,1>& p0 = pos_[id0];
        Eigen::Matrix<Real,3,1>& p1 = pos_[id1];
        Eigen::Quaternion<Real>& q0 = ori_[idq0];
        // rest lenght of the edge
        Real& l = stretchShear_restLengths_[i]; 


        //third director d3 = q0 * e_3 * q0_conjugate
        Eigen::Matrix<Real,3,1> d3; 
        d3[0] = static_cast<Real>(2.0) * (q0.x() * q0.z() + q0.w() * q0.y());
        d3[1] = static_cast<Real>(2.0) * (q0.y() * q0.z() - q0.w() * q0.x());
        d3[2] = q0.w() * q0.w() - q0.x() * q0.x() - q0.y() * q0.y() + q0.z() * q0.z();


        Eigen::Matrix<Real,3,1> gamma = (p1 - p0) / l - d3;
	    gamma /= (w1 + w0) / l + wq0 * static_cast<Real>(4.0)*l + eps_;


        if (std::abs(stretchingAndShearingKs[0] - stretchingAndShearingKs[1]) < eps_ && std::abs(stretchingAndShearingKs[0] - stretchingAndShearingKs[2]) < eps_)	//all Ks are approx. equal
            for (int i = 0; i<3; i++) gamma[i] *= stretchingAndShearingKs[i];
        else //diffenent stretching and shearing Ks. Transform diag(Ks[0], Ks[1], Ks[2]) into world space using Ks_w = R(q0) * diag(Ks[0], Ks[1], Ks[2]) * R^T(q0) and multiply it with gamma
        {
            Eigen::Matrix<Real,3,3> R = q0.toRotationMatrix();
            gamma = (R.transpose() * gamma).eval();
            for (int i = 0; i<3; i++) gamma[i] *= stretchingAndShearingKs[i];
            gamma = (R * gamma).eval();
        }


        dp0 = w0 * gamma;
        dp1 =-w1 * gamma;


        Eigen::Quaternion<Real> q_e_3_bar(q0.z(), -q0.y(), q0.x(), -q0.w());	//compute q*e_3.conjugate (cheaper than quaternion product)
        dq0 = Eigen::Quaternion<Real>(0.0, gamma.x(), gamma.y(), gamma.z()) * q_e_3_bar;
        dq0.coeffs() *= static_cast<Real>(2.0) * wq0 * l;


        // Now apply the corrections
        if (w0 != 0.0)
			p0 += dp0;
		if (w1 != 0.0)
			p1 += dp1;
		if (wq0 != 0.0)
		{
			q0.coeffs() += dq0.coeffs();
			q0.normalize();
		}

    }
}

void Dlo::solveBendTwistConstraints(const Real &dt){
    Eigen::Matrix<Real,3,1> bendingAndTwistingKs;
    bendingAndTwistingKs[0] = 1.0/(bending_compliance_x_+eps_);
    bendingAndTwistingKs[1] = 1.0/(bending_compliance_y_+eps_);
    bendingAndTwistingKs[2] = 1.0/(twisting_compliance_+eps_);

    int i = 0;
    const int n = bendTwist_restDarbouxVectors_.size(); // num. of constraints
    for (int itr = 0; itr < n; itr++){
        // Bilateral interleaving order
        if (n % 2 == 0) {
            (itr % 2 == 0) ? i = itr :  i = n-itr;
        } else {
            (itr % 2 == 0) ? i = itr :  i = n-itr-1;
        }

        // Correction vectors place holders
        Eigen::Quaternion<Real> dq0;
        Eigen::Quaternion<Real> dq1;

        // IDs 
        const int& idq0 = bendTwist_ids_(0,i);
        const int& idq1 = bendTwist_ids_(1,i);

        // Inverse masses of orientations ( Take avr inverse inertia for now. )
        const Real& wq0 = (static_cast<Real>(1.0)/static_cast<Real>(3.0))*(inv_iner_[idq0](0,0) + inv_iner_[idq0](1,1) + inv_iner_[idq0](2,2)); 
        const Real& wq1 = (static_cast<Real>(1.0)/static_cast<Real>(3.0))*(inv_iner_[idq1](0,0) + inv_iner_[idq1](1,1) + inv_iner_[idq1](2,2)); 

        // Current orientations
        Eigen::Quaternion<Real>& q0 = ori_[idq0];
        Eigen::Quaternion<Real>& q1 = ori_[idq1];

        // rest darboux vector
        Eigen::Quaternion<Real>& restDarbouxVector = bendTwist_restDarbouxVectors_[i]; 

        Eigen::Quaternion<Real> omega = q0.conjugate() * q1;   //darboux vector

        Eigen::Quaternion<Real> omega_plus;
        omega_plus.coeffs() = omega.coeffs() + restDarbouxVector.coeffs();     //delta Omega with -Omega_0
        omega.coeffs() = omega.coeffs() - restDarbouxVector.coeffs();                 //delta Omega with + omega_0
        if (omega.squaredNorm() > omega_plus.squaredNorm()) omega = omega_plus;

        for (int i = 0; i < 3; i++) omega.coeffs()[i] *= bendingAndTwistingKs[i] / (wq0 + wq1 + static_cast<Real>(1.0e-6));
        omega.w() = 0.0;    //discrete Darboux vector does not have vanishing scalar part

        dq0 = q1 * omega;
        dq1 = q0 * omega;
        dq0.coeffs() *= wq0;
        dq1.coeffs() *=-wq1;

        // Now apply the corrections
		if (wq0 != 0.0)
		{
			q0.coeffs() += dq0.coeffs();
			q0.normalize();
		}
        if (wq1 != 0.0)
		{
			q1.coeffs() += dq1.coeffs();
			q1.normalize();
		}
    }
}

void Dlo::postSolve(const Real &dt){
    // Update linear velocities
    for (int i = 0; i< num_particles_; i++){
        if (inv_mass_[i] != 0){
            vel_.col(i) = (pos_[i] - prev_pos_[i])/dt;
        }
    }
    // Update angular velocities
    for (int i = 0; i< num_quaternions_; i++){
        if (!inv_iner_[i].isZero(0)){
            const Eigen::Quaternion<Real> relRot = (ori_[i] * prev_ori_[i].conjugate());
            omega_.col(i) = 2.0*relRot.vec()/dt;
        }
    }
}

/*
void Dlo::resetForces(){
    // Also Reset accumulated forces for the next iteration
    for_.setZero();
}

// Find the nearest 3D position vector col id in the given matrix
int Dlo::findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, const Eigen::Matrix<Real,3,1>& pos) {
  int nearestId = -1;
  Real minDistance = std::numeric_limits<Real>::max();
  for (int i = 0; i < matrix.cols(); ++i) {
    Eigen::Matrix<Real,3,1> currentPos = matrix.col(i);
    Real currentDistance = (currentPos - pos).norm();
    if (currentDistance < minDistance) {
      nearestId = i;
      minDistance = currentDistance;
    }
  }
  return nearestId;
}

int Dlo::attachNearest(const Eigen::Matrix<Real,3,1> &pos){
    int id = findNearestPositionVectorId(pos_,pos);
    // Make that particle stationary
    if (id >= 0){
        inv_mass_(id) = 0.0;
        attached_ids_.push_back(id); // add fixed particle id to the attached_ids_ vector
    }
    return id;
}

void Dlo::updateAttachedPose(const int &id, const Eigen::Matrix<Real,3,1> &pos){
    pos_.col(id) = pos;
}
*/

std::vector<Eigen::Matrix<Real,3,1>> *Dlo::getPosPtr(){
    return &pos_;
}

Eigen::Matrix<Real,3,Eigen::Dynamic> *Dlo::getVelPtr(){
    return &vel_;
}

Eigen::Matrix<Real,3,Eigen::Dynamic> *Dlo::getForPtr(){
    return &for_;
}

Eigen::Matrix2Xi *Dlo::getStretchShearIdsPtr(){
    return &stretchShear_ids_;
}

Eigen::Matrix2Xi *Dlo::getBendTwistIdsPtr(){
    return &bendTwist_ids_;
}

std::vector<int> *Dlo::getAttachedIdsPtr(){
    return &attached_ids_;
}