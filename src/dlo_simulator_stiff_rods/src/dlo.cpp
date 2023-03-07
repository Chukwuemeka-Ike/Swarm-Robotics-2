/*
 * Author: Burak Aksoy
 */

#include "dlo_simulator_stiff_rods/utilities/dlo.h"

using namespace pbd_object;

const Real eps_ = static_cast<Real>(1e-10);

Dlo::Dlo(){

}

Dlo::Dlo(const MeshDLO &mesh, 
        const Real &zero_stretch_stiffness, 
        const Real &young_modulus, 
        const Real &torsion_modulus, 
        const Real &density,
        const Real &radius,
        const bool &use_direct_kkt_solver):
    mesh_(mesh),
    zero_stretch_stiffness_(zero_stretch_stiffness),
    young_modulus_(young_modulus),
    torsion_modulus_(torsion_modulus),
    density_(density),
    radius_(radius),
    use_direct_kkt_solver_(use_direct_kkt_solver)
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

    // Initialize constraint values for each constraint
    RHS_.assign(num_quaternions_-1,Eigen::Matrix<Real,6,1>::Zero());

    // Initialize 2 jacobians for each constraint
    bendingAndTorsionJacobians_.resize(num_quaternions_-1);
	std::vector<Eigen::Matrix<Real,3,3>> sampleJacobians(2);
	sampleJacobians[0].setZero();
	sampleJacobians[1].setZero();
	std::fill(bendingAndTorsionJacobians_.begin(), bendingAndTorsionJacobians_.end(), sampleJacobians);

    setMasses();

    setStretchBendTwistConstraints();

    // attached_ids_  //Initially empty vector of integers to store the ids of attached (fixed) particles.
}

Dlo::~Dlo(){

}


void Dlo::setStretchBendTwistConstraints(){
    /* 
    Specifies:
    - stretchBendTwist_ids_ (from DLO mesh)
    - stretchBendTwist_restDarbouxVectors_
    - stretchBendTwist_constraintPosInfo_:
        col 0:	connector in segment 0 (local)
        col 1:	connector in segment 1 (local)
        col 2:	connector in segment 0 (global)
        col 3:	connector in segment 1 (global)    
    - average_segment_lengths_
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

    stretchBendTwist_ids_ = quaternion_ids_mat;

    for (int i = 0; i < stretchBendTwist_ids_.cols(); i++){
        int id0 = stretchBendTwist_ids_(0,i);
        int id1 = stretchBendTwist_ids_(1,i);
        
        //
        // compute rest Darboux vector based on eqn 7
        //        
        Eigen::Quaternion<Real> rest_darboux_vect = ori_[id0].conjugate() * ori_[id1]; 
        Real averageSegmentLength = 0.5*(mesh_.segment_lengths[id0] + mesh_.segment_lengths[id1]);
        stretchBendTwist_restDarbouxVectors_.push_back((2./averageSegmentLength)*rest_darboux_vect.vec());
        average_segment_lengths_.push_back(averageSegmentLength);

        //
        // set initial constraint position info 
        //
        // transform in local coordinates
        const Eigen::Matrix<Real,3,3> rot0 = ori_[id0].toRotationMatrix();
        const Eigen::Matrix<Real,3,3> rot1 = ori_[id1].toRotationMatrix();

        Eigen::Matrix<Real, 3, 4> constraintPosInfo;
        // locally each constraint position is along the z axis at the half lenght of the segment
        constraintPosInfo.col(0) = Eigen::Matrix<Real,3,1>(0,0,0.5*mesh_.segment_lengths[id0]);
        constraintPosInfo.col(1) = Eigen::Matrix<Real,3,1>(0,0,-0.5*mesh_.segment_lengths[id1]);
        constraintPosInfo.col(2) = rot0 * constraintPosInfo.col(0) + mesh_.vertices[id0];
        constraintPosInfo.col(3) = rot1 * constraintPosInfo.col(1) + mesh_.vertices[id1];

        // std::cout << "0: " << constraintPosInfo.col(2) << std::endl;
        // std::cout << "1: " << constraintPosInfo.col(3) << std::endl;
        // std::cout << "------------------------------" << std::endl;

        stretchBendTwist_constraintPosInfo_.push_back(constraintPosInfo);
    }

    initTree();
}

// -----------------------------------------------------------------
void Dlo::initTree(){
    // initLists;
    root_ = new Node;
    forward_ = new std::list<Node*>;
    backward_ = new std::list<Node*>;

    initNodes();
}


void Dlo::initNodes() {
    // select the first segment node as root
    // then starting from this node, all edges (joints ie constraints) are followed
    // and the children and parent nodes are saved
    root_->parent = NULL;
    root_->isconstraint = false;
    root_->index = 0;

    root_->D.setZero();
    root_->Dinv.setZero();
    root_->J.setZero();

    initSegmentNode(root_);
    
    orderMatrixH(root_);
}

void Dlo::initSegmentNode(Node *n){
    // four our simple single branch rod the tree is simple to create
    // because its a simple doubly linked list
    // For each constraint:
    for (int i = 0; i < stretchBendTwist_restDarbouxVectors_.size(); i++){
        // IDs 
        const int& id0 = stretchBendTwist_ids_(0,i);
        const int& id1 = stretchBendTwist_ids_(1,i);

        // Note that i is the id of the constraint
        Node * constraintNode = new Node();
        constraintNode->index = i;
        constraintNode->isconstraint = true;
        constraintNode->parent = n;
        constraintNode->parent->index = id0;

        constraintNode->D.setZero();
        constraintNode->Dinv.setZero();
        constraintNode->J.setZero();
        constraintNode->soln.setZero();

        n->children.push_back(constraintNode);

        Node * segmentNode = new Node();
        segmentNode->isconstraint = false;
        segmentNode->parent = constraintNode;
        segmentNode->index = id1;

        segmentNode->D.setZero();
        segmentNode->Dinv.setZero();
        segmentNode->J.setZero();
        segmentNode->soln.setZero();

        constraintNode->children.push_back(segmentNode);
        
        n = segmentNode;
    }
}

void Dlo::orderMatrixH(Node *n){
    // To store globally
    for (unsigned int i = 0; i < n->children.size(); i++)
		orderMatrixH(n->children[i]);
	forward_->push_back(n);
	backward_->push_front(n);
}

// -----------------------------------------------------------------

void Dlo::setMasses(){
    /* 
    Specifies:
    - inv_mass_, using the edge rest lengths, DLO length, radius and density information.
    - inv_iner_,
    - iner_
    */

    for (int i = 0; i < num_particles_; i++) {
        Real l = mesh_.segment_lengths[i]; // segment length
        Real V = M_PI*(radius_*radius_)*l;  // volume
        Real mass = V * density_; // segment mass

        if (mass > 0.0) {
            Real p_0_mass = ( inv_mass_[i] > 0.0 ) ? 1.0/inv_mass_[i] : 0.0;

            Real q_0_inertia_xx = ( inv_iner_[i](0,0) > 0.0 ) ? 1.0/inv_iner_[i](0,0) : 0.0;
            Real q_0_inertia_yy = ( inv_iner_[i](1,1) > 0.0 ) ? 1.0/inv_iner_[i](1,1) : 0.0;
            Real q_0_inertia_zz = ( inv_iner_[i](2,2) > 0.0 ) ? 1.0/inv_iner_[i](2,2) : 0.0;

            p_0_mass += mass;
            
            q_0_inertia_xx += (0.25)*mass*(radius_*radius_) + (1./12.)*mass*(l*l);
            q_0_inertia_yy += (0.25)*mass*(radius_*radius_) + (1./12.)*mass*(l*l);
            q_0_inertia_zz += 0.5*mass*(radius_*radius_);

            inv_mass_[i] = 1.0/p_0_mass;
            
            inv_iner_[i](0,0) = 1.0/q_0_inertia_xx;
            inv_iner_[i](1,1) = 1.0/q_0_inertia_yy;
            inv_iner_[i](2,2) = 1.0/q_0_inertia_zz;

            iner_[i](0,0) = q_0_inertia_xx;
            iner_[i](1,1) = q_0_inertia_yy;
            iner_[i](2,2) = q_0_inertia_zz;
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

    for (int i = 0; i < num_particles_; i++) {
        min_x = std::min(min_x, pos_[i](0));
        max_x = std::max(max_x, pos_[i](0));
        min_y = std::min(min_y, pos_[i](1));
        max_y = std::max(max_y, pos_[i](1));
    }

    Real eps = 0.0001;

    for (int i = 0; i < num_particles_; i++) {
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
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            case 2:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    inv_mass_[i] = 0.0;
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            default:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    inv_mass_[i] = 0.0;
                    inv_iner_[i].setZero();
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
        }
    }
}

void Dlo::preSolve(const Real &dt, const Eigen::Matrix<Real,3,1> &gravity){
    #pragma omp parallel default(shared)
    {
        // Semi implicit euler (position)
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
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
        // #pragma omp for schedule(static) 
        #pragma omp parallel for
        for (int i = 0; i< num_quaternions_; i++){
            // if (!inv_iner_[i].isZero(0)){
            if (!inv_mass_[i]!= 0){
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
}

void Dlo::solve(const Real &dt){
    solveStretchBendTwistConstraints(dt);
}

void Dlo::solveStretchBendTwistConstraints(const Real &dt){
    // Inits before projection (alpha tilde, eqn 24)
    Real inv_dt_sqr = static_cast<Real>(1.0)/(dt*dt);
    // compute compliance parameter of the stretch constraint part
    Eigen::Matrix<Real,3,1> stretch_compliance; // upper diagonal of eqn 24

    stretch_compliance <<
		inv_dt_sqr / (zero_stretch_stiffness_),
		inv_dt_sqr / (zero_stretch_stiffness_),
		inv_dt_sqr / (zero_stretch_stiffness_);


    // compute compliance parameter of the bending and torsion constraint part
    Eigen::Matrix<Real,3,1> bending_and_torsion_compliance; //lower diagonal of eqn 24

    Real secondMomentOfArea(static_cast<Real>(M_PI_4) * std::pow(radius_, static_cast<Real>(4.0)));
	Real bendingStiffness(young_modulus_ * secondMomentOfArea);
	Real torsionStiffness(static_cast<Real>(2.0) * torsion_modulus_ * secondMomentOfArea);

    // assumption: the rod axis follows the z-axis of the local frame
    bending_and_torsion_compliance << 
        inv_dt_sqr / bendingStiffness,
        inv_dt_sqr / bendingStiffness,
        inv_dt_sqr / torsionStiffness;

    max_error_ = 0.0;

    // updates on the constraints
    // For each constraint
    for (int i = 0; i < stretchBendTwist_restDarbouxVectors_.size(); i++){
        // IDs 
        const int& id0 = stretchBendTwist_ids_(0,i);
        const int& id1 = stretchBendTwist_ids_(1,i);

        const Real& averageSegmentLength = average_segment_lengths_[i];

        // bending_and_torsion_compliance *= static_cast<Real>(1.0) / averageSegmentLength; // why?        

        // inverse masses of these segments
        const Real& invMass0 = inv_mass_[id0];
        const Real& invMass1 = inv_mass_[id1];

        // inverse inertia matrices of these segments (TODO: these needs to be rotated to world frame)
        const Eigen::Matrix<Real,3,3>& inertiaInverseW0 = inv_iner_[id0];
        const Eigen::Matrix<Real,3,3>& inertiaInverseW1 = inv_iner_[id1];

        // Current positions
        Eigen::Matrix<Real,3,1>& p0 = pos_[id0];
        Eigen::Matrix<Real,3,1>& p1 = pos_[id1];

        // Current orientations
        Eigen::Quaternion<Real>& q0 = ori_[id0];
        Eigen::Quaternion<Real>& q1 = ori_[id1];
        const Eigen::Matrix<Real,3,3> rot0 = q0.toRotationMatrix();
        const Eigen::Matrix<Real,3,3> rot1 = q1.toRotationMatrix();

        // Current constraint pos info needs to be updated
        Eigen::Matrix<Real, 3, 4>& constraintPosInfo = stretchBendTwist_constraintPosInfo_[i];

        // update constraint (for eqn 23, upper part)
        constraintPosInfo.col(2) = rot0 * constraintPosInfo.col(0) + p0;
        constraintPosInfo.col(3) = rot1 * constraintPosInfo.col(1) + p1;

        const Eigen::Matrix<Real,3,1>& connector0 = constraintPosInfo.col(2);
        const Eigen::Matrix<Real,3,1>& connector1 = constraintPosInfo.col(3);

        // Compute zero-stretch part of constraint violation (eqn 23, upper part)
        Eigen::Matrix<Real,3,1> stretchViolation = connector0 - connector1;

        // rest darboux vector (imaginary part of it)
        Eigen::Matrix<Real,3,1>& restDarbouxVector = stretchBendTwist_restDarbouxVectors_[i]; 

        // Current darboux vector (imaginary part of it)
        Eigen::Matrix<Real,3,1> omega = (2./averageSegmentLength)*((q0.conjugate() * q1).vec());   //darboux vector

        // Compute bending and torsion part of constraint violation (eqn 23, lower part)
        Eigen::Matrix<Real,3,1> bendingAndTorsionViolation = (omega) - restDarbouxVector;

        // fill right hand side of the linear equation system (Equation (19))
        Eigen::Matrix<Real, 6, 1>& rhs= RHS_[i];
        rhs.block<3, 1>(0, 0) = -stretchViolation;
        rhs.block<3, 1>(3, 0) = -bendingAndTorsionViolation;

        // compute max error
		for (unsigned char j(0); j < 6; ++j)
		{
			max_error_ = std::max(max_error_, std::abs(rhs[j]));
		}
        
        // compute G matrices (Equation (27))
        Eigen::Matrix<Real, 4, 3> G0, G1;
        // w component at index 3
        G0 <<
            static_cast<Real>(0.5)*q0.w(), static_cast<Real>(0.5)*q0.z(), -static_cast<Real>(0.5)*q0.y(),
            -static_cast<Real>(0.5)*q0.z(), static_cast<Real>(0.5)*q0.w(), static_cast<Real>(0.5)*q0.x(),
            static_cast<Real>(0.5)*q0.y(), -static_cast<Real>(0.5)*q0.x(), static_cast<Real>(0.5)*q0.w(),
            -static_cast<Real>(0.5)*q0.x(), -static_cast<Real>(0.5)*q0.y(), -static_cast<Real>(0.5)*q0.z();
        // w component at index 3
        G1 <<
            static_cast<Real>(0.5)*q1.w(), static_cast<Real>(0.5)*q1.z(), -static_cast<Real>(0.5)*q1.y(),
            -static_cast<Real>(0.5)*q1.z(), static_cast<Real>(0.5)*q1.w(), static_cast<Real>(0.5)*q1.x(),
            static_cast<Real>(0.5)*q1.y(), -static_cast<Real>(0.5)*q1.x(), static_cast<Real>(0.5)*q1.w(),
            -static_cast<Real>(0.5)*q1.x(), -static_cast<Real>(0.5)*q1.y(), -static_cast<Real>(0.5)*q1.z();

        // compute bending and torsion Jacobians (Equation (10) and Equation (11))
	    Eigen::Matrix<Real, 3, 4> jOmega0, jOmega1;
        // w component at index 3, Equation (11)
        jOmega0 <<
            -q1.w(), -q1.z(), q1.y(), q1.x(),
            q1.z(), -q1.w(), -q1.x(), q1.y(),
            -q1.y(), q1.x(), -q1.w(), q1.z();
        // w component at index 3, Equation (10)
        jOmega1 <<
            q0.w(), q0.z(), -q0.y(), -q0.x(),
            -q0.z(), q0.w(), q0.x(), -q0.y(),
            q0.y(), -q0.x(), q0.w(), -q0.z();
        jOmega0 *= static_cast<Real>(2.0) / averageSegmentLength;
        jOmega1 *= static_cast<Real>(2.0) / averageSegmentLength;

        // Lower right part of eqn 25
        Eigen::Matrix<Real, 3, 3> & jOmegaG0 = bendingAndTorsionJacobians_[i][0];
        jOmegaG0 = jOmega0*G0;
        

        // Lower right part of eqn 26
        Eigen::Matrix<Real, 3, 3> & jOmegaG1 = bendingAndTorsionJacobians_[i][1];
        jOmegaG1 = jOmega1*G1;

        if (!use_direct_kkt_solver_){
            // Start actual solving from here -------
            // compute matrix of the linear equation system (using Equations (25), (26), and (28) in Equation (19))
            Eigen::Matrix<Real, 6, 6> JMJT = Eigen::Matrix<Real, 6, 6>::Zero(); // Initialize place holder for J*M^-1*J^T

            // compute stretch block
            Eigen::Matrix<Real,3,3> K1, K2;
            computeMatrixK(connector0, invMass0, p0, inertiaInverseW0, K1);
            computeMatrixK(connector1, invMass1, p1, inertiaInverseW1, K2);
            JMJT.block<3, 3>(0, 0) = K1 + K2;


            // compute coupling blocks
            const Eigen::Matrix<Real,3,1> ra = connector0 - p0;
            const Eigen::Matrix<Real,3,1> rb = connector1 - p1;

            Eigen::Matrix<Real,3,3> ra_crossT, rb_crossT;
            crossProductMatrix(-ra, ra_crossT); // use -ra to get the transpose
            crossProductMatrix(-rb, rb_crossT); // use -rb to get the transpose


            Eigen::Matrix<Real,3,3> offdiag(Eigen::Matrix<Real,3,3>::Zero());
            if (invMass0 != 0.0)
            {
                offdiag = jOmegaG0 * inertiaInverseW0 * ra_crossT * (-1);
            }

            if (invMass1 != 0.0)
            {
                offdiag += jOmegaG1 * inertiaInverseW1 * rb_crossT;
            }
            JMJT.block<3, 3>(3, 0) = offdiag;
            JMJT.block<3, 3>(0, 3) = offdiag.transpose();

            // compute bending and torsion block
            Eigen::Matrix<Real,3,3> MInvJT0 = inertiaInverseW0 * jOmegaG0.transpose();
            Eigen::Matrix<Real,3,3> MInvJT1 = inertiaInverseW1 * jOmegaG1.transpose();

            Eigen::Matrix<Real,3,3> JMJTOmega(Eigen::Matrix<Real,3,3>::Zero());
            if (invMass0 != 0.0)
            {
                JMJTOmega = jOmegaG0*MInvJT0;
            }

            if (invMass1 != 0.0)
            {
                JMJTOmega += jOmegaG1*MInvJT1;
            }
            JMJT.block<3, 3>(3, 3) = JMJTOmega;

            // add compliance
            JMJT(0, 0) += stretch_compliance(0);
            JMJT(1, 1) += stretch_compliance(1);
            JMJT(2, 2) += stretch_compliance(2);
            JMJT(3, 3) += bending_and_torsion_compliance(0);
            JMJT(4, 4) += bending_and_torsion_compliance(1);
            JMJT(5, 5) += bending_and_torsion_compliance(2);

            // solve linear equation system (Equation 19)
            Eigen::Matrix<Real,6,1> deltaLambda = JMJT.ldlt().solve(rhs);

            // compute position and orientation updates (using Equations (25), (26), and (28) in Equation (20))
            Eigen::Matrix<Real,3,1> deltaLambdaStretch = deltaLambda.block<3, 1>(0, 0);
            Eigen::Matrix<Real,3,1> deltaLambdaBendingAndTorsion = deltaLambda.block<3, 1>(3, 0);

            // Correction vectors place holders ----------------------------
            Eigen::Matrix<Real,3,1> dp0;
            Eigen::Matrix<Real,3,1> dp1;
            Eigen::Quaternion<Real> dq0;
            Eigen::Quaternion<Real> dq1;

            dp0.setZero();
            dp1.setZero();
            dq0.coeffs().setZero();
            dq1.coeffs().setZero();

            if (invMass0 != 0.)
            {
                dp0 += invMass0 * deltaLambdaStretch;
                dq0.coeffs() += G0 * (inertiaInverseW0 * ra_crossT * (-1 * deltaLambdaStretch) + MInvJT0 * deltaLambdaBendingAndTorsion);
            }

            if (invMass1 != 0.)
            {
                dp1 -= invMass1 * deltaLambdaStretch;
                dq1.coeffs() += G1 * (inertiaInverseW1 * rb_crossT * deltaLambdaStretch + MInvJT1 * deltaLambdaBendingAndTorsion);
            }

            // Now apply the corrections -----------------------------
            if (invMass0 != 0.0)
            {
                p0 += dp0;
                q0.coeffs() += dq0.coeffs();
                q0.normalize();
            }
            if (invMass1 != 0.0)
            {
                p1 += dp1;
                q1.coeffs() += dq1.coeffs();
                q1.normalize();
            }
        }
    }

    if (use_direct_kkt_solver_){
        // Factor procedure
        factor(stretch_compliance, bending_and_torsion_compliance);
        solver();
    }
}

void Dlo::solver(){
    std::list<Node*>::iterator nodeIter;
	for (nodeIter = forward_->begin(); nodeIter != forward_->end(); nodeIter++)
	{
		Node *node = *nodeIter;
		if (node->isconstraint)
		{
			node->soln = -RHS_[node->index];
		}
		else
		{
			node->soln.setZero();
		}
		std::vector <Node*> &children = node->children;
		for (size_t i = 0; i < children.size(); ++i)
		{
			Eigen::Matrix<Real,6,6> cJT = children[i]->J.transpose();
			Eigen::Matrix<Real,6,1> &csoln = children[i]->soln;
			Eigen::Matrix<Real,6,1> v = cJT * csoln;
			node->soln = node->soln - v;
		}
	}

    for (nodeIter = backward_->begin(); nodeIter != backward_->end(); nodeIter++)
	{
		Node *node = *nodeIter;

		bool noZeroDinv(true);
		if (!node->isconstraint)
		{
            const int & ind = node->index;
            Real &inv_m = inv_mass_[ind];
			noZeroDinv = (inv_m != 0.0); // segment->isDynamic();
		}
		if (noZeroDinv) // if DInv == 0 child value is 0 and node->soln is not altered
		{
			node->soln = node->DLDLT.solve(node->soln);

			if (node->parent != NULL)
			{
				node->soln -= node->J * node->parent->soln;
			}
		}
		else
		{
			node->soln.setZero(); // segment of node is not dynamic
		}
	}

    // compute position and orientation updates
	for (nodeIter = forward_->begin(); nodeIter != forward_->end(); nodeIter++)
	{
		Node *node = *nodeIter;
		if (!node->isconstraint)
		{
			const int & ind = node->index;
            Real &inv_m = inv_mass_[ind];

			if (inv_m == 0.0)
			{
				break;
			}

			const Eigen::Matrix<Real,6,1> & soln(node->soln);
			Eigen::Matrix<Real,3,1> deltaXSoln = Eigen::Matrix<Real,3,1>(-soln[0], -soln[1], -soln[2]);
			// corr_x[ind] = deltaXSoln;

            // Apply the position solution
            pos_[ind] += deltaXSoln;

			Eigen::Matrix<Real, 4, 3> G;
            Eigen::Quaternion<Real>& q0 = ori_[ind];
            G <<
            static_cast<Real>(0.5)*q0.w(), static_cast<Real>(0.5)*q0.z(), -static_cast<Real>(0.5)*q0.y(),
            -static_cast<Real>(0.5)*q0.z(), static_cast<Real>(0.5)*q0.w(), static_cast<Real>(0.5)*q0.x(),
            static_cast<Real>(0.5)*q0.y(), -static_cast<Real>(0.5)*q0.x(), static_cast<Real>(0.5)*q0.w(),
            -static_cast<Real>(0.5)*q0.x(), -static_cast<Real>(0.5)*q0.y(), -static_cast<Real>(0.5)*q0.z();

			Eigen::Quaternion<Real> deltaQSoln;
			deltaQSoln.coeffs() = G * Eigen::Matrix<Real,3,1>(-soln[3], -soln[4], -soln[5]);
			// corr_q[ind] = deltaQSoln;

            // Apply the orientation solution
            ori_[ind].coeffs() += deltaQSoln.coeffs();
            ori_[ind].normalize();
		}
	}
}

void Dlo::factor(const Eigen::Matrix<Real,3,1> &stretch_compliance,
                const Eigen::Matrix<Real,3,1> &bending_and_torsion_compliance){
    std::list<Node*>::iterator nodeIter;
	for (nodeIter = forward_->begin(); nodeIter != forward_->end(); nodeIter++)
	{
		Node *node = *nodeIter;
		// compute system matrix diagonal
		if (node->isconstraint){
			//insert compliance
			node->D.setZero();

			node->D(0, 0) -= stretch_compliance[0];
			node->D(1, 1) -= stretch_compliance[1];
			node->D(2, 2) -= stretch_compliance[2];

			node->D(3, 3) -= bending_and_torsion_compliance[0];
			node->D(4, 4) -= bending_and_torsion_compliance[1];
			node->D(5, 5) -= bending_and_torsion_compliance[2];
		}
		else{
			// getMassMatrix to node->D
            Eigen::Matrix<Real, 6, 6> & M = node->D;

            const int & ind = node->index;
            Real &inv_m = inv_mass_[ind];

            if (inv_m == 0.0) {
                M = Eigen::Matrix<Real, 6, 6>::Identity();
            } else {
                Real mass = 1.0/inv_m;
                
                const Eigen::Matrix<Real,3,3> & inertiaLocal = iner_[ind];  // local inertia
                Eigen::Matrix<Real,3,3> rotationMatrix = ori_[ind].toRotationMatrix();
                Eigen::Matrix<Real,3,3> inertia = rotationMatrix * inertiaLocal * rotationMatrix.transpose();
                
                // Upper half
                for (int i = 0; i < 3; i++)
                for (int j = 0; j < 6; j++)
                if (i == j)
                    M(i, j) = mass;
                else
                    M(i, j) = 0.0;

                // lower left
                for (int i = 3; i < 6; i++)
                for (int j = 0; j < 3; j++)
                    M(i, j) = 0.0;

                // lower right
                for (int i = 3; i < 6; i++)
                for (int j = 3; j < 6; j++)
                    M(i, j) = inertia(i - 3, j - 3);
            }

		}

		// compute Jacobian
		if (node->parent != NULL)
		{
			if (node->isconstraint)
			{
				//compute J 
                const int & constraint_ind = node->index;
                const int & segment_ind =  node->parent->index;

				Real sign = 1;
				int segmentIndex = 0;
				if (segment_ind == stretchBendTwist_ids_(1,constraint_ind))
				{
					segmentIndex = 1;
					sign = -1;
				}

				const Eigen::Matrix<Real, 3, 4> &constraintInfo = stretchBendTwist_constraintPosInfo_[constraint_ind];
				const Eigen::Matrix<Real,3,1> r = constraintInfo.col(2 + segmentIndex) - pos_[segment_ind];

				Eigen::Matrix<Real,3,3> r_cross;
				Real crossSign(-static_cast<Real>(1.0)*sign);
				crossProductMatrix(crossSign*r, r_cross);

				Eigen::DiagonalMatrix<Real, 3> upperLeft(sign, sign, sign);
				node->J.block<3, 3>(0, 0) = upperLeft;

				Eigen::Matrix<Real,3,3> lowerLeft(Eigen::Matrix<Real,3,3>::Zero());
				node->J.block<3, 3>(3, 0) = lowerLeft;

				node->J.block<3, 3>(0, 3) = r_cross;

				Eigen::Matrix<Real,3,3> &lowerRight(bendingAndTorsionJacobians_[constraint_ind][segmentIndex]);
				node->J.block<3, 3>(3, 3) = lowerRight;
			}
			else
			{
				//compute JT
				const int & constraint_ind = node->parent->index;
                const int & segment_ind = node->index;

				Real sign = 1;
				int segmentIndex = 0;
				if (segment_ind == stretchBendTwist_ids_(1,constraint_ind))
				{
					segmentIndex = 1;
					sign = -1;
				}

				const Eigen::Matrix<Real, 3, 4> &constraintInfo = stretchBendTwist_constraintPosInfo_[constraint_ind];
				const Eigen::Matrix<Real,3,1> r = constraintInfo.col(2 + segmentIndex) - pos_[segment_ind];
				
                Eigen::Matrix<Real,3,3> r_crossT;
				crossProductMatrix(sign*r, r_crossT);

				Eigen::DiagonalMatrix<Real, 3> upperLeft(sign, sign, sign);
				node->J.block<3, 3>(0, 0) = upperLeft;

				node->J.block<3, 3>(3, 0) = r_crossT;

				Eigen::Matrix<Real,3,3> upperRight(Eigen::Matrix<Real,3,3>::Zero());
				node->J.block<3, 3>(0, 3) = upperRight;

				Eigen::Matrix<Real,3,3> lowerRight(bendingAndTorsionJacobians_[constraint_ind][segmentIndex].transpose());
				node->J.block<3, 3>(3, 3) = lowerRight;
			}
		}
	}

    for (nodeIter = forward_->begin(); nodeIter != forward_->end(); nodeIter++)
	{
		Node *node = *nodeIter;
		std::vector <Node*> children = node->children;
		for (size_t i = 0; i < children.size(); i++)
		{
			Eigen::Matrix<Real,6,6> JT = (children[i]->J).transpose();
			Eigen::Matrix<Real,6,6> &D = children[i]->D;
			Eigen::Matrix<Real,6,6> &J = children[i]->J;
			Eigen::Matrix<Real,6,6> JTDJ = ((JT * D) * J);
			node->D = node->D - JTDJ;
		}
		bool chk = false;
		if (!node->isconstraint)
		{
            const int & ind = node->index;
            Real &inv_m = inv_mass_[ind];

            if (inv_m == 0.0)
			{
				node->Dinv.setZero();
				chk = true;
			}
		}

		node->DLDLT.compute(node->D); // result reused in solve()
		if (node->parent != NULL)
		{
			if (!chk)
			{
				node->J = node->DLDLT.solve(node->J);
			}
			else
			{
				node->J.setZero();
			}
		}
	}
}

void Dlo::computeMatrixK(const Eigen::Matrix<Real,3,1> &connector, 
                        const Real invMass, 
                        const Eigen::Matrix<Real,3,1> &x, 
                        const Eigen::Matrix<Real,3,3> &inertiaInverseW, 
                        Eigen::Matrix<Real,3,3> &K) {
	if (invMass != 0.0)
	{
        // vector from center of mass to conneting point in world frame
		const Eigen::Matrix<Real,3,1> v = connector - x;
		const Real a = v[0];
		const Real b = v[1];
		const Real c = v[2];

		// J is symmetric
		const Real j11 = inertiaInverseW(0, 0);
		const Real j12 = inertiaInverseW(0, 1);
		const Real j13 = inertiaInverseW(0, 2);
		const Real j22 = inertiaInverseW(1, 1);
		const Real j23 = inertiaInverseW(1, 2);
		const Real j33 = inertiaInverseW(2, 2);

		K(0, 0) = c*c*j22 - b*c*(j23 + j23) + b*b*j33 + invMass;
		K(0, 1) = -(c*c*j12) + a*c*j23 + b*c*j13 - a*b*j33;
		K(0, 2) = b*c*j12 - a*c*j22 - b*b*j13 + a*b*j23;
		K(1, 0) = K(0, 1);
		K(1, 1) = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + invMass;
		K(1, 2) = -(b*c*j11) + a*c*j12 + a*b*j13 - a*a*j23;
		K(2, 0) = K(0, 2);
		K(2, 1) = K(1, 2);
		K(2, 2) = b*b*j11 - a*b*(j12 + j12) + a*a*j22 + invMass;
	}
	else
		K.setZero();
}

void Dlo::crossProductMatrix(const Eigen::Matrix<Real,3,1> &v, Eigen::Matrix<Real,3,3> &v_hat){
	v_hat << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
}

void Dlo::postSolve(const Real &dt){
    #pragma omp parallel default(shared)
    {
        // Update linear velocities
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (inv_mass_[i] != 0){
                vel_.col(i) = (pos_[i] - prev_pos_[i])/dt;
            }
        }
        // Update angular velocities
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_quaternions_; i++){
            // if (!inv_iner_[i].isZero(0)){
            if (!inv_mass_[i]!= 0){
                const Eigen::Quaternion<Real> relRot = (ori_[i] * prev_ori_[i].conjugate());
                omega_.col(i) = 2.0*relRot.vec()/dt;
            }
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

Eigen::Matrix2Xi *Dlo::getStretchBendTwistIdsPtr(){
    return &stretchBendTwist_ids_;
}

std::vector<Eigen::Quaternion<Real>> *Dlo::getOriPtr(){
    return &ori_;
}

std::vector<Real> *Dlo::getSegmentLengthsPtr(){
    return &mesh_.segment_lengths;
}

std::vector<int> *Dlo::getAttachedIdsPtr(){
    return &attached_ids_;
}