#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#include "GL/glew.h"
#include "GL/glut.h"



#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 300.0;   
//double BehaviorController::gOriKv = 20.0;    
//double BehaviorController::gOriKp = 25.0; 
double BehaviorController::gOriKv = 32.0;
double BehaviorController::gOriKp = 16.0 * 16.0; 
double BehaviorController::gVelKv = 5.0;    
double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 100.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;
double BehaviorController::mDeltaT = 0.1;
unsigned int BehaviorController::row_num = 1;

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;

	reset();
}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();

}

vec3 BehaviorController::getForce(RefFrameType type)
{
	vec3 force;
	if (type == WORLD)
		force = m_Guide.getGlobalRotation()*m_force;
	else force = m_force;

	return force;
}

vec3 BehaviorController::getTorque(RefFrameType type)
{
	vec3 torque;
	if (type == WORLD)
		torque = m_Guide.getGlobalRotation()*m_torque;
	else torque = m_torque;

	return torque;
}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
	m_BehaviorList[PURSUIT] = new Pursuit(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[EVASION] = new Evasion(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FORMATION] = new Formation(m_pBehaviorTarget, m_AgentList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.

			// TODO: Insert your code here
	}
	
}

void BehaviorController::control(double deltaT)

{
	this->mDeltaT = deltaT;
	if (mpActiveBehavior)
	{ 
		// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
		// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands
		// Desired speed is in the world space.
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_Vdesired[1] = 0;
		//std::cout << "mpActiveBehavior:" << mpActiveBehavior->GetName() << std::endl;
#define CONTROL_2D

#ifdef CONTROL_2D
		// IMPLEMENTATION1: Force and torque controllers for 2D planar case - DO THIS FIRST
		//  force and torque inputs are computed from the desired speed (Sd) 
		//  and desired direction (thetad) as follows:
		//
		//              Velocity P controller : fz = mass * Kv * (Sd - Vz)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot -Kp * (thetad - theta))
		//  where 
		//    Vz is the z-component of the velocity vector 
		//    theta is the actual heading angle value (i.e. y-axis component of orientation state vector)
		//    fz is the z component of the body axes force vector (fx = fy = 0.0)
		// and the values of the gains Kv and Kp are set to achieve the desired transient response for the
		// speed and direction (Note: Kv and Kp are different for each controller)

		// IMPLEMENTATION1 - 2D Force and Torque controllers
		// Begin:  2D Force and Torque controllers
		//
		// TODO: insert your code here to compute m_force and m_torque
		// Compute the desired speed (scalar) and desired direction (scalar).
		
		//m_Vdesired = vec3(0, 0, -1000);
		// Damping Ratio: 2, Natural frequency: 5.
		double Sd = m_Vdesired.Length();
		m_vd = Sd;
		

		vec3 tempV = m_Vdesired;
		vec3 dirD = tempV.Normalize();
		double thetad;

		if (Sd < 2)
		{
			thetad = m_lastThetad;
			m_thetad = m_lastThetad;
		}
		else
		{
			thetad = atan2(dirD[0], dirD[2]);
			m_lastThetad = m_thetad;
			m_thetad = thetad;
		}
		
		
		
		// std::cout << "thetad:" << thetad * Rad2Deg << std::endl;
		// std::cout << "this->m_Euler[1]:" << this->m_Euler[1] * Rad2Deg << std::endl;
		// std::cout << "thetad - this->m_Euler[1]:" << (thetad - this->m_Euler[1]) * Rad2Deg << std::endl;

		m_force[2] = this->gMass * this->gVelKv * (Sd - this->m_VelB[2]);
		// std::cout << "m_force[2]:" << m_force[2] << std::endl;
		//m_force[2] = this->gMass * 30 * (Sd - this->m_VelB[2]);
		//m_torque[1] = this->gInertia * (-this->gOriKv * this->m_AVelB[1] - this->gOriKp * (thetad - this->m_Euler[1]));
		double thetaDelta = thetad - this->m_Euler[1];
		// std::cout << "thetaDelta:" << thetaDelta * Rad2Deg << std::endl;
		ClampAngle(thetaDelta);
		// std::cout << "ClampAngle(thetaDelta):" << thetaDelta * Rad2Deg << std::endl;
		m_torque[1] = this->gInertia * (this->gOriKp * (thetaDelta) - this->gOriKv * this->m_AVelB[1]);
		//m_torque[1] = this->gInertia * (120 * (thetad - this->m_Euler[1]) - 30 * this->m_AVelB[1]);
		// std::cout << "Force:" << m_force << std::endl;
		// std::cout << "Torque:" << m_torque << std::endl;
		// std::cout << "this->m_AVelB[1]:" << this->m_AVelB[1] << std::endl;


		// End:  2D Force and Torque controllers
#else
		//IMPLEMENTATION2: Force and torque controllers for 3D case (this is required for the Unity plugin) 
		// DO THIS AFTER confirming that the force and torque controllers above are working for the 2D case
		//
		// Here are the steps to implementing the 3D torque controller
		// 1. get current rotation matrix of the Guide (R) - which represents the agent's orientation in the world
		// 2. construct the desired agent rotation matrix.  That is, Rd.  
		//      To do this, assume the Z-axis of the agent is facing in the 
		//      direction of the desired velocity vector, the Y-axis points UP and the X-axis points left
		// 3. Compute the change in rotation matrix (deltaR) required to rotate the agent to the desired
		//      orientation, where Rd = R * deltaAR
		// 4. Get the axis and angle of the quaternion corresponding to deltaR
		//
		// 5. compute the 3D force and torque vectors as follows:
		//
		//       force = w x Vb  + Kv * (Vdb - Vb)
		//       torque = w x Iw +  I * (Kp * axis * angle - Kv * w) 
		//
		//     where 
		//           Vb is the body axes velocity vector
		//           Vdb is the desired velocity vector in body axes (should only have a nonzero z-component)
		//           w is the body axes angular velocity, 
		//           I is the agent moment of inertia matrix (can assume it is the Identity matrix for now) 
		//           Kp and Kv are the same gains used in the 2D force and torque controllers above
		//
		// See the Behavioral Animation Lecture Notes on Canvas (slide 31) for more details.

		// IMPLEMENTATION2 - 3D Force and Torque controllers
		// Begin:  3D Force and Torque controllers
		//
			// TODO: insert your code here to compute m_force and m_torque






		// End:  3D Force and Torque controllers

#endif
		// when desired agent speed and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force = vec3(0.0);
			m_torque = vec3(0.0);
		}
	}
	else
	{
		m_force = vec3(0.0);
		m_torque = vec3(0.0);
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);

	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}


void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the current control inputs and state vectors
//  This function sets derive vector to appropriate values after being called
{
	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];

	// Compute the stateDot vector given the values of the current state vector and control input vector

	// m_state[0] = m_Pos0 = world position: [x 0 z]T for the 2D planar case
	// m_state[1] = m_Euler = euler angles:  [ 0 theta_y 0]T for the 2D planar case 
	// m_state[2] = m_VelB = body velocity:  [ Vx 0 Vz]T for the 2D planar case
	// m_state[3] = m_AVelB =  body angular velocity [ 0 thetaDot 0]T for the 2D planar case 

#define DYNAMICS_2D

#ifdef DYNAMICS_2D
	// IMPLEMENTATION1: 2D planar case - DO THIS FIRST  
	// assume the  following:
	//    V_y = 0 in both body and world coordinates
	//    Vdot_x = V_x = 0 in body coordinates for no slip condition
	//    only the theta_y Euler angle is nonzero.  theta_x = theta_z = 0.0
	//    only the force_z component is nonzero in body coordinates 
	//    only the torque_y component is nonzero in body coordinates
	//
	//   Equations of motion
	//     Translation: Vdot = (1/m) * f
	//     Rotation:    wdot = (1/Iyy) * torque
	//  
	// Begin:  2D Planar Dynamics Implementation
	//
		// TODO: insert your code here 
	// Force and Torque is in the Agent Coordinate System.
	// Assume everything in the m_state are in world space.
	//vec3 p0Dot = m_state[2];
	//m_Vel0 = p0Dot;
	//vec3 thetaDot = state[3];
	//vec3 Vdot = getGuide().getLocal2Global() * (1 / this->gMass) * force;
	//vec3 wdot = (1 / this->gInertia) * torque;
	//stateDot[0] = p0Dot;
	//stateDot[1] = thetaDot;
	//stateDot[2] = Vdot;
	//stateDot[3] = wdot;


	mat3 R_b_0;
	R_b_0.FromAxisAngle(vec3(0, 1, 0), m_state[1][1]); // Assume the m_Euler is in Radian.
	vec3 p0Dot = R_b_0 * m_state[2];
	if (p0Dot.Length() < 2)
	{
		p0Dot = vec3(0, 0, 0);
	}
	// m_Vel0 = p0Dot; // Last frame's world velocity is computed from the last frame's body coord velocity and last frame's Euler angle.
	vec3 thetaDot = state[3];
	// std::cout << "force:" << force << std::endl;
	vec3 Vdot = (1 / this->gMass) * force;
	vec3 wdot = (1 / this->gInertia) * torque;
	stateDot[0] = p0Dot;
	stateDot[1] = thetaDot;
	stateDot[2] = Vdot;
	stateDot[3] = wdot;
	// m_Guide.get
	// std::cout << "Vdot:" << Vdot << std::endl;

	// End:  2D Planar Dynamics Implementation

#else
	// IMPLEMENTATION2: 3D case -
	//   Equations of motion in Body Axes

	//     Translation: Vdot = (1/m) * f  +  w x V
	//     Rotation:    wdot = Iinv( torque - wb x Iw)
	// 
	//             where f = force in body coordinates
	//                   torque = torque in body coordinates
	//                   w = angular velocity in body coordinates
	//                   V = velocity in body coordinates
	//
	//  To convert forces, torques and velocities from body to world coordinates use rotation matrix
	//                   I = momemnt of inertia (can assume equals Identity matrix in this assignment

	//  to compute sEuler angle rates for statedot1, 
	//   know that w = Lmat * EulerRates where Lmat can be computed based on a Y->X->Z order of rotation

	// Begin:  3D  Dynamics Implementation
	//
		// TODO: insert your code here 






	// End:  3D Dynamics Implementation




#endif
}

void BehaviorController::updateState(float deltaT, int integratorType)
{
	//  Update the state vector (m_state) given the statedot vector (m_stateDot) using 
	//  Euler (integratorType = 0) or RK2 (integratorType = 1) integration
	//  this should be similar to what you implemented in the particle system assignment
	
	// m_state[0] = m_Pos0 = world position: [x 0 z]T for the 2D planar case
	// m_state[1] = m_Euler = euler angles:  [ 0 theta_y 0]T for the 2D planar case 
	// m_state[2] = m_VelB = body velocity:  [ Vx 0 Vz]T for the 2D planar case
	// m_state[3] = m_AVelB =  body angular velocity [ 0 thetaDot 0]T for the 2D planar case 
	// m_stateDot[0] = world velocity:  [x 0 z]T for the 2D planar case
	// m_stateDot[1] = world angular velocity:  [ 0 theta_y 0]T for the 2D planar case 
	// m_stateDot[2] = body velocity acceleration:  [ Vx 0 Vz]T for the 2D planar case
	// m_stateDot[3] = body angular velocity acceleration:  [ 0 thetaDot 0]T for the 2D planar case 


	// TODO: add your code here
	switch (integratorType)
	{
		case 0:
		{
			// Add your code here
			// std::cout << "Last frame's world velocity:" << m_stateDot[0] << std::endl;
			// std::cout << "Last frame's world angular velocity:" << m_stateDot[1] << std::endl;
			// std::cout << "Last frame's body velocity acceleration:" << m_stateDot[2] << std::endl;
			// std::cout << "Last frame's body angular velocity acceleration:" << m_stateDot[3] << std::endl;
			//if (m_stateDot[0].Length() < DBL_EPSILON)
			//{
			//	m_state[0] = vec3(0, 0, 0);
			//}
			//else
			//{
			m_state[0] = m_Pos0 = m_state[0] + m_stateDot[0] * deltaT;
			//}
			m_state[1] = m_Euler = m_state[1] + m_stateDot[1] * deltaT;
			m_state[2] = m_VelB = m_state[2] + m_stateDot[2] * deltaT;
			m_state[3] = m_AVelB = m_state[3] + m_stateDot[3] * deltaT;
			// std::cout << "This frame's world pos:" << m_state[0] << std::endl;
			// std::cout << "This frame's euler:" << m_state[1] << std::endl;
			// std::cout << "This frame's body velocity:" << m_state[2] << std::endl;
			// std::cout << "This frame's body angular velocity:" << m_state[3] << std::endl;
			break;
		}
		case 1:
		{
			// Add your code here
			/*
			vec3 lastFramePos = vec3(m_state.at(0), m_state.at(1), m_state.at(2));
			vec3 lastFrameVelocity = vec3(m_state.at(3), m_state.at(4), m_state.at(5));
			vec3 lastFramePosDot = vec3(m_stateDot.at(0), m_stateDot.at(1), m_stateDot.at(2));
			vec3 lastFrameVelocityDot = vec3(m_stateDot.at(3), m_stateDot.at(4), m_stateDot.at(5));
			vec3 currFramePPosDot = lastFrameVelocity + lastFrameVelocityDot * deltaT;

			vec3 currFramePos = lastFramePos + deltaT * (lastFramePosDot + currFramePPosDot) / 2;
			vec3 currFrameVelocity = currFramePPosDot;

			// Update Current State
			m_state.at(0) = currFramePos[0];
			m_state.at(1) = currFramePos[1];
			m_state.at(2) = currFramePos[2];
			m_state.at(3) = currFrameVelocity[0];
			m_state.at(4) = currFrameVelocity[1];
			m_state.at(5) = currFrameVelocity[2];
			*/
			break;
		}
	}

	if (abs(m_vd) < DBL_EPSILON && m_state[VEL].Length() < 2)
	{
		m_state[VEL] = vec3(0, 0, 0);
	}

	ClampAngle(m_state[ORI][1]);


	//  given the new values in m_state, the following variables are updated as follows: 

	//m_Pos0 += vec3(1, 0, 1);
	//m_Euler = vec3(0, 90 * Deg2Rad, 0);
	m_Pos0 = m_state[POS];     // agent world position,  m_Pos0 = [x 0 z]T for the 2D planar case
	m_Euler = m_state[ORI];    // agent body Euler angles, m_Euler = [0 theta 0]T for the 2D planar case
	m_VelB = m_state[VEL];     // agent body axes velocity, m_VelB = [Vx 0 Vz]T for the 2D planar case
	m_AVelB = m_state[AVEL];   // agent body axes angular velocity,  m_AVelB = [0 thetaDot 0]T for the 2D planar case
	// std::cout << "m_Pos0:" << m_Pos0 << std::endl;
	// std::cout << "m_VelB:" << m_VelB << std::endl;
 	
	//  Perform validation check to make sure all values are within MAX values
	// TODO: add your code here
	if (m_VelB.Length() > gMaxSpeed)
	{
		std::cout << "WARING: m_VelB.Length > gMaxSpeed" << std::endl;
		m_VelB = (m_VelB / m_VelB.Length()) * gMaxSpeed;
		m_state[VEL] = m_VelB;
	}

	if (m_AVelB.Length() > gMaxAngularSpeed)
	{
		std::cout << "WARING: m_AVelB.Length() > gMaxAngularSpeed" << std::endl;
		m_AVelB = (m_AVelB / m_AVelB.Length()) * gMaxAngularSpeed;
		m_state[AVEL] = m_AVelB;
	}

	if (m_force.Length() > gMaxForce)
	{
		std::cout << "WARING: m_force.Length() > gMaxForce" << std::endl;
		m_force = (m_force / m_force.Length()) * gMaxForce;
	}

	if (m_torque.Length() > gMaxTorque)
	{
		std::cout << "WARING: m_torque.Length() > gMaxTorque" << std::endl;
		m_torque = (m_torque / m_torque.Length()) * gMaxTorque;
	}

	// update Vel0:
	// m_Vel0 = this->getGuide().getLocal2Global() * m_VelB;
	mat3 currFrameRotationMat;
	currFrameRotationMat.FromAxisAngle(vec3(0, 1, 0), m_Euler[1]);
	m_Vel0 = currFrameRotationMat * m_VelB;
	//std::cout << "m_Vel0:" << m_Vel0 << std::endl;

	// update the Agent Guide rotation matrix (R)
	vec3 direction, Xaxis, Yaxis, Zaxis;
	if (m_Vel0.Length() < 1.0)
	{
		direction = m_lastVel0;
		direction.Normalize();
		// update state vector Euler angle values (y component for 2D case)
		m_state[ORI] = atan2(direction[_X], direction[_Z]);
	}
	else
	{
		direction = m_Vel0;
		m_lastVel0 = m_Vel0;
	}
	// compute Z component of rotation matrix from direction of nonzero velocity vector
	direction.Normalize();
	Zaxis = direction;



	// compute X and Y components of rotation matrix given up vector and direction vectors
	vec3 up(0.0, 1.0, 0.0);

	//TODO: insert your code here
	mat3 nightyDegreeMat;
	nightyDegreeMat.FromAxisAngle(vec3(0, 1, 0), 90 * Deg2Rad);
	Xaxis = nightyDegreeMat * direction;
	Yaxis = up;

	mat3 R(Xaxis, Yaxis, Zaxis);
	R = R.Transpose();  //mat3 constructor above sets rows of R matrix.  Transpose to put in column format

	// Here is the code directly control the position and orientation of the agent.
	m_Guide.setLocalRotation(R);
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);
	m_Guide.setGlobalRotation(R);
	m_Guide.setGlobalTranslation(m_Guide.getLocalTranslation() + m_Vel0 * deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}
}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
#ifdef DEBUG_BEHAVIORDISPLAY	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale * getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale * getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	//glRotatef(90 - angle[1], 0, 1, 0);
	glRotatef(angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehaviorType();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(this);
#endif
}

