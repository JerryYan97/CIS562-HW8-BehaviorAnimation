#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"
#include <stdlib.h>
#include <time.h>

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	//vec3 targetPosLocal = m_pTarget->getLocalTranslation();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	//std::cout << "targetPosLocal:" << targetPosLocal << std::endl;
	//std::cout << "targetPosLocal:" << targetPosGlobal << std::endl;
	//vec3 targetPos = targetPosGlobal;

	// TODO: add your code here to compute Vdesired
    // vec3 localActorPos = 
	//vec3 actorVel = actor->getVelocity();
	//vec3 actorVelDir = actorVel.Normalize();
	
	vec3 error = targetPos - actorPos;
	/*vec3 globalOrientation = actor->getOrientation();
	float theta = globalOrientation[1];

	//std::cout << "globalOrientation:" << globalOrientation[0] << " " << globalOrientation[1] << " " << globalOrientation[2] << std::endl;
	mat3 thetaMat, thetaDMat, nightyDegMat;
	thetaMat.FromAxisAngle(vec3(0, 1, 0), globalOrientation[1]);
	nightyDegMat.FromAxisAngle(vec3(0, 1, 0), 90 * Deg2Rad);


	//vec3 errorX = Dot(error, actorVelDir) * actorVelDir;
	//vec3 errorY = error - errorX;
	vec3 actorZAxisDir = thetaMat * vec3(0, 0, 1);
	vec3 actorXAxisDir = nightyDegMat * actorZAxisDir;
	float errorXSignedLength = Dot(error, actorZAxisDir);
	float errorYSignedLength = Dot(error, actorXAxisDir);

	float thetaD = theta + atan2(errorYSignedLength, errorXSignedLength);
	// std::cout << "thetaD:" << thetaD * Rad2Deg << std::endl;
	//thetaMat.FromAxisAngle(vec3(0, 1, 0), 30 * Deg2Rad);

	thetaDMat.FromAxisAngle(vec3(0, 1, 0), thetaD);

	//if (error.Length() < 10)
	//{
	//	Vdesired = vec3(0.0, 0.0, 0.0);
	//}
	//else
	//{
		//Vdesired = actor->gMaxSpeed * thetaDMat * vec3(0, 0, 1);
	//}
	
	//Vdesired = vec3(0, 0, -50);
	
	// Vdesired = vec3(0, 0, )*/
	Vdesired = error.Normalize() * actor->gMaxSpeed;
	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	//vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 error = targetPos - actorPos;
	/*vec3 globalOrientation = actor->getOrientation();
	float theta = globalOrientation[1];

	mat3 thetaMat, thetaDMat, nightyDegMat;
	thetaMat.FromAxisAngle(vec3(0, 1, 0), globalOrientation[1]);
	nightyDegMat.FromAxisAngle(vec3(0, 1, 0), 90 * Deg2Rad);

	vec3 actorZAxisDir = thetaMat * vec3(0, 0, 1);
	vec3 actorXAxisDir = nightyDegMat * actorZAxisDir;
	float errorXSignedLength = Dot(error, actorZAxisDir);
	float errorYSignedLength = Dot(error, actorXAxisDir);

	float thetaD = theta + atan2(errorYSignedLength, errorXSignedLength);

	thetaDMat.FromAxisAngle(vec3(0, 1, 0), thetaD);*/

	// Vdesired = actor->gMaxSpeed * thetaDMat * vec3(0, 0, 1);
	Vdesired = error.Normalize() * actor->gMaxSpeed;

	return (-Vdesired);

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	// vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 error = targetPos - actorPos;
	vec3 Varrival = actor->KArrival * error;
	if (Varrival.Length() < 10)
	{
		Varrival = 0;
	}
	std::cout << "error:" << error << std::endl;
	std::cout << "error Length:" << error.Length() << std::endl;
	Vdesired = Varrival;
	



	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	// vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();

	// TODO: add your code here to compute Vdesired
	vec3 error = targetPos - actorPos;
	error = - (error / (error.Length() * error.Length()));
	vec3 Vdeparture = actor->KDeparture * error;
	std::cout << "Vdeparture:" << Vdeparture << std::endl;
	Vdesired = Vdeparture;

	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	vec3 targetPos = m_pTarget->getGlobalTranslation();

	//TODO: add your code here
	vec3 Varrival(0, 0, 0);
	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
	vec3 error = targetPos - m_actorPos;
	Varrival = actor->KArrival * error;
	if (Varrival.Length() < 10)
	{
		Varrival = 0;
	}
	//for(unsigned int i = 0; i < ; i++)
	//std::cout << "m_obstaclePos" << m_obstaclePos << std::endl;


	// Step 2. compute Lb
	//TODO: add your code here
	std::cout << "m_actorVel:" << m_actorVel << std::endl;
	std::cout << "m_actorVel Length:" << m_actorVel.Length() << std::endl;
	vec3 Lb = m_actorVel * actor->TAvoid;


	// Step 3. find closest obstacle 
	//TODO: add your code here
	Obstacle* cloestObstacle = nullptr;
	double dMinLength = DBL_MAX;
	for (unsigned int i = 0; i < mObstacles->size(); i++)
	{
		Obstacle* tempObstacle = &mObstacles->at(i);
		vec3 d = tempObstacle->m_Center.getGlobalTranslation() - m_actorPos;
		if (d.Length() < dMinLength)
		{
			dMinLength = d.Length();
			cloestObstacle = tempObstacle;
		}
	}

	m_obstaclePos = cloestObstacle->m_Center.getGlobalTranslation();

	if (cloestObstacle == nullptr)
	{
		std::cout << "ERROR: Does not find out the cloestest Obstacle." << std::endl;
		return vec3(0, 0, 0);
	}

	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	vec3 currVelDir = m_actorVel.Normalize();
	vec3 dWorld = cloestObstacle->m_Center.getGlobalTranslation() - m_actorPos;
	
	if (currVelDir.Length() < DBL_EPSILON)
	{
		// The current velocity is 0.
		currVelDir = actor->getGuide().getGlobalRotation() * vec3(0, 0, 1);
	}

	double vdDot = Dot(currVelDir, dWorld);
	double dXSign = 1;
	vec3 dX = vdDot * currVelDir;
	if (vdDot < 0)
	{
		dXSign = -1;
	}

	if (dXSign < 0)
	{
		Vdesired = Varrival;
	}
	else
	{
		vec3 dY = dWorld - dX;

		//std::cout << "Dot(dX, dY) Check:" << Dot(dX, dY) << std::endl;

		double LbLength = Lb.Length();
		double dXLength = dX.Length();
		double r0Length = cloestObstacle->m_Radius;
		double dyLength = dY.Length();
		std::cout << "dXLength:" << dXLength << std::endl;
		std::cout << "LbLength:" << LbLength << std::endl;
		std::cout << "Varrival:" << Varrival << std::endl;
		std::cout << "Varrival Length:" << Varrival.Length() << std::endl;

		vec3 Vavoid(0, 0, 0);
		//TODO: add your code here to compute Vavoid 
		if (dXLength > LbLength)
		{
			// No collision:
			Vdesired = Varrival;
		}
		else
		{
			if (dyLength > actor->gAgentRadius + r0Length)
			{
				// No collision:
				Vdesired = Varrival;
			}
			else
			{
				// Potential for collision:
				// Normal velocity field:
				vec3 avoidDir = -dY.Normalize();
				double avoidSpeed = (actor->KAvoid * ((actor->gAgentRadius + r0Length) - dyLength)) / (actor->gAgentRadius + r0Length);
				Vavoid = avoidSpeed * avoidDir;
				std::cout << "Vavoid:" << Vavoid << std::endl;

				// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
				//TODO: add your code here
				Vdesired = Varrival + Vavoid;

			}
		}
		if (Vdesired.Length() < 10)
		{
			Vdesired = 0;
		}
	}

	
	std::cout << std::endl;
	return Vdesired;
	
}

void Avoid::display(BehaviorController* actor)
{
#ifdef DEBUG_BEHAVIORDISPLAY
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	//vec3 dir = vec3(sin(angle[1]), 0, cos(angle[1]));
	vec3 probe = dir * (vel.Length() / BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;

	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
#endif
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}
unsigned int timeCounter = 0;
vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here
	srand(time(NULL) + timeCounter);
	timeCounter++;
	if (timeCounter > 100)
	{
		timeCounter = 0;
	}
	//vec3 randomSign = vec3(rand(), rand(), rand());
	//std::cout << "time(NULL):" << time(NULL) << std::endl;
	vec3 randomSign;
	for (unsigned int i = 0; i < 3; i++)
	{
		//float temp = randomSign[i] / float(int(RAND_MAX));
		//std::cout << "rand():" << rand() << std::endl;
		//std::cout << "temp:" << temp << std::endl;
		//std::cout << "float(int(RAND_MAX)):" << float(int(RAND_MAX))  << std::endl;
		int temp = rand() % 4;
		if (temp == 2 || temp == 0)
		{
			randomSign[i] = -1;
		}
		else
		{
			randomSign[i] = 1;
		}
	}
	//std::cout << "randomSign:" << randomSign << std::endl;

	vec3 noiseVec;
	for (unsigned int i = 0; i < 3; i++)
	{
		noiseVec[i] = float(rand()) * randomSign[i];
	}
	//std::cout << "noiseVec before normalize:" << noiseVec << std::endl;
	noiseVec = noiseVec.Normalize();
	//std::cout << "noiseVec:" << noiseVec << std::endl;

	// Step2. scale it with a noise factor
	//TODO: add your code here
	vec3 rNoise = noiseVec * actor->KNoise;


	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	m_Wander = actor->KWander * (m_Wander + rNoise).Normalize();
	//std::cout << "rNoise:" << rNoise << std::endl;
	//std::cout << "m_Wander:" << m_Wander << std::endl;


	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	Vdesired = -actorPos.Normalize() * 20 + m_Wander;



	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	//vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here
	
	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
		vec3 Valignment = vec3(0.0, 0.0, 0.0);
		double wi = 1.0;
		double wiAcc = 0.0;
		if (actor == leader)
		{
			vec3 error = targetPos - actorPos;
			vec3 Varrival = actor->KArrival * error;
			if (Varrival.Length() < 10)
			{
				Varrival = 0;
			}
			Vdesired = Varrival;
		}
		else
		{
			for (unsigned int i = 0; i < agentList.size(); i++)
			{
				AActor& currAgent = agentList.at(i);
			    if (currAgent.getBehaviorController() != actor)
				{
					vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
					vec3 di = currAgentPos - actorPos;
					double diLength = di.Length();
					if (diLength < actor->gKNeighborhood)
					{
						Valignment += (wi * currAgent.getBehaviorController()->getVelocity());
						wiAcc += wi;
					}
				}
			}
			if (abs(wiAcc) < DBL_EPSILON)
			{
				Vdesired = vec3(0, 0, 0);
			}
			else
			{
				Vdesired = Valignment / wiAcc;
			}
		}
		std::cout << "Vdesired:" << Vdesired << std::endl;
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 
	vec3 Vseparate = vec3(0.0, 0.0, 0.0);
	double wi = 1.0;
	for (unsigned int i = 0; i < agentList.size(); i++)
	{
		AActor& currAgent = agentList.at(i);
		if (currAgent.getBehaviorController() != actor)
		{
			vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
			vec3 di = currAgentPos - actorPos;
			double diLength = di.Length();
			if (diLength < actor->gKNeighborhood)
			{
				if (abs(diLength) < DBL_EPSILON)
				{
					Vseparate += (actor->gMaxSpeed * vec3(0, 0, 1));
				}
				else
				{
					Vseparate += (wi * (di / (diLength * diLength)));
				}
			}
		}
		// vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
		// currAgent.getBehaviorController()->getGuide().setGlobalTranslation(currAgent.getBehaviorController()->getGuide().getGlobalTranslation() + vec3(0, 0, 1));
		// currAgent.getBehaviorController()->getGuide().setLocalTranslation(vec3(-100, 0, -100));
	}
	Vseparate *= actor->KSeparation;
	Vdesired = - Vseparate;




	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 
	vec3 cm = vec3(0.0, 0.0, 0.0);
	double wi = 1.0;
	double wiAcc = 0.0;
	// find nearby vehicles
	// compute the center of mass
	for (unsigned int i = 0; i < agentList.size(); i++)
	{
		AActor& currAgent = agentList.at(i);
		if (currAgent.getBehaviorController() != actor)
		{
			vec3 d = currAgent.getBehaviorController()->getPosition() - actorPos;
			if (d.Length() < actor->gKNeighborhood)
			{
				wiAcc += wi;
				cm += currAgent.getBehaviorController()->getPosition() * wi;
			}
		}
	}
	if (abs(wiAcc) < DBL_EPSILON)
	{
		cm = actorPos;
	}
	else
	{
		cm /= wiAcc;
	}
	
	
	// 
	Vdesired = actor->KCohesion * (cm - actorPos);
	


	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	// Vseparate:
	vec3 Vseparate = vec3(0.0, 0.0, 0.0);
	double wi = 1.0;
	for (unsigned int i = 0; i < agentList.size(); i++)
	{
		AActor& currAgent = agentList.at(i);
		if (currAgent.getBehaviorController() != actor)
		{
			vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
			vec3 di = currAgentPos - actorPos;
			double diLength = di.Length();
			if (diLength < actor->gKNeighborhood)
			{
				if (abs(diLength) < DBL_EPSILON)
				{
					Vseparate += (actor->gMaxSpeed * vec3(0, 0, 1));
				}
				else
				{
					Vseparate += (wi * (di / (diLength * diLength)));
				}
			}
		}
	}
	Vseparate *= (-actor->KSeparation);
	


	// Vcohesion:
	vec3 cm = vec3(0.0, 0.0, 0.0);
	wi = 1.0;
	double wiAcc = 0.0;
	// find nearby vehicles
	// compute the center of mass
	for (unsigned int i = 0; i < agentList.size(); i++)
	{
		AActor& currAgent = agentList.at(i);
		if (currAgent.getBehaviorController() != actor)
		{
			vec3 d = currAgent.getBehaviorController()->getPosition() - actorPos;
			if (d.Length() < actor->gKNeighborhood)
			{
				wiAcc += wi;
				cm += currAgent.getBehaviorController()->getPosition() * wi;
			}
		}
	}
	if (abs(wiAcc) < DBL_EPSILON)
	{
		cm = actorPos;
	}
	else
	{
		cm /= wiAcc;
	}
	vec3 Vcohesion = actor->KCohesion * (cm - actorPos);

	// Valignment:
	vec3 Valignment = vec3(0.0, 0.0, 0.0);
	wi = 1.0;
	wiAcc = 0.0;
	if (actor == leader)
	{
		vec3 error = targetPos - actorPos;
		vec3 Varrival = actor->KArrival * error;
		if (Varrival.Length() < 10)
		{
			Varrival = 0;
		}
		Valignment = Varrival;
	}
	else
	{
		for (unsigned int i = 0; i < agentList.size(); i++)
		{
			AActor& currAgent = agentList.at(i);
			if (currAgent.getBehaviorController() != actor)
			{
				vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
				vec3 di = currAgentPos - actorPos;
				double diLength = di.Length();
				if (diLength < actor->gKNeighborhood)
				{
					Valignment += (wi * currAgent.getBehaviorController()->getVelocity());
					wiAcc += wi;
				}
			}
		}
		if (abs(wiAcc) < DBL_EPSILON)
		{
			Valignment = vec3(0, 0, 0);
		}
		else
		{
			Valignment = Valignment / wiAcc;
		}
	}
	// std::cout << "Vseparate:" << Vseparate << std::endl;
	// std::cout << "Vcohesion:" << Vcohesion << std::endl;
	// std::cout << "Valignment:" << Valignment << std::endl;
	Vdesired = Vseparate + Vcohesion + Valignment;

	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 4.0;  float CArrival = 2.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent
	
	// Vseparate:
	vec3 Vseparate = vec3(0.0, 0.0, 0.0);

	vec3 error;
	if (actor == leader)
	{
		error = targetPos - actorPos;
	}
	else
	{
		vec3 leaderVel = leader->getVelocity();
		double leaderSpeed = leaderVel.Length();
		vec3 leaderFaceDir = leader->getGuide().getGlobalRotation() * vec3(0, 0, 1);
		vec3 leaderFollowOffset = -leaderFaceDir * (leaderSpeed / 20.0 + 100.0);
		//vec3 actorFollowOffset = -actorFaceDir * (100.0);
		error = (leader->getPosition() + leaderFollowOffset) - actorPos;

		double wi = 1.0;
		for (unsigned int i = 0; i < agentList.size(); i++)
		{
			AActor& currAgent = agentList.at(i);
			if (currAgent.getBehaviorController() != actor)
			{
				vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
				vec3 di = currAgentPos - actorPos;
				double diLength = di.Length();
				if (diLength < actor->gKNeighborhood)
				{
					if (abs(diLength) < DBL_EPSILON)
					{
						Vseparate += (actor->gMaxSpeed * vec3(0, 0, 1));
					}
					else
					{
						Vseparate += (wi * (di / (diLength * diLength)));
					}
				}
			}
		}
		Vseparate *= (-actor->KSeparation);
	}
	//Vseparate = 0.0;

	// Leader Following:
	vec3 Varrival = actor->KArrival * error;
	if (Varrival.Length() < 10)
	{
		Varrival = 0;
	}

	if (actor != leader)
	{
		Vdesired = CArrival * Varrival + CSeparation * Vseparate;
	}
	else
	{
		Vdesired = Varrival;
	}
	
	/*
	if (actor == leader)
	{
		std::cout << "Leader's Vdesired:" << Vdesired << std::endl;
	}
	*/
	return Vdesired;
}


Pursuit::Pursuit(AJoint* target, vector<AActor>* agents)
{
	m_name = "pursuit";
	m_AgentList = agents;
	m_pTarget = target;
}

Pursuit::Pursuit(Pursuit& orig)
{
	m_name = "pursuit";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Pursuit::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vector<AActor>& agentList = *m_AgentList;
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	// other agents pursuit leader.
	if (actor == leader)
	{
		vec3 error = targetPos - actorPos;
		vec3 Vseek = actor->gMaxSpeed * error.Normalize();
		Vdesired = Vseek;
	}
	else
	{
		vec3 predictPos = leader->getPosition();
		// Velocity-based predictor
		// predictPos += (actor->mDeltaT * leader->getVelocity());
		predictPos += (0.5 * leader->getVelocity());
		vec3 error = predictPos - actorPos;

		// Vseparate:
		vec3 Vseparate = vec3(0.0, 0.0, 0.0);
		double wi = 1.0;
		for (unsigned int i = 0; i < agentList.size(); i++)
		{
			AActor& currAgent = agentList.at(i);
			if (currAgent.getBehaviorController() != actor)
			{
				vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
				vec3 di = currAgentPos - actorPos;
				double diLength = di.Length();
				if (diLength < actor->gKNeighborhood)
				{
					if (abs(diLength) < DBL_EPSILON)
					{
						Vseparate += (actor->gMaxSpeed * vec3(0, 0, 1));
					}
					else
					{
						Vseparate += (wi * (di / (diLength * diLength)));
					}
				}
			}
		}
		Vseparate *= (-actor->KSeparation);

		Vdesired = error.Normalize() * actor->gMaxSpeed + Vseparate;
	}	

	return Vdesired;
}

Pursuit::~Pursuit()
{

}

Evasion::Evasion(AJoint* target, vector<AActor>* agents)
{
	m_name = "evasion";
	m_AgentList = agents;
	m_pTarget = target;
}

Evasion::Evasion(Evasion& orig)
{
	m_name = "evasion";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Evasion::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vector<AActor>& agentList = *m_AgentList;
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	if (actor == leader)
	{
		vec3 error = targetPos - actorPos;
		vec3 Vseek = actor->gMaxSpeed * error.Normalize();
		Vdesired = Vseek;
	}
	else
	{
		vec3 predictPos = leader->getPosition();
		// Velocity-based predictor
		// predictPos += (actor->mDeltaT * leader->getVelocity());
		predictPos += (0.5 * leader->getVelocity());
		vec3 error = predictPos - actorPos;

		// Vseparate:
		vec3 Vseparate = vec3(0.0, 0.0, 0.0);
		double wi = 1.0;
		for (unsigned int i = 0; i < agentList.size(); i++)
		{
			AActor& currAgent = agentList.at(i);
			if (currAgent.getBehaviorController() != actor)
			{
				vec3 currAgentPos = currAgent.getBehaviorController()->getGuide().getGlobalTranslation();
				vec3 di = currAgentPos - actorPos;
				double diLength = di.Length();
				if (diLength < actor->gKNeighborhood)
				{
					if (abs(diLength) < DBL_EPSILON)
					{
						Vseparate += (actor->gMaxSpeed * vec3(0, 0, 1));
					}
					else
					{
						Vseparate += (wi * (di / (diLength * diLength)));
					}
				}
			}
		}
		Vseparate *= (-actor->KSeparation);

		Vdesired = - error.Normalize() * actor->gMaxSpeed + Vseparate;
	}


	return Vdesired;
}

Evasion::~Evasion()
{}

Formation::~Formation()
{

}

Formation::Formation(AJoint* target, vector<AActor>* agents)
{
	m_name = "formation";
	m_AgentList = agents;
	m_pTarget = target;
}

Formation::Formation(Formation& orig)
{
	m_name = "formation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Formation::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vector<AActor>& agentList = *m_AgentList;

	//vec3 teamFirstPos = vec3(0, 0, 0);
	vec3 teamFirstPos = targetPos;
	vec3 inTeamPos = vec3(0, 0, 0);
	unsigned int actorListPos = 0;

	for (unsigned int i = 0; i < agentList.size(); i++)
	{
		if (agentList.at(i).getBehaviorController() == actor)
		{
			actorListPos = i;
			break;
		}
	}

	unsigned int rowBelong = actorListPos / actor->row_num;
	vec3 rowFirstPos = teamFirstPos + vec3(0, 0, 1) * rowBelong * (-200.0);


	inTeamPos = rowFirstPos + (-200.0) * vec3(1, 0, 0) * (actorListPos - rowBelong * actor->row_num);

	vec3 error = inTeamPos - actorPos;
	vec3 Varrival = actor->KArrival * error;
	if (Varrival.Length() < 10)
	{
		Varrival = 0;
		//if (abs(actor->getOrientation()[1]) > 0.1)
		//{
			
		//}
	}
	Vdesired = Varrival;

	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

