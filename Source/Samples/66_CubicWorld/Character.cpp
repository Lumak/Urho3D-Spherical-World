//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Math/Ray.h>

#include <Urho3D/Physics/PhysicsUtils.h>
#include <Bullet/BulletDynamics/Dynamics/btRigidBody.h>

#include <SDL/SDL_log.h>
#include "Character.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
#define FIXED_GRAVITY           9.81f
#define MAX_STEPDOWN_HEIGHT     0.5f
#define DEFAULT_NUMFRAMES_LERP  6.0f

//=============================================================================
//=============================================================================
Character::Character(Context* context) :
    LogicComponent(context),
    onGround_(false),
    okToJump_(true),
    inAirTimer_(0.0f),
    jumpStarted_(false),
    prevYaw_(0.0f),
    curDistToWorld_(1.0f),
    defaultNumFramesToLerp_(DEFAULT_NUMFRAMES_LERP),
    isInTransition_(false)
{
    // Only the physics update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_FIXEDUPDATE | USE_FIXEDPOSTUPDATE);
}

void Character::RegisterObject(Context* context)
{
    context->RegisterFactory<Character>();

    // These macros register the class attributes to the Context for automatic load / save handling.
    // We specify the Default attribute mode which means it will be used both for saving into file, and network replication
    URHO3D_ATTRIBUTE("Controls Yaw", float, controls_.yaw_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Controls Pitch", float, controls_.pitch_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("On Ground", bool, onGround_, false, AM_DEFAULT);
    URHO3D_ATTRIBUTE("OK To Jump", bool, okToJump_, true, AM_DEFAULT);
    URHO3D_ATTRIBUTE("In Air Timer", float, inAirTimer_, 0.0f, AM_DEFAULT);
}

void Character::Start()
{
    // get the world sphere node
    worldNode_ = GetScene()->GetChild("WorldNode");

    // init dir to center of the world
    dirToCenterWorld_ = node_->GetWorldUp() * -1.0f;
    RaycastToWorld();

    // Component has been inserted into its scene node. Subscribe to events now
    SubscribeToEvent(GetNode(), E_NODECOLLISION, URHO3D_HANDLER(Character, HandleNodeCollision));
}

void Character::FixedUpdate(float timeStep)
{
    /// \todo Could cache the components for faster access instead of finding them each frame
    RigidBody* body = GetComponent<RigidBody>();
    AnimationController* animCtrl = node_->GetComponent<AnimationController>(true);

    // Update the in air timer. Reset if grounded
    if (!onGround_)
        inAirTimer_ += timeStep;
    else
        inAirTimer_ = 0.0f;
    // When character has been in air less than 1/10 second, it's still interpreted as being on ground
    bool softGrounded = inAirTimer_ < INAIR_THRESHOLD_TIME;

    // Update movement & animation
    const Quaternion& rot = body->GetRotation();
    Vector3 moveDir = Vector3::ZERO;

    if (controls_.IsDown(CTRL_FORWARD))
        moveDir = rot * Vector3::FORWARD;
    if (controls_.IsDown(CTRL_BACK))
        moveDir = rot * Vector3::BACK;
    if (controls_.IsDown(CTRL_LEFT))
        moveDir += rot * Vector3::LEFT;
    if (controls_.IsDown(CTRL_RIGHT))
        moveDir += rot * Vector3::RIGHT;

    // raycast to world and apply gravity
    RaycastToWorld();

    // Normalize move vector so that diagonal strafing is not faster
    if (moveDir.LengthSquared() > 0.0f)
    {
        moveDir.Normalize();
    }

    // If in air, allow control, but slower than when on ground
    curMoveDir_ = moveDir;
    body->ApplyImpulse(curMoveDir_ * (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));

    if (softGrounded)
    {
        // brake force
        Vector3 upDirCenterWorld = dirToCenterWorld_ * -1.0f;
        Vector3 linVel = body->GetLinearVelocity();
        float relVel = upDirCenterWorld.DotProduct(linVel);
        Vector3 lateralFrictionDir = linVel - upDirCenterWorld * relVel;
        if (lateralFrictionDir.LengthSquared() > M_EPSILON)
        {
            lateralFrictionDir *= 1.0f/lateralFrictionDir.Length();
            Vector3 brakeForce = (linVel.Length() * -lateralFrictionDir) * BRAKE_FORCE;
            body->ApplyImpulse(brakeForce);
        }

        // Jump. Must release jump control between jumps
        isJumping_ = false;
        if (controls_.IsDown(CTRL_JUMP))
        {
            isJumping_ = true;
            if (okToJump_)
            {
                okToJump_ = false;
                jumpStarted_ = true;

                body->ApplyImpulse(-dirToCenterWorld_ * JUMP_FORCE);

                animCtrl->StopLayer(0);
                animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_JumpStart.ani", 0, false, 0.2f);
                animCtrl->SetTime("Platforms/Models/BetaLowpoly/Beta_JumpStart.ani", 0);
            }
        }
        else
            okToJump_ = true;
    }

    if ( !onGround_ || jumpStarted_ )
    {
        if (jumpStarted_)
        {
            if (animCtrl->IsAtEnd("Platforms/Models/BetaLowpoly/Beta_JumpStart.ani"))
            {
                animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_JumpLoop1.ani", 0, true, 0.3f);
                animCtrl->SetTime("Platforms/Models/BetaLowpoly/Beta_JumpLoop1.ani", 0);
                jumpStarted_ = false;
            }
        }
        else
        {
            if (curDistToWorld_ > MAX_STEPDOWN_HEIGHT )
            {
                animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_JumpLoop1.ani", 0, true, 0.2f);
            }
        }
    }
    else
    {
        // Play walk animation if moving on ground, otherwise fade it out
        if ((softGrounded) && !moveDir.Equals(Vector3::ZERO))
        {
            animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_Run.ani", 0, true, 0.2f);
        }
        else
        {
            animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_Idle.ani", 0, true, 0.2f);
        }

        // Set walk animation speed proportional to velocity
        //animCtrl->SetSpeed("Shooter/Models/Beta/Beta_Run.ani", planeVelocity.Length() * 0.3f);
    }

    // Reset grounded flag for next frame
    onGround_ = false;
}

void Character::RaycastToWorld()
{
    RigidBody* body = GetComponent<RigidBody>();
    Vector3 bodyUpPos = body->GetPosition();
    Vector3 castRayVec = worldNode_->GetWorldPosition() - bodyUpPos;
    Vector3 castRayN = castRayVec.Normalized();

    curDistToWorld_ = castRayVec.Length();
    prevDirToCenterWorld_ = dirToCenterWorld_;

    PhysicsRaycastResult result;
    GetScene()->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(bodyUpPos, castRayN), curDistToWorld_, ColMask_WorldRayCast);

    if (result.body_)
    {
        dirToCenterWorld_ = result.normal_ * -1.0f;
        curDistToWorld_ = result.distance_;
    }
}

void Character::FixedPostUpdate(float timeStep)
{
    UpdateOrientation();
}

void Character::UpdateOrientation()
{
    Vector3 upVecCenterWorld = dirToCenterWorld_ * -1.0f;
    Vector3 newRgt = upVecCenterWorld.CrossProduct( node_->GetWorldDirection() ).Normalized();
    Vector3 newFwd = newRgt.CrossProduct(upVecCenterWorld).Normalized();
    Quaternion bodyRot(newRgt, upVecCenterWorld, newFwd);

    // calculate transitional values on surface change
    if (prevDirToCenterWorld_.DotProduct(dirToCenterWorld_) < 0.1f)
    {
        if (!isInTransition_)
        {
            Quaternion dRot(prevDirToCenterWorld_, dirToCenterWorld_);
            newFwd = dRot * node_->GetWorldDirection();
            newRgt = dRot * node_->GetWorldRight();

            finalRotation_ = Quaternion(newRgt, upVecCenterWorld, newFwd);
            transRotation_ = node_->GetWorldRotation();
            numFramesToLerp_ = defaultNumFramesToLerp_;
            isInTransition_ = true;

            // prevent surface toggling when the character is running along an edge
            float dotN = node_->GetWorldDirection().AbsDotProduct(-dirToCenterWorld_);
            if (!IsNaN(dotN))
            {
                numFramesToLerp_ = Max(numFramesToLerp_ * dotN, 1.0f);
            }
        }
        else 
        {
            Vector3 curDwnVec = transRotation_ * Vector3::DOWN;

            if (curDwnVec.AbsDotProduct(dirToCenterWorld_) < 0.1f)
            {
                Vector3 curFwdVec = transRotation_ * Vector3::FORWARD;
                Quaternion dRot(curDwnVec, dirToCenterWorld_);
                newFwd = dRot * curFwdVec;
                newRgt = dRot * transRotation_ * Vector3::RIGHT;

                finalRotation_ = Quaternion(newRgt, upVecCenterWorld, newFwd);
                numFramesToLerp_ = defaultNumFramesToLerp_;

                // prevent surface toggling when the character is running along an edge
                float dotN = curFwdVec.AbsDotProduct(-dirToCenterWorld_);
                if (!IsNaN(dotN))
                {
                    numFramesToLerp_ = Max(numFramesToLerp_ * dotN, 1.0f);
                }
            }
        }
    }

    // check for transition completion
    if (isInTransition_)
    {
        transRotation_ = transRotation_.Slerp(finalRotation_, 1.0f / numFramesToLerp_).Normalized();
        bodyRot = transRotation_;
        float dottrans = Abs(transRotation_.DotProduct(finalRotation_));

        if (dottrans > 0.99f)
        {
            isInTransition_ = false;
        }
    }

    if (!Equals(prevYaw_, controls_.yaw_) )
    {
        float dyaw = controls_.yaw_ - prevYaw_;
        prevYaw_ = controls_.yaw_;

        Quaternion yrot(dyaw, Vector3::UP);
        bodyRot = bodyRot * yrot;

        if (isInTransition_)
        {
            finalRotation_ = finalRotation_ * yrot;
            transRotation_ = transRotation_ * yrot;
        }
    }

    node_->SetWorldRotation(bodyRot);

    // set gravity vec
    RigidBody* body = GetComponent<RigidBody>();
    Vector3 gravityToWorld = (bodyRot * Vector3::DOWN) * FIXED_GRAVITY;
    body->SetGravityOverride(gravityToWorld);
}

void Character::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    // Check collision contacts and see if character is standing on ground (look for a contact that has near vertical normal)
    using namespace NodeCollision;

    // skip triggers for now
    if (((RigidBody*)eventData[P_OTHERBODY].GetVoidPtr())->IsTrigger() )
        return;

    MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());
    Vector3 charUpVec = node_->GetWorldUp();
    Vector3 charDwnVec = charUpVec * -1.0f;
    Vector3 charPosUp = node_->GetWorldPosition() + charUpVec;
    
    while (!contacts.IsEof())
    {
        Vector3 contactPosition = contacts.ReadVector3();
        Vector3 contactNormal = contacts.ReadVector3();
        /*float contactDistance = */contacts.ReadFloat();
        /*float contactImpulse = */contacts.ReadFloat();

        // touching something that could be a ground surface
        Vector3 dirToSurf = (contactPosition - charPosUp).Normalized();
        if (dirToSurf.DotProduct(charDwnVec) > 0.5f && contactNormal.DotProduct(charUpVec) > 0.5f)
        {
            onGround_ = true;
            break;
        }
    }
}
