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
#include "TriggerMessage.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
#define FIXED_GRAVITY           9.81f
#define MAX_STEPDOWN_HEIGHT     0.5f

//=============================================================================
//=============================================================================
Character::Character(Context* context) :
    LogicComponent(context),
    onGround_(false),
    okToJump_(true),
    inAirTimer_(0.0f),
    jumpStarted_(false),
    applyCylindericalGravity_(false),
    charPosMask_(0),
    cylinderDistance_(0.0f),
    jumpBlocked_(false),
    gravCamDir_(Vector3::ZERO)
{
    // Only the physics update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_FIXEDUPDATE);
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
    // init char anim, so we don't see the t-pose char as it's spawned
    AnimationController* animCtrl = node_->GetComponent<AnimationController>(true);
    animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_JumpLoop1.ani", 0, true, 0.0f);

    // register for charDynamicsTriggers
    PODVector<Node*> trigNodes = GetScene()->GetChildrenWithTag("charDynamicsTrigger", true);

    for ( unsigned i = 0; i < trigNodes.Size(); ++i )
    {
        SubscribeToEvent(trigNodes[i], E_NODECOLLISIONSTART, URHO3D_HANDLER(Character, HandleTriggerEntered));
        SubscribeToEvent(trigNodes[i], E_NODECOLLISIONEND, URHO3D_HANDLER(Character, HandleTriggerExited));
    }

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

    // override gravity
    if (applyCylindericalGravity_)
    {
        dirToCenterWorld_ = (body->GetPosition() - cylinderCenter_);

        // **note** the z_ offset is negligible for a large cylinder, but we're zero'ing it out for this demo
        // see "gravCamAxis_" suffix to know which axis (vector) can be zero'd out
        dirToCenterWorld_.z_ = 0.0f;
        dirToCenterWorld_.Normalize();
        body->SetGravityOverride(dirToCenterWorld_ * FIXED_GRAVITY);
    }
    else
    {
        dirToCenterWorld_ = Vector3::DOWN;
        body->SetGravityOverride(dirToCenterWorld_ * FIXED_GRAVITY);
    }

    // Update movement & animation
    Quaternion rot = camNode_->GetWorldRotation();
    if (gravCamDir_ != Vector3::ZERO )
    {
        Vector3 upv = dirToCenterWorld_ * -1.0f;
        Vector3 fwd = gravCamDir_;
        Vector3 rgt = upv.CrossProduct(fwd);
        rot = Quaternion(rgt, upv, fwd);
    }

    // get movedir based on camera rotation
    Vector3 moveDir = Vector3::ZERO;

    if (controls_.IsDown(CTRL_FORWARD))
        moveDir = rot * Vector3::FORWARD;
    if (controls_.IsDown(CTRL_BACK))
        moveDir = rot * Vector3::BACK;
    if (controls_.IsDown(CTRL_LEFT))
        moveDir += rot * Vector3::LEFT;
    if (controls_.IsDown(CTRL_RIGHT))
        moveDir += rot * Vector3::RIGHT;

    // ** flatten y_ in a non-cylinderical gravity area
    if (gravCamDir_ == Vector3::ZERO )
    {
        moveDir.y_= 0.0f;
    }

    // adjust body rotation based on movedir
    if (moveDir.LengthSquared() > 0.0f)
    {
        moveDir.Normalize();
        curMoveDir_ = moveDir;
        Vector3 upv = dirToCenterWorld_ *-1.0f;
        Vector3 fwd = curMoveDir_;
        Vector3 rgt = (upv.CrossProduct(fwd)).Normalized();

        body->SetRotation(Quaternion(rgt, upv, fwd));

        body->ApplyImpulse(curMoveDir_* (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));
    }
    // If in air, allow control, but slower than when on ground
    curMoveDir_ = moveDir;

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

        // jump if not blocked (not on a lift)
        if (!jumpBlocked_)
        {
            isJumping_ = false;
            if (controls_.IsDown(CTRL_JUMP))
            {
                isJumping_ = true;
                if (okToJump_)
                {
                    okToJump_ = false;
                    jumpStarted_ = true;

                    float jumpForce = JUMP_FORCE;
                    body->ApplyImpulse(-dirToCenterWorld_ * jumpForce);

                    animCtrl->StopLayer(0);
                    animCtrl->PlayExclusive("Platforms/Models/BetaLowpoly/Beta_JumpStart.ani", 0, false, 0.2f);
                    animCtrl->SetTime("Platforms/Models/BetaLowpoly/Beta_JumpStart.ani", 0);
                }
            }
            else
                okToJump_ = true;
        }
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
            const float rayDistance = 50.0f;
            PhysicsRaycastResult result;
            GetScene()->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(node_->GetPosition(), dirToCenterWorld_), rayDistance, 0xff);
            
            if (result.body_ && result.distance_ > MAX_STEPDOWN_HEIGHT )
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
        //animCtrl->SetSpeed("Shooter/Models/BetaLowpoly/Beta_Run.ani", planeVelocity.Length() * 0.3f);
    }

    // Reset grounded flag for next frame
    onGround_ = false;
}

void Character::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    // Check collision contacts and see if character is standing on ground (look for a contact that has near vertical normal)
    using namespace NodeCollision;

    // skip triggers
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

void Character::HandleTriggerEntered(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    RigidBody *rbody = (RigidBody*)eventData[P_BODY].GetVoidPtr();
    Node *bodyNode = rbody->GetNode();
    const StringVector& tagList = bodyNode->GetTags();

    // parse conditions
    for ( unsigned i = 0; i < tagList.Size(); ++i )
    {
        const String &tag = tagList[i];

        if (tag.StartsWith("charPos_"))
        {
            if (tag.EndsWith("Normal"))
            {
                charPosMask_ |= CharPosMask_Normal;
            }
            else if (tag.EndsWith("OnLift"))
            {
                jumpBlocked_ = true;
                SendCharOnLiftEvent(true);
            }

            unsigned tagPos = tag.Find("charPos_CylinderLen_");
            if (tagPos != String::NPOS)
            {
                charPosMask_ |= CharPosMask_CylinderCenter;
                cylinderDistance_ = (float)atoi(&tag.CString()[strlen("charPos_CylinderLen_")]);
                cylinderCenter_ = rbody->GetPosition();
            }

        }
        else if (tag.StartsWith("gravDir_"))
        {
            if (tag.EndsWith("Normal"))
            {
                applyCylindericalGravity_ = false;
            }
            else
            {
                unsigned tagPos = tag.Find("CylinderCenter");
                if (tagPos != String::NPOS)
                {
                    applyCylindericalGravity_ = true;
                    cylinderCenter_ = rbody->GetPosition();
                }
            }
        }
        else 
        {
            if (tag.Compare("gravCamAxis_ZPos") == 0) 
            {
                gravCamDir_ = Vector3::FORWARD; // ZPos = v(0,0,1)
            }
        }
    }
}

void Character::HandleTriggerExited(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    RigidBody *rbody = (RigidBody*)eventData[P_BODY].GetVoidPtr();
    Node *bodyNode = rbody->GetNode();
    const StringVector& tagList = bodyNode->GetTags();

    // parse conditions
    for ( unsigned i = 0; i < tagList.Size(); ++i )
    {
        const String &tag = tagList[i];

        if (tag.StartsWith("charPos_"))
        {
            if (tag.EndsWith("Normal"))
            {
                charPosMask_ &= ~CharPosMask_Normal;
            }
            else if (tag.EndsWith("OnLift"))
            {
                jumpBlocked_ = false;
                SendCharOnLiftEvent(false);
            }

            unsigned tagPos = tag.Find("CylinderLen_");
            if (tagPos != String::NPOS)
            {
                charPosMask_ &= ~CharPosMask_CylinderCenter;
            }
        }

        if (tag.Compare("gravCamAxis_ZPos") == 0) 
        {
            gravCamDir_ = Vector3::ZERO;
        }
    }
}

void Character::SendCharOnLiftEvent(bool onLift)
{
    using namespace CharOnLift;

    VariantMap eventData;
    eventData[P_ONLIFT] = onLift;

    SendEvent(E_CHAR_ONLIFT, eventData);
}

