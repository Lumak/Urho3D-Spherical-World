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
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <SDL/SDL_log.h>

#include "Lift.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
Lift::Lift(Context* context)
    : LogicComponent(context)
    , liftState_(LIFT_STATE_START)
    , liftButtonState_(LIFT_BUTTON_UP)
    , liftSpeed_(5.0f)
    , buttonPressed_(false)
    , buttonPressedHeight_(0.13f)
    , buttonDelayPopup_(false)
{
    SetUpdateEventMask(0);
}

Lift::~Lift()
{
}

void Lift::RegisterObject(Context* context)
{
    context->RegisterFactory<Lift>();
}

void Lift::Start()
{
}

void Lift::Initialize(Node *liftNode, const Vector3 &finishPosition)
{
    // get other lift components
    liftNode_        = liftNode;
    liftButtonNode_  = liftNode_->GetChild("LiftButton", true);

    assert( liftNode_ && liftButtonNode_ && "missing nodes!" );

    liftRigidBody_ = liftNode_->GetComponent<RigidBody>();
    massInitial_   = liftRigidBody_->GetMass();
    liftRigidBody_->SetMass(0.0f);

    // positions
    initialPosition_   = liftNode_->GetWorldPosition();
    finishPosition_    = finishPosition;
    directionToFinish_ = (finishPosition_ - initialPosition_).Normalized();
    totalDistance_     = (finishPosition_ - initialPosition_).Length();

    // button collision
    RigidBody* buttonBody = liftButtonNode_->GetComponent<RigidBody>();
    buttonBody->SetCollisionEventMode(COLLISION_ACTIVE);

    // single-shot events
    SubscribeToEvent(liftNode_, E_NODECOLLISIONSTART, URHO3D_HANDLER(Lift, HandleOnLiftCollision));
    SubscribeToEvent(liftNode_, E_NODECOLLISIONEND, URHO3D_HANDLER(Lift, HandleOffLiftCollision));
    SubscribeToEvent(liftButtonNode_, E_NODECOLLISIONSTART, URHO3D_HANDLER(Lift, HandleButtonStartCollision));
    SubscribeToEvent(liftButtonNode_, E_NODECOLLISIONEND, URHO3D_HANDLER(Lift, HandleButtonEndCollision));
}

void Lift::FixedUpdate(float timeStep)
{
    Vector3 liftPos   = liftRigidBody_->GetPosition();
    float startLinVel = liftRigidBody_->GetLinearVelocity().Length();
    float newLinVel   = (startLinVel > liftSpeed_)?liftSpeed_:Lerp(startLinVel, liftSpeed_, 0.02f);

    // move lift
    if (liftState_ == LIFT_STATE_MOVETO_FINISH)
    {
        Vector3 curDistance  = finishPosition_ - liftPos;
        Vector3 curDirection = curDistance.Normalized();
        float dist = curDistance.Length();
        float dotd = directionToFinish_.DotProduct(curDirection);

        if (dotd > 0.0f)
        {
            // slow down near the end
            if (dist < 1.0f)
            {
                float prevVel = newLinVel;
                newLinVel = Lerp(newLinVel, 0.5f, 1.0f - dist);

                AdjustBodySpeedOnLift(newLinVel, prevVel);
            }

            Vector3 linVel = directionToFinish_ * newLinVel;
            liftRigidBody_->SetLinearVelocity(linVel);
        }
        else
        {
            SetTransitionCompleted(LIFT_STATE_FINISH);
        }
    }
    else if (liftState_ == LIFT_STATE_MOVETO_START)
    {
        Vector3 curDistance  = initialPosition_ - liftPos;
        Vector3 curDirection = curDistance.Normalized();
        float dist = curDistance.Length();
        float dotd = directionToFinish_.DotProduct(curDirection);

        if (dotd < 0.0f)
        {
            // accleration on the way down - apply delta velocity
            // let physics take care of accleration on the way up
            if (totalDistance_ - dist < 1.0f)
            {
                AdjustBodySpeedOnLift(newLinVel, startLinVel);
            }
            
            // slow down near the end
            if (dist < 1.0f)
            {
                newLinVel = Lerp(newLinVel, 0.5f, 1.0f - dist);
            }

            Vector3 linVel = -directionToFinish_ * newLinVel;
            liftRigidBody_->SetLinearVelocity(linVel);
        }
        else
        {
            SetTransitionCompleted(LIFT_STATE_START);
        }
    }

    // reenable button
    if ( buttonDelayPopup_ && timerButtonPopup_.GetMSec(false) > 40.0f )
    {
        //liftButtonNode_->SetEnabled(true);
        buttonDelayPopup_ = false;
        SetUpdateEventMask(0);
    }
}

void Lift::AdjustBodySpeedOnLift(float curVel, float prevVel)
{
    float deltaVel = curVel - prevVel;

    // apply delta velocity on the body to prevent it from popping up
    // for multiple bodies, create a container and iterate the same
    if (bodyOnLift_)
    {
        bodyOnLift_->ApplyImpulse(Vector3(0, deltaVel, 0));
    }
}

void Lift::SetTransitionCompleted(int toState)
{
    if (toState == LIFT_STATE_START)
    {
        liftRigidBody_->SetPosition(initialPosition_);
    }
    else
    {
        liftRigidBody_->SetPosition(finishPosition_);
    }

    liftState_ = toState;
    liftRigidBody_->SetLinearVelocity(Vector3::ZERO);

    // change it to static to prevent it from moving while player body is on it
    liftRigidBody_->SetMass(0.0f);

    // adjust button
    if (liftButtonState_ == LIFT_BUTTON_UP)
    {
        ButtonPressAnimate(false);
    }

    if (!buttonDelayPopup_)
    {
        SetUpdateEventMask(0);
    }
}

void Lift::ButtonPressAnimate(bool pressed)
{
    if (pressed)
    {
        liftButtonNode_->SetPosition(liftButtonNode_->GetPosition() + Vector3(0, -buttonPressedHeight_, 0));
        buttonPressed_ = true;
    }
    else
    {
        liftButtonNode_->SetPosition(liftButtonNode_->GetPosition() + Vector3(0, buttonPressedHeight_, 0));
        buttonPressed_ = false;

        timerButtonPopup_.Reset();
        buttonDelayPopup_ = true;
        SetUpdateEventMask(USE_FIXEDUPDATE);
    }
}

void Lift::HandleOnLiftCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    bodyOnLift_ = (RigidBody*)eventData[P_OTHERBODY].GetVoidPtr();

    // for multiple bodies, create a container and check which body has entered the lift
}

void Lift::HandleOffLiftCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    bodyOnLift_ = NULL;

    // for multiple bodies, create a container and check which body has exited the lift
}

void Lift::HandleButtonStartCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    if (liftButtonState_ == LIFT_BUTTON_UP && !buttonDelayPopup_)
    {
        if (liftState_ == LIFT_STATE_START)
        {
            liftRigidBody_->SetMass(massInitial_);
            liftState_ = LIFT_STATE_MOVETO_FINISH;

            // adjust button
            ButtonPressAnimate(true);

            SetUpdateEventMask(USE_FIXEDUPDATE);
        }
        else if (liftState_ == LIFT_STATE_FINISH)
        {
            liftRigidBody_->SetMass(massInitial_);
            liftState_ = LIFT_STATE_MOVETO_START;

            // adjust button
            ButtonPressAnimate(true);

            SetUpdateEventMask(USE_FIXEDUPDATE);
        }
    
        liftButtonState_ = LIFT_BUTTON_DOWN;
    
        // play sound and animation
    }
}

void Lift::HandleButtonEndCollision(StringHash eventType, VariantMap& eventData)
{
    if (liftButtonState_ == LIFT_BUTTON_DOWN)
    {
        liftButtonState_ = LIFT_BUTTON_UP;
    
        // button animation
        if (liftState_ == LIFT_STATE_START || liftState_ == LIFT_STATE_FINISH)
        {
            if (buttonPressed_)
            {
                ButtonPressAnimate(false);
            }
        }
    }
}



