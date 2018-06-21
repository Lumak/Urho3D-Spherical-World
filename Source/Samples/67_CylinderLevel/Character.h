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

#pragma once

#include <Urho3D/Input/Controls.h>
#include <Urho3D/Scene/LogicComponent.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Physics/RigidBody.h>

//namespace Urho3D
//{
//class Node;
//}
using namespace Urho3D;

//=============================================================================
//=============================================================================
const int CTRL_FORWARD = 1;
const int CTRL_BACK = 2;
const int CTRL_LEFT = 4;
const int CTRL_RIGHT = 8;
const int CTRL_JUMP = 16;

const float MOVE_FORCE = 1.2f;
const float INAIR_MOVE_FORCE = 0.01f;
const float BRAKE_FORCE = 0.2f;
const float JUMP_FORCE = 7.0f;
const float YAW_SENSITIVITY = 0.01f;
const float INAIR_THRESHOLD_TIME = 0.1f;

//=============================================================================
//=============================================================================
/// Character component, responsible for physical movement according to controls, as well as animation.
class Character : public LogicComponent
{
    URHO3D_OBJECT(Character, LogicComponent);

public:
    /// Construct.
    Character(Context* context);
    
    /// Register object factory and attributes.
    static void RegisterObject(Context* context);
    
    virtual void Start();
    virtual void FixedUpdate(float timeStep);

    // set cam node
    void SetCamNode(Node* camNode) { camNode_ = camNode; }

public:
    Controls controls_;

protected:
    void HandleNodeCollision(StringHash eventType, VariantMap& eventData);
    void HandleTriggerEntered(StringHash eventType, VariantMap& eventData);
    void HandleTriggerExited(StringHash eventType, VariantMap& eventData);
    void SendCharOnLiftEvent(bool lerpY);
    
protected:
    bool onGround_;
    bool okToJump_;
    float inAirTimer_;

    // extra vars
    Vector3 curMoveDir_;
    bool isJumping_;
    bool jumpStarted_;

    // cam node
    WeakPtr<Node>  camNode_;

    // cylinder info
    bool           applyCylindericalGravity_;
    Vector3        cylinderCenter_;
    Vector3        gravCamDir_;
    Vector3        dirToCenterWorld_;
    unsigned       charPosMask_;
    float          cylinderDistance_;

    // no jump flag
    bool           jumpBlocked_;
};
