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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include "Character.h"
#include "CharacterDemo.h"
#include "Touch.h"
#include "Lift.h"
#include "CollisionLayer.h"
#include "TriggerMessage.h"
#include "SmoothStep.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
#define DEFAULT_CAM_PITCH       10.0f
#define DEFAULT_CAM_DISTANCE    10.0f

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(CharacterDemo)

//=============================================================================
//=============================================================================
CharacterDemo::CharacterDemo(Context* context) :
    Sample(context),
    firstPerson_(false),
    drawDebug_(false),
    freeCamera_(false),
    camPosition_(Vector3::ZERO),
    camRotation_(Quaternion::IDENTITY),
    charOnLift_(false),
    camDirIndex_(CamDir_Uninit),
    camDistance_(DEFAULT_CAM_DISTANCE),
    cylinderCamMode_(false)
{
    // Register factory and attributes for the Character component so it can be created via CreateComponent, and loaded / saved
    Character::RegisterObject(context);
    Lift::RegisterObject(context);
}

CharacterDemo::~CharacterDemo()
{
}

void CharacterDemo::Setup()
{
    engineParameters_["WindowTitle"]  = GetTypeName();
    engineParameters_["LogName"]      = GetSubsystem<FileSystem>()->GetProgramDir() + "shooterdemo.log";
    engineParameters_["FullScreen"]   = false;
    engineParameters_["Headless"]     = false;
    engineParameters_["WindowWidth"]  = 1280; 
    engineParameters_["WindowHeight"] = 720;
}

void CharacterDemo::Start()
{
    // Execute base class startup
    Sample::Start();
    if (touchEnabled_)
        touch_ = new Touch(context_, TOUCH_SENSITIVITY);

    ChangeDebugHudText();

    // Create static scene content
    CreateScene();

    // Create the controllable character
    CreateCharacter();

    // Create the UI content
    CreateInstructions();

    // Subscribe to necessary events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void CharacterDemo::ChangeDebugHudText()
{
    // change profiler text
    if (GetSubsystem<DebugHud>())
    {
        Text *dbgText = GetSubsystem<DebugHud>()->GetProfilerText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetStatsText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetMemoryText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetModeText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);
    }
}

void CharacterDemo::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);
    GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

    File loadFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Platforms/Levels/cylinderLevel.xml", FILE_READ);
    scene_->LoadXML(loadFile);

    // init lift
    Lift *lift = scene_->CreateComponent<Lift>();
    Node* liftNode = scene_->GetChild("Lift", true);
    Node* liftEndNode = scene_->GetChild("LiftTop", true);
    lift->Initialize(liftNode, liftEndNode->GetWorldPosition());
}

void CharacterDemo::CreateCharacter()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // spawn pos
    Node *spawnNode = scene_->GetChild("playerSpawn");

    // char node
    Node* objectNode = scene_->CreateChild("Character");
    objectNode->SetPosition(spawnNode->GetPosition());

    // spin node
    Node* adjustNode = objectNode->CreateChild("AdjNode");
    adjustNode->SetRotation( Quaternion(180, Vector3(0,1,0) ) );
    
    // Create the rendering component + animation controller
    AnimatedModel* object = adjustNode->CreateComponent<AnimatedModel>();
    object->SetModel(cache->GetResource<Model>("Platforms/Models/BetaLowpoly/Beta.mdl"));
    object->SetMaterial(0, cache->GetResource<Material>("Platforms/Materials/BetaBody_MAT.xml"));
    object->SetMaterial(1, cache->GetResource<Material>("Platforms/Materials/BetaBody_MAT.xml"));
    object->SetMaterial(2, cache->GetResource<Material>("Platforms/Materials/BetaJoints_MAT.xml"));
    object->SetCastShadows(true);
    adjustNode->CreateComponent<AnimationController>();

    // Create rigidbody, and set non-zero mass so that the body becomes dynamic
    RigidBody* body = objectNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(ColLayer_Character);
    body->SetCollisionMask(ColMask_Character);
    body->SetMass(1.0f);
    body->SetLinearDamping(0.4f);

    body->SetAngularFactor(Vector3::ZERO);
    body->SetCollisionEventMode(COLLISION_ALWAYS);

    // Set a capsule shape for collision
    CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
    shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.94f, 0.0f));

    character_ = objectNode->CreateComponent<Character>();

    // set cam node
    character_->SetCamNode(cameraNode_);

    // player light node
    playerLightNode_ = objectNode->CreateChild();
    playerLightNode_->SetName("playerLight");
    playerLightNode_->SetPosition(Vector3(0,3,0));
    Light* light = playerLightNode_->CreateComponent<Light>();
    light->SetLightType(LIGHT_POINT);
    light->SetCastShadows(true);
    light->SetRange(4.0f);
    playerLightNode_->SetEnabled(false);
}

void CharacterDemo::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    //instructionText->SetText(
    //    "Use WASD keys and mouse/touch to move\n"
    //    "Space to jump, F to toggle 1st/3rd person\n"
    //    "F5 to save scene, F7 to load"
    //);
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void CharacterDemo::SubscribeToEvents()
{
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostUpdate));
    UnsubscribeFromEvent(E_SCENEUPDATE);

    // register for camera triggers
    PODVector<Node*> camTrigNodes = scene_->GetChildrenWithTag("camTrigger", true);

    for ( unsigned i = 0; i < camTrigNodes.Size(); ++i )
    {
        SubscribeToEvent(camTrigNodes[i], E_NODECOLLISIONSTART, URHO3D_HANDLER(CharacterDemo, HandleCamTriggerEntered));
        SubscribeToEvent(camTrigNodes[i], E_NODECOLLISIONEND, URHO3D_HANDLER(CharacterDemo, HandleCamTriggerExited));
    }

    // register for light triggers
    PODVector<Node*> lightTrigNodes = scene_->GetChildrenWithTag("lightTrigger", true);

    for ( unsigned i = 0; i < lightTrigNodes.Size(); ++i )
    {
        SubscribeToEvent(lightTrigNodes[i], E_NODECOLLISIONSTART, URHO3D_HANDLER(CharacterDemo, HandleLightTriggerEntered));
    }

    // register for char on lift event
    SubscribeToEvent(E_CHAR_ONLIFT, URHO3D_HANDLER(CharacterDemo, HandleCharOnLiftEvent));
}

void CharacterDemo::HandleCamTriggerEntered(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;
    {
        Node *bodyNode = ((RigidBody*)eventData[P_BODY].GetVoidPtr())->GetNode();
        const StringVector& tagList = bodyNode->GetTags();
        int prevCamIdx = camDirIndex_;

        // parse conditions
        for ( unsigned i = 0; i < tagList.Size(); ++i )
        {
            const String &tag = tagList[i];

            if (tag.StartsWith("camDir_"))
            {
                if (tag.EndsWith("XPos"))
                {
                    camDirIndex_ = CamDir_XPos;
                }
                else if (tag.EndsWith("XNeg"))
                {
                    camDirIndex_ = CamDir_XNeg;
                }
                else if (tag.EndsWith("ZPos"))
                {
                    camDirIndex_ = CamDir_ZPos;
                }
                else if (tag.EndsWith("ZNeg"))
                {
                    camDirIndex_ = CamDir_ZNeg;
                }

                if (tag.EndsWith("CylinderCenter"))
                {
                    cylinderCamMode_ = true;
                    cylinderPos_ = bodyNode->GetWorldPosition();
                }
            }
        }

        // init cam pos/rot
        if (prevCamIdx == CamDir_Uninit && camDirIndex_ != CamDir_Uninit)
        {
            if (character_)
            {
                Vector3 upVec, fwdVec, rgtVec;

                if (GetCameraVectors(rgtVec, upVec, fwdVec))
                {
                    camRotation_ = Quaternion(rgtVec, upVec, fwdVec);
                    camPosition_ = character_->GetNode()->GetWorldPosition();
                }
            }
        }
    }
}

void CharacterDemo::HandleCamTriggerExited(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;
    {
        Node *bodyNode = ((RigidBody*)eventData[P_BODY].GetVoidPtr())->GetNode();
        const StringVector& tagList = bodyNode->GetTags();

        // parse conditions
        for ( unsigned i = 0; i < tagList.Size(); ++i )
        {
            const String &tag = tagList[i];

            if (tag.StartsWith("camDir_"))
            {
                if (tag.EndsWith("CylinderCenter"))
                {
                    cylinderCamMode_ = false;
                }
            }
        }
    }
}

void CharacterDemo::HandleLightTriggerEntered(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;
    Node *bodyNode = ((RigidBody*)eventData[P_BODY].GetVoidPtr())->GetNode();
    const StringVector& tagList = bodyNode->GetTags();

    // parse conditions
    for ( unsigned i = 0; i < tagList.Size(); ++i )
    {
        const String &tag = tagList[i];

        if (tag.StartsWith("globalLight_"))
        {
            Node *globalLtNode = scene_->GetChild("GlobalLight", true);
            if (tag.EndsWith("Enable"))
            {
                globalLtNode->SetEnabled(true);
                playerLightNode_->SetEnabled(false);
            }
            else
            {
                globalLtNode->SetEnabled(false);
                playerLightNode_->SetEnabled(true);
            }
        }
    }
}

void CharacterDemo::HandleCharOnLiftEvent(StringHash eventType, VariantMap& eventData)
{
    using namespace CharOnLift;

    charOnLift_ = eventData[P_ONLIFT].GetBool();
}

void CharacterDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    Input* input = GetSubsystem<Input>();
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    if (input->GetKeyPress(KEY_F6))
        freeCamera_ =!freeCamera_;
    
    if (freeCamera_)
    {
        MoveCamera(timeStep);
        return;
    }
    if (character_)
    {
        // Clear previous controls
        character_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT | CTRL_JUMP, false);

        // Update controls using touch utility class
        if (touch_)
            touch_->UpdateTouches(character_->controls_);

        // Update controls using keys
        UI* ui = GetSubsystem<UI>();
        if (!ui->GetFocusElement())
        {
            if (!touch_ || !touch_->useGyroscope_)
            {
                character_->controls_.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
                character_->controls_.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
                character_->controls_.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
                character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
            }
            character_->controls_.Set(CTRL_JUMP, input->GetKeyDown(KEY_SPACE));

            // Add character yaw & pitch from the mouse motion or touch input
            if (touchEnabled_)
            {
                for (unsigned i = 0; i < input->GetNumTouches(); ++i)
                {
                    TouchState* state = input->GetTouch(i);
                    if (!state->touchedElement_)    // Touch on empty space
                    {
                        Camera* camera = cameraNode_->GetComponent<Camera>();
                        if (!camera)
                            return;

                        Graphics* graphics = GetSubsystem<Graphics>();
                        character_->controls_.yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                        character_->controls_.pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;
                    }
                }
            }
            else
            {
                character_->controls_.yaw_ += (float)input->GetMouseMoveX() * 0.1f;
                character_->controls_.pitch_ += (float)input->GetMouseMoveY() * 0.1f;
            }
            // Limit pitch
            character_->controls_.pitch_ = Clamp(character_->controls_.pitch_, -80.0f, 80.0f);

            // Turn on/off gyroscope on mobile platform
            if (touch_ && input->GetKeyPress(KEY_G))
                touch_->useGyroscope_ = !touch_->useGyroscope_;
        }
    }

    if (input->GetKeyPress(KEY_F5))
        drawDebug_ = !drawDebug_;
}

void CharacterDemo::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (drawDebug_)
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);

    if (freeCamera_)
        return;

    if (!character_)
        return;

    using namespace Update;

    float timeStep = eventData[P_TIMESTEP].GetFloat();
    Node* characterNode = character_->GetNode();

    Vector3 charPos = characterNode->GetWorldPosition();
    Vector3 upVec, fwdVec, rgtVec;
    Quaternion rot, dir;
    float camDistance = DEFAULT_CAM_DISTANCE;

    // get cam vectors and calc rot
    GetCameraVectors(rgtVec, upVec, fwdVec);
    rot = Quaternion(rgtVec, upVec, fwdVec);

    if (cylinderCamMode_)
    {
        Vector3 toCenter = (cylinderPos_ - charPos);
        toCenter.y_ = 0.0f;
        toCenter.Normalize();
        rot = Quaternion(Vector3::FORWARD, toCenter);

        // change cam dist
        camDistance = DEFAULT_CAM_DISTANCE * 1.75f;
    }

    // smooth pos/rot
    const float rotLerpRate = 6.0f;
    const float posLerpRate = 10.0f;
    const float distLerpRate = 2.0f;

    // stay close to the char.y_ when on a lift
    if (charOnLift_)
    {
        camPosition_.y_ = charPos.y_;
    }

    // smooth cam
    camPosition_ = SmoothStep(camPosition_, charPos, timeStep * posLerpRate);
    camRotation_ = SmoothStepAngle(camRotation_, rot, timeStep * rotLerpRate);
    camDistance_ = Lerp(camDistance_, camDistance, timeStep * distLerpRate);

    // default cam pitch
    character_->controls_.pitch_ = DEFAULT_CAM_PITCH;
    dir = camRotation_ * Quaternion(character_->controls_.pitch_, Vector3::RIGHT);

    {
        Vector3 aimPoint = camPosition_ + camRotation_ * Vector3(0.0f, 1.7f, 0.0f);
        Vector3 rayDir = dir * Vector3::BACK;
        float rayDistance = touch_ ? touch_->cameraDistance_ : camDistance_;
        PhysicsRaycastResult result;
        scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(aimPoint, rayDir), rayDistance, ColMask_Camera);
        if (result.body_)
            rayDistance = Min(rayDistance, result.distance_);
        rayDistance = Clamp(rayDistance, CAMERA_MIN_DIST, CAMERA_MAX_DIST);

        cameraNode_->SetPosition(aimPoint + rayDir * rayDistance);
        cameraNode_->SetRotation(dir);
    }
}

bool CharacterDemo::GetCameraVectors(Vector3 &right, Vector3 &up, Vector3 &forward)
{
    if (!character_)
        return false;

    Node* characterNode = character_->GetNode();
    up = characterNode->GetUp();
    forward = Vector3::FORWARD;

    // current cam setting
    switch (camDirIndex_)
    {
    case CamDir_XPos:
        forward = Vector3::RIGHT;
        break;
    case CamDir_XNeg:
        forward = Vector3::LEFT; 
        break;
    case CamDir_ZPos:
        forward = Vector3::FORWARD; 
        break;
    case CamDir_ZNeg:
        forward = Vector3::BACK; 
        break;
    }
    right = up.CrossProduct(forward);
    right.Normalize();

    return true;
}

void CharacterDemo::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    Input* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
}


