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

#include <Urho3D/DebugNew.h>

//=============================================================================
// CatmullRom smooth fn (alternative to simple lerp)
// - substitue p0 = p1 and p3 = p2
//=============================================================================
//template <typename T> Variant CalculateCatmullRom(const T& p0, const T& p1, const T& p2, const T& p3, float t, float t2, float t3)
//{
//    return Variant(0.5f * ((2.0f * p1) + (-p0 + p2) * t +
//        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
//        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3));
//}
//
// *** substitue p0 = p1 and p3 = p2 ***
//Variant(0.5f * ((2.0f * p1) + (-p1 + p2) * t +
//        (2.0f * p1 - 5.0f * p1 + 4.0f * p2 - p2) * t2 +
//        (-p1 + 3.0f * p1 - 3.0f * p2 + p2) * t3));
//
//Variant(0.5f * ( 2.0f * p1 + (-p1 + p2) * t + ( -3.0f * p1 + 3.0f * p2) * t2 + ( 2.0f * p1 - 2.0f * p2) * t3));
//
//Variant(0.5f * ( 2.0f*p1 + (-p1 + p2) * t + (-p1 + p2) * 3.0f*t2 + (-p1 + p2) * -2.0f*t3));
//
//Variant(p1 + (-p1 + p2) * 0.5f*t + (-p1 + p2) * 1.5f*t2 + (-p1 + p2) * -t3);
//
//Variant(p1 + (p2-p1)*(0.5f*t + 1.5f*t2 - t3));

template <typename T>
T SmoothStep(const T &p1, const T &p2, float t, float tolerance=0.001f)
{
    if (t < M_EPSILON) 
        return p1;
    if (t > 1.0f - M_EPSILON) 
        return p2;
    if ((p1-p2).LengthSquared() < tolerance*tolerance) 
        return p2;

    float t2 = t * t;
    float t3 = t2 * t;
    return p1.Lerp(p2, 0.5f * t + 1.5f * t2 - t3);
}

Quaternion SmoothStepAngle(const Quaternion &p1, const Quaternion &p2, float t, float tolerance=0.001f)
{
    if (t < M_EPSILON) 
        return p1;
    if (t > 1.0f - M_EPSILON) 
        return p2;
    if ((p1-p2).LengthSquared() < tolerance*tolerance) 
        return p2;

    float t2 = t * t;
    float t3 = t2 * t;
    return p1.Slerp(p2, 0.5f * t + 1.5f * t2 - t3);
}

#define USE_CATMULLROM_SMOOTH

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(CharacterDemo)

//=============================================================================
//=============================================================================
CharacterDemo::CharacterDemo(Context* context) :
    Sample(context),
    firstPerson_(false),
    drawDebug_(false),
    freeCamera_(false)
{
    // Register factory and attributes for the Character component so it can be created via CreateComponent, and loaded / saved
    Character::RegisterObject(context);
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

    // Create scene subsystem components
    scene_->CreateComponent<Octree>();
    PhysicsWorld *physicsWorld = scene_->CreateComponent<PhysicsWorld>();
    DebugRenderer *dbgRenderer = scene_->CreateComponent<DebugRenderer>();
    physicsWorld->SetDebugRenderer( dbgRenderer );
    physicsWorld->SetGravity(Vector3::ZERO);

    // Create camera and define viewport. We will be doing load / save, so it's convenient to create the camera outside the scene,
    // so that it won't be destroyed and recreated, and we don't have to redefine the viewport on load
    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);
    GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

    // Create static scene content. First create a zone for ambient lighting and fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    Zone* zone = zoneNode->CreateComponent<Zone>();
    zone->SetAmbientColor(Color(0.2f, 0.3f, 0.4f));
    zone->SetFogColor(Color(0.5f, 0.5f, 0.7f));
    zone->SetFogStart(100.0f);
    zone->SetFogEnd(300.0f);
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));

    // Create a directional light with cascaded shadow mapping
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
    Light* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    light->SetSpecularIntensity(0.5f);

    // Create the floor object
    Node* sphWorldNode = scene_->CreateChild("SphereWorld");
    sphWorldNode->SetPosition(Vector3(0.0f, 0.0f, 0.0f));
    sphWorldNode->SetScale(Vector3(50.0f, 50.0f, 50.0f));
    StaticModel* object = sphWorldNode->CreateComponent<StaticModel>();
    object->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
    object->SetMaterial(cache->GetResource<Material>("Platforms/Materials/playgroundMat.xml"));

    RigidBody* body = sphWorldNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(ColLayer_World);
    body->SetCollisionMask(ColMask_World);
    CollisionShape* shape = sphWorldNode->CreateComponent<CollisionShape>();
    shape->SetSphere(1.0f);
}

void CharacterDemo::CreateCharacter()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* objectNode = scene_->CreateChild("Player");
    objectNode->SetPosition(Vector3(0.0f, 26.0f, 0.0f));

    // init char pos/rot
    charPos_ = objectNode->GetPosition();
    charRot_ = objectNode->GetRotation();

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
    shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.9f, 0.0f));

    character_ = objectNode->CreateComponent<Character>();
}

void CharacterDemo::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    instructionText->SetTextAlignment(HA_CENTER);
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void CharacterDemo::SubscribeToEvents()
{
    // Subscribe to Update event for setting the character controls before physics simulation
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));

    // Subscribe to PostUpdate event for updating the camera position after physics simulation
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostUpdate));

    // Unsubscribe the SceneUpdate event from base class as the camera node is being controlled in HandlePostUpdate() in this sample
    UnsubscribeFromEvent(E_SCENEUPDATE);
}

void CharacterDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    Input* input = GetSubsystem<Input>();
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    if (input->GetKeyPress(KEY_F6))
    {
        freeCamera_ =!freeCamera_;

        if (freeCamera_)
        {
            if (character_)
            {
                Node* charNode = character_->GetNode();
                cameraNode_->SetPosition(charNode->GetWorldPosition() + charNode->GetWorldRight() * 15.0f);
                Quaternion qrot(Vector3::FORWARD, -charNode->GetWorldRight());
                yaw_ = qrot.EulerAngles().y_;
                pitch_ = qrot.EulerAngles().x_;
            }
        }
    }
    
    if (freeCamera_)
        return;

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
    using namespace Update;
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    if (drawDebug_)
    {
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
    }

    if (freeCamera_)
    {
        MoveCamera(timeStep);
        return;
    }

    UpdateCamera(timeStep);
}

void CharacterDemo::UpdateCamera(float timeStep)
{
    if (!character_)
        return;

    // interpolate char pos/rot
    const float rotLerpRate = 10.0f;
    const float posLerpRate = 10.0f;
    Node* characterNode = character_->GetNode();
    
    #ifndef USE_CATMULLROM_SMOOTH
    charRot_ = charRot_.Slerp(characterNode->GetWorldRotation(), timeStep * rotLerpRate);
    charPos_ = charPos_.Lerp(characterNode->GetWorldPosition(),  timeStep * posLerpRate);
    #else
    charRot_ = SmoothStepAngle(charRot_, characterNode->GetWorldRotation(), timeStep * rotLerpRate);
    charPos_ = SmoothStep<Vector3>(charPos_, characterNode->GetWorldPosition(), posLerpRate * timeStep);
    #endif

    Quaternion rot = charRot_;
    Quaternion dir = rot * Quaternion(character_->controls_.pitch_, Vector3::RIGHT);

    {
        // Third person camera: position behind the character
        Vector3 aimPoint = charPos_ + rot * Vector3(0.0f, 1.7f, 0.0f);

        // Collide camera ray with static physics objects (layer bitmask 2) to ensure we see the character properly
        Vector3 rayDir = dir * Vector3::BACK;
        float rayDistance = touch_ ? touch_->cameraDistance_ : CAMERA_INITIAL_DIST;
        PhysicsRaycastResult result;
        scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(aimPoint, rayDir), rayDistance, ~ColLayer_Character);
        if (result.body_)
            rayDistance = Min(rayDistance, result.distance_);
        rayDistance = Clamp(rayDistance, CAMERA_MIN_DIST, CAMERA_MAX_DIST);

        cameraNode_->SetPosition(aimPoint + rayDir * rayDistance);
        cameraNode_->SetRotation(dir);
    }
}

void CharacterDemo::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    //if (GetSubsystem<UI>()->GetFocusElement())
    //    return;

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


