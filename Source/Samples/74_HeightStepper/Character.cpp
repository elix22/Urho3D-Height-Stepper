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

#include "Character.h"

//=============================================================================
//=============================================================================
#define MAX_STEPDOWN_HEIGHT     0.7f

//=============================================================================
//=============================================================================
Character::Character(Context* context) :
    LogicComponent(context),
    onGround_(false),
    okToJump_(true),
    inAirTimer_(0.0f),
    jumpStarted_(false)
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

void Character::DelayedStart()
{
    animCtrl_ = node_->GetComponent<AnimationController>(true);
    animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_JumpLoop.ani", 0, true, 0.3f);
}

void Character::Start()
{
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
    const Quaternion& rot = node_->GetRotation();
    Vector3 moveDir = Vector3::ZERO;
    const Vector3& velocity = body->GetLinearVelocity();
    // Velocity on the XZ plane
    Vector3 planeVelocity(velocity.x_, 0.0f, velocity.z_);

    if (controls_.IsDown(CTRL_FORWARD))
        moveDir += Vector3::FORWARD;
    if (controls_.IsDown(CTRL_BACK))
        moveDir += Vector3::BACK;
    if (controls_.IsDown(CTRL_LEFT))
        moveDir += Vector3::LEFT;
    if (controls_.IsDown(CTRL_RIGHT))
        moveDir += Vector3::RIGHT;

    // Normalize move vector so that diagonal strafing is not faster
    if (moveDir.LengthSquared() > 0.0f)
        moveDir.Normalize();

    // If in air, allow control, but slower than when on ground
    body->ApplyImpulse(rot * moveDir * (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));

    if (softGrounded)
    {
        // When on ground, apply a braking force to limit maximum ground velocity
        Vector3 brakeForce = -planeVelocity * BRAKE_FORCE;
        body->ApplyImpulse(brakeForce);

        // Jump. Must release jump control between jumps
        if (controls_.IsDown(CTRL_JUMP))
        {
            if (okToJump_)
            {
                body->ApplyImpulse(Vector3::UP * JUMP_FORCE);
                jumpStarted_ = true;
                okToJump_ = false;
                animCtrl_->StopLayer(0);
                animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_JumpStart.ani", 0, false, 0.2f);
                animCtrl_->SetTime("HeightStepper/Girlbot/Girlbot_JumpStart.ani", 0);
            }
        }
        else
            okToJump_ = true;
    }

    if (!onGround_ || jumpStarted_)
    {
        if (jumpStarted_)
        {
            if (animCtrl_->IsAtEnd("HeightStepper/Girlbot/Girlbot_JumpStart.ani"))
            {
                animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_JumpLoop.ani", 0, true, 0.3f);
                animCtrl_->SetTime("HeightStepper/Girlbot/Girlbot_JumpLoop.ani", 0);
                jumpStarted_ = false;
            }
        }
        else
        {
            float rayDistance = 50.0f;
            PhysicsRaycastResult result;
            GetScene()->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(node_->GetPosition(), Vector3::DOWN), rayDistance, 0xff);

            if (result.body_ && result.distance_ > MAX_STEPDOWN_HEIGHT )
            {
                animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_JumpLoop.ani", 0, true, 0.2f);
            }
            else if (result.body_ == NULL)
            {
                // fall to death animation
            }
        }
    }
    else
    {
        // Play walk animation if moving on ground, otherwise fade it out
        if (softGrounded && !moveDir.Equals(Vector3::ZERO))
        {
            animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_Run.ani", 0, true, 0.2f);
        }
        else
            animCtrl_->PlayExclusive("HeightStepper/Girlbot/Girlbot_Idle.ani", 0, true, 0.2f);

        // Set walk animation speed proportional to velocity
        //float spd = Clamp(planeVelocity.Length() * 0.3f, 1.0f, 2.0f);
        //animCtrl_->SetSpeed("HeightStepper/Girlbot/Girlbot_Run.ani", spd);
    }

    // Reset grounded flag for next frame
    onGround_ = false;
}

void Character::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    // Check collision contacts and see if character is standing on ground (look for a contact that has near vertical normal)
    using namespace NodeCollision;

    MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());

    while (!contacts.IsEof())
    {
        Vector3 contactPosition = contacts.ReadVector3();
        Vector3 contactNormal = contacts.ReadVector3();
        /*float contactDistance = */contacts.ReadFloat();
        /*float contactImpulse = */contacts.ReadFloat();

        // If contact is below node center and pointing up, assume it's a ground contact
        if (contactPosition.y_ < (node_->GetPosition().y_ + 1.0f))
        {
            float level = contactNormal.y_;
            if (level > 0.75)
                onGround_ = true;
        }
    }
}
