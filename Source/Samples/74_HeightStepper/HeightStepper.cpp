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
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <SDL/SDL_log.h>

#include "HeightStepper.h"
#include "CollisionLayer.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
HeightStepper::HeightStepper(Context* context)
    : LogicComponent(context)
    , minStepHeight_(0.08f)
    , maxStepHeight_(0.30f)
    , maxClimbAngle_(40.0f)
    , minStepNormal_(0.5f)
    , applyImpulseToChar_(true)
    , charStepUpDuration_(0.10f)
    , nodePlacementPos_(Vector3(0.0f, -0.01f, -0.05f))
    , restLookAtPos_(Vector3(0.0f, 2.0f, 1.0f))
    , stepState_(StepState_Idle)
{
    SetUpdateEventMask(0);
}

void HeightStepper::RegisterObject(Context* context)
{
    context->RegisterFactory<HeightStepper>();
}

bool HeightStepper::AddStepper(Node *charNode, Node *footNode, const Vector3& walkerScale, bool useDbgMaterial)
{
    // character info
    charNode_    = charNode;
    footNode_    = footNode;
    prevFootPos_ = footNode_->GetWorldPosition();

    // create
    stepperNode_ = node_->CreateChild("walkerNode");
    stepperNode_->SetScale(walkerScale);

    // reduce the step-up slope by moving the col shape a tad behind char's foot
    const float walkerZBackPosition = 0.48f;
    stepperNode_->SetPosition(Vector3(0.0f, 0.0f, walkerScale.z_ * walkerZBackPosition));

    // body 
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    stepperBody_ = stepperNode_->CreateComponent<RigidBody>();
    stepperBody_->SetCollisionLayerAndMask(ColLayer_Walker, ColMask_Walker);
    stepperBody_->SetTrigger(true);
    stepperBody_->SetKinematic(true);
    stepperBody_->SetMass(1.0f);
    CollisionShape* colShape = stepperNode_->CreateComponent<CollisionShape>();
    // very thin box since it must be a convex shape and can't use convex plane
    colShape->SetBox(Vector3(1.0f, 0.001f, 1.0f));

    // solid
    solidNode_ = node_->CreateChild("solidNode");
    solidNode_->SetScale(walkerScale);
    solidNode_->SetPosition(Vector3(0.0f, 0.0f, walkerScale.z_ * walkerZBackPosition));
    solidBody_ = solidNode_->CreateComponent<RigidBody>();
    solidBody_->SetCollisionLayerAndMask(ColLayer_Static, ColMask_Walker);
    colShape = solidNode_->CreateComponent<CollisionShape>();
    colShape->SetTriangleMesh(cache->GetResource<Model>("Models/Plane.mdl"));

    SubscribeToEvent(stepperNode_, E_NODECOLLISION, URHO3D_HANDLER(HeightStepper, HandleNodeCollision));

    // dbg mat
    if (useDbgMaterial)
    {
        StaticModel *statModel = stepperNode_->CreateComponent<StaticModel>();
        statModel->SetModel(cache->GetResource<Model>("Models/Plane.mdl"));
        statModel->SetMaterial(cache->GetResource<Material>("HeightStepper/Scene/Materials/grnMat.xml"));

        statModel = solidNode_->CreateComponent<StaticModel>();
        statModel->SetModel(cache->GetResource<Model>("Models/Plane.mdl"));
        statModel->SetMaterial(cache->GetResource<Material>("HeightStepper/Scene/Materials/blueMat.xml"));
    }

    // walker active
    SetSolid(false);
    SetUpdateEventMask(USE_FIXEDUPDATE);

    return true;
}

void HeightStepper::Start()
{
}

void HeightStepper::SetSolid(bool solid)
{
    if (solid)
    {
        solidNode_->SetEnabled(true);
        stepperNode_->SetEnabled(false);
    }
    else
    {
        solidNode_->SetEnabled(false);
        stepperNode_->SetEnabled(true);
    }
}

bool HeightStepper::IsSolid() const
{
    return solidNode_->IsEnabled();
}

void HeightStepper::ToggleStepper(bool enable)
{
    if (enable)
    {
        stepperNode_->SetEnabled(true);
        solidNode_->SetEnabled(false);
        stepState_ = StepState_Idle;
    }
    else
    {
        stepperNode_->SetEnabled(false);
        solidNode_->SetEnabled(false);
        stepState_ = StepState_Idle;
    }
}

void HeightStepper::ToggleDbgTexture(bool enable)
{
    PODVector<StaticModel*> mdlList;
    node_->GetComponents<StaticModel>(mdlList, true);

    for ( unsigned i = 0; i < mdlList.Size(); ++i )
    {
        mdlList[i]->SetEnabled(enable);
    }
}

void HeightStepper::FixedUpdate(float timeStep)
{
    // char data
    Vector3 charPos = charNode_->GetWorldPosition();
    Vector3 charDir = charNode_->GetWorldDirection();
    Quaternion charRot = charNode_->GetWorldRotation();
    Vector3 footPos = footNode_->GetWorldPosition();
    Vector3 footTravelDir = footPos - prevFootPos_;

    // set cur look at pos
    Vector3 curLookPos = footPos;
    Vector3 segLToFoot = curLookPos - node_->GetWorldPosition();
    prevFootPos_ = footPos;

    // foot status
    const float minFootFrontIndicator = 0.05f;
    const float minFootMovingFwdIndicator = 0.01f;
    const float minCharStepOffdist = 0.4f;

    footInfront_ = (segLToFoot.DotProduct(charDir) > minFootFrontIndicator);
    bool movingfwd= (footTravelDir.DotProduct(charDir) > minFootMovingFwdIndicator);
    bool validCondition = footInfront_ && movingfwd;

    // don't move the node until the char has stepped over it
    if (stepState_ == StepState_Solid)
    {
        float distToChar = (charPos - node_->GetWorldPosition()).Length();

        if (distToChar > minCharStepOffdist)
        {
            SetSolid(false);

            stepState_ = StepState_Idle;
        }
        else
        {
            if (!footInfront_)
            {
                stepState_ = StepState_WaitFwd;
            }

            return;
        }
    }
    else if (stepState_ == StepState_WaitFwd)
    {
        if (!footInfront_)
        {
            return;
        }
        else
        {
            SetSolid(false);

            stepState_ = StepState_Idle;
        }
    }

    // don't move the node until the char has walked over it
    node_->SetWorldPosition(charPos + charRot * nodePlacementPos_);

    if (!validCondition)
    {
        // rest position - look up and forward a bit to stand the collision shape some what upright
        node_->LookAt(charPos + charRot * restLookAtPos_);
    }
    else
    {
        node_->LookAt(curLookPos);
    }
}

void HeightStepper::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    // skip if not in the right condition
    if (stepState_ != StepState_Idle || !footInfront_)
    {
        return;
    }

    MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());
    Vector3 nodePos = node_->GetWorldPosition();
    Vector3 nodeAngle = node_->GetWorldRotation().EulerAngles();

    // angle doesn't change while in this function
    if (Abs(nodeAngle.x_) > maxClimbAngle_)
    {
        return;
    }

    while (!contacts.IsEof())
    {
        Vector3 contactPosition = contacts.ReadVector3();
        Vector3 contactNormal = contacts.ReadVector3();
        /*float contactDistance = */contacts.ReadFloat();
        /*float contactImpulse = */contacts.ReadFloat();

        // validate
        Vector3 segL = contactPosition - nodePos;
        float stepHeight = Vector3::UP.DotProduct(segL);

        // this avoids garbage data below char's foot
        if (stepHeight < minStepHeight_ || stepHeight > maxStepHeight_)
        {
            continue;
        }

        // skip unsteppable contact normals
        if (Vector3::UP.DotProduct(contactNormal) < minStepNormal_)
        {
            continue;
        }

        // apply impulse to char if configured
        if (applyImpulseToChar_)
        {
            charNode_->GetComponent<RigidBody>(true)->ApplyImpulse(Vector3::UP * stepHeight / charStepUpDuration_);
        }

        SetSolid(true);

        stepState_ = StepState_Solid;

        break;
    }
}


