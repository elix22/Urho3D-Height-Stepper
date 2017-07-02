# Urho3D Height Stepper
-----------------------------------------------------------------------------------

Description
-----------------------------------------------------------------------------------
Physics object that helps character step over objects in the world.


Screenshots
-----------------------------------------------------------------------------------
![alt tag](https://github.com/Lumak/Urho3D-Height-Stepper/blob/master/screenshot/stepper.jpg)


Configurables:
-----------------------------------------------------------------------------------
* minStepHeight_ - minimum height of an object that you can step over (deault = 0.08).
* maxClimbAngle_ - maximum climb angle (default = 40 degrees).
* applyImpulseToChar_ - whether the stepper should apply impulse to the character when it detects a steppable object (default = true).
* charStepUpDuration_ - the time it takes for character to step up the height, smaller the value, higher the impulse (default = 0.25), applicable if applyImpulseToChar_ is set to true.

To Build
-----------------------------------------------------------------------------------
To build it, unzip/drop the repository into your Urho3D/ folder and build it the same way as you'd build the default Samples that come with Urho3D.

License
-----------------------------------------------------------------------------------
The MIT License (MIT)







