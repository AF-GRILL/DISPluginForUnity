# Changelog

## Beta 0.11.0

- Add ability to have wildcards for DIS enumerations.

## Beta 0.10.1

- Update PDUProcessor to go through all received packets if multiple received in a frame.

## Beta 0.10.0

- Fix ground clamping issues and add UnityEvent for it in DISReceiveComponent.
- Add error catching in UDPReceiverMulti.
- Fix issues with geospatial conversions in GeoreferenceSystem.
- Fix orienting entities properly when far away from origin in Conversions.

## Beta 0.9.0

- Add Detonation PDU handle for NO SPECIFIC ENTITY in DIS Game Manager
- Add catch for failed PDU unmarshalling in PDU Processor
- Add multicast TTL setting in PDU Sender
- Update entity state calculations every frame in DIS Send Component
- Combine lat/lon parameters in multiple Conversions functions to be FLatLonAlt struct
- Fix issues in FPB and RPB Dead Reckoning body acceleration calculations
- Fix issues with Quaternion dead reckoning threshold calculations

## Beta 0.4.1 -- Initial Unity version

- Made DIS Enumeration Mappings a scriptable object for easy reuse.
- Bug fixes to the DIS Sending component, Dead Reckoning, and unwinding rotations in the Conversions library.
- Updated LLA/ECEF conversions in Conversions library to more accurate math.

## Beta 0.4.0

- Added Floating Point error check to fix float values outside of ACos Range.
- Edited DISManager Prefab for more general defaults
- Added Unity Event for when a new DIS Entity is created.
- Added disconnect method to PDUReceiver.

## Beta 0.3.0

- Finished implementing the DISSendComponent
- Renamed DISEnums.cs to DISEnumsAndStructs.cs as structs for geospatial variables were all added in. 
- Added in rotational conversions to GeoreferenceSystem for geospatial to/from Unity. 
- In DISSendComponent, updated Angular Velocity calculations to utilize quaternions rather than euler angles. 
- In DISSendComponent, moved ESPDU variable calculations for linear velocity, linear acceleration, and angular velocity to a timer to give user more flexibility over frequency of them. 
- Fixed small math errors in Conversions library where ACos functions were missing where needed.

## Beta 0.2.0

- Added more conversions to the Conversions library
- Added Beginnings of GeoreferenceSystem
- Updated DISSendComponent's calculation of body acceleration. Also moved one of its functions to the Conversions library where it is more appropriate.
- Reworked parameters on one of the DeadReckoningLibrary functions.
- Updated comments in Conversions library and removed unneeded debug print statement
- Removed DIS Entity Base.
- Separated DIS component into a Send and Receive component. The DISSendComponent is not fully finished.

## Beta 0.1.0

- Implemented OpenDIS C# libraries.
- Added a UDP Sender and receiver.
- Added in a PDU processor.
- Added in a DISComponent to handle received DIS packets for associated entities.
- Added in a DISGameManager to house DIS enumeration to Game Object mappings.