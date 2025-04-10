[![DISPluginLogo](Resources/Icon128.png)](https://www.af-grill.com/)

# Getting Started

- This plugin was made for Unity 2020.3.26f1
- Clone this repository
- Put the unpacked plugin folder in the 'Assets' folder for the desired project.
- Launch the project and allow Unity to build the newly added files.

# Required Game Objects

- Certain game objects are required to be in a given scene in order for the plugin to work as desired. These are listed below:
	- _**NOTE:**_ Only one of each unique game object type listed below should be in a single level.

![DISManager](Resources/ReadMeImages/DISManager.png)

- The DIS Manager game object is required in the level and is built into the GRILL DIS for Unity plugin.
	- This game object stores the DIS Enumeration to class mappings and other various DIS information.
	- _**More information about scripts contained on the game object itself can be found in the sections below.**_

# Implemented DIS PDUs

The GRILL DIS for Unity plugin currently supports the below DIS PDUs:
- Designator
- Detonation
- Electromagnetic Emissions
- Entity State
- Entity State Update
- Fire
- Remove Entity
- Signal
- Start Resume
- Stop Freeze

If additional PDU support is desired, follow the pattern that is done in the PDUReceiver script's "ProcessDISPacket" function.

# Setting Up an Empty Project

If creating a new project using the GRILL DIS for Unity plugin, the following steps can be performed to set it up with DIS capabilities.
_**Additional info for all of these topics can be found in their respective sections below.**_

**Receiving DIS**
1. Place a single DISManager game obect into a scene.
	- The DISManager comes with the plugin and is located in the "GRILL_DIS/DISPluginContent" folder.
2. On the DISManager game object, alter the network parameters of the PDUReceiver according to the needs of the project.
	- This script is utilized to handle incoming DIS network packets.
3. Make custom Unity game objects for all DIS entities the project is expected to receive and handle.
	- Attach a DIS Receive Component script to all of the game objects that are going to be DIS entities and set the parameters in the script accordingly.
		- This script contains events for handling incoming DIS information.
4. Create a custom DIS Enumeration Mapping asset that maps DIS entity types to the custom Unity game objects made in step 3.
5. On the DISManager game object, set the "DIS Enumeration Mapping" variable on its associated DIS Game Manager script to the DIS Enumeration Mapping asset made in step 4.
6. On the DISManager game object, set the "Origin LLA" variable on its associated Georeference System script to the Latitude, Longitude, and altitude the Unity origin is desired to be at.

**Sending DIS**
1. Place a single DISManager game obect into a scene.
2. On the DISManager game object, alter the network parameters of the PDUSender according to the needs of the project.
	- This script is utilized to emit DIS network packets
3. Make custom Unity game objects for all DIS entities the project needs to emit.
	- Attach a DIS Send Component script to all of the game objects that are going to be DIS entities and set the parameters in the script accordingly.
		- The DIS Send Component script will emit DIS PDUs using the PDUSender script on the DISManager game object.
4. On the DISManager game object, set the "Origin LLA" variable on its associated Georeference System script to the Latitude, Longitude, and altitude the Unity origin is desired to be at.

**General Flow** 
- The DIS plugin maintains a mapping within the DIS Game Manager script on the DISManager game object that maps DIS Entity IDs to Unity game objects that exist in the current scene.
- When an EntityStatePDU is received, it gets parsed and info it contains is utilized to check the mapping.
	- If a match is found:
		- A game object for the EntityStatePDU already exists in the scene and is notified about the new information.
	- If a match is not found:
		- The DISManager looks in the DIS Enumeration Mapping asset on its DIS Game Manager script for what game object should represent this new entity.
			- If an entity type is found in the DIS Enumeration Mapping asset that matches the entity type contained in the EntityStatePDU, the corresponding game object is spawned.
			- Otherwise nothing is done.

# PDU Receiver Script

- The PDU Receiver script is what is used to handle receipt of UDP packets.
- Delegates received PDUs to appropriate DIS entity game object.
- Notable functions:
    - Start UDP Receiver

![PDUReceiverScript](Resources/ReadMeImages/PDUReceiverScript.png)

The PDU Receiver script has the following settings:
- **Auto Connect at Start**
	- Whether or not the UDP socket for receiving DIS packets should be auto connected.
- **Use Multicast**
	- Whether or not this socket will be receiving Multicast connections.
- **Allow Loopback**
	- Whether or not to receive packets that originate from our local IP.
- **IP Address**
	- The IP address to receive DIS packets on. Will only be used if Use Multicast is disabled.
- **Multicast Address**
	- The Multicast address to receive DIS packets on. Will only be used if Use Multicast is enabled.
- **Port**
	- The port to receive DIS packets on.
	
# PDU Sender Script

- The PDU Sender script is what is used to handle sending of UDP packets.
- Delegates received PDUs to appropriate DIS entity game object.
- Notable functions:
    - Init
		- Starts the sender
	- Stop
	- Change Address
		- Safely updates the IP and Port of the sender
	- Contains send methods for all OpenDIS supported PDU types

![PDUSenderScript](Resources/ReadMeImages/PDUSenderScript.png)

The PDU Sender script has the following settings:
- **Connection Type**
	- The type of connection to open for sending.
	- Options are:
		- Unicast
		- Broadcast
		- Multicast
- **Auto-Connect**
	- Whether or not the UDP socket for sending DIS packets should be auto connected.
- **IP Address**
	- The IP address to receive DIS packets on. Will be disabled if Connection Type is set to Broadcast. Should be a Multicast address if the socket connection type is set to Multicast.
	- _**NOTE:**_ If Connection Type is set to Broadcast, outgoing DIS packets will be broadcasted on 255.255.255.255.
- **Port**
	- The port to send DIS packets on.
- **Max Queue Size**
	- The maximum number of queued outgoing PDUs before the most recent ones get removed.

# DIS Game Manager

- The DIS Game Manager is responsible for creating/removing DIS entities as packets are processed by the PDU Receiver script. It also informs the appropriate DIS Entities when DIS packets are received that impact them. This is done through notifying their associated DIS Component script.
- The DIS Game Manager has the following settings:
    - **Exercise ID**: The exercise ID of the DIS sim this project will be associated with.
    - **Site ID**: The site ID of this DIS sim.
    - **Application ID**: The application ID of this DIS sim.
	- **DIS Enumeration Mapping**: Contains desired game object to DIS Enumeration mappings.
        - _**NOTE:**_ Although not required, for proper DIS delegation each game object in the Entity Types mapping should have a DIS Receive Component script attached to it.
	- **DIS Entity Parent Container**: The parent that all of the DIS Entities spawned by the plugin should be placed under.

![DISGameManagerScript](Resources/ReadMeImages/DISGameManagerScript.png)

- The DIS Game Manager contains:
    - Listing of DIS Entities and their associated enumeration. This is loaded from the DIS Enumeration Mapping variable.
    - Listing of Entity IDs and their active DIS Entities in the world. This is a living list that is added to/removed from as new packets are received.
    - The DIS exercise, site, and application IDs.
- Notable functions:
	- Spawn or Get Game Object from Entity State PDU
    - Add DIS Entity to Map
    - Remove DIS Entity from Map

# DIS Receive Component Script

- The DIS Receive Component script is responsible for handling all receive DIS functionality and DIS PDU updates for its associated DIS Entity.
- Handles dead reckoning and ground clamping updates.
- Contains various DIS related variables.
- Notable functions:
    - Ground Clamping
		- Default implemented behavior line traces toward the earth using NED vectors. Places the entity on the hit location on the ground.
        - Can be overriden for a custom implementation.
- Contains event bindings for:
    - Receiving each type of DIS Entity PDU currently implemented.
    - Dead reckoning update

![DISReceiveComponent](Resources/ReadMeImages/DISReceiveComponentScript.png)
	
- Has variables for:
    - Most Recent Entity State PDU
    - Dead Reckoning Entity State PDU
        - This is an Entity State PDU whose information has been updated with the most recent Dead Reckoning information.
    - Latest Entity State PDU Timestamp
    - Spawned From Network
        - Whether or not this entity was spawned from the network.
    - Entity Type
		- This record specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.
        - This value gets set when an Entity State PDU or Entity State Update PDU is received for the associated entity.
    - Entity ID
		- This record specifies the site ID, application ID, and entity ID fields. They combine to form a unique identifier of the entity in the exercise.
        - This value gets set when an Entity State PDU or Entity State Update PDU is received for the associated entity.
    - Entity Force ID
		- This field distinguishes the different teams or sides in a DIS exercise.
        - This value gets set when an Entity State PDU or Entity State Update PDU is received for the associated entity.
    - Entity Marking
		- This record is used to specify the friendly name of the entity to be interpreted for display.
        - This value gets set when an Entity State PDU or Entity State Update PDU is received for the associated entity.
    - DIS Timeout
		- How long to wait in seconds after an Entity State PDU is received before deleting. Gets refreshed after an Entity State PDU is received.
	- DIS Culling Mode
		- Culls DIS packets based on settings
			- Options:
				- None
				- Cull Dead Reckoning
					- Cull Dead Reckoning updates. Distance updates get culled at is dictated by the 'DIS Culling Distance' variable.
				- Cull All
                    - Currently only culls Dead Reckoning updates.
	- DIS Culling Distance
		- The distance away from the camera that entities will start to have DIS packets culled.
    - Perform Dead Reckoning
        - Whether or not dead reckoning should be performed.
	- Perform Dead Reckoning Smoothing
		- Whether or not to smooth location/rotation changes after receiving a new Entity State update. Prevents entity teleporting.
	- Dead Reckoning Smoothing Period Seconds
		- Time in seconds that smoothing should take place.
    - Perform Ground Clamping
        - Performs ground clamping based on settings
			- Options:
				- None
				- Ground Clamp with DIS Options
					- Perform ground clamping. Ignore ground clamping for munitions and non-ground entity types.
				- Always Ground Clamp
					- Always perform ground clamping regardless of entity type.
    - Ground Clamping Collision Channel
        - The collision channel that should be used for ground clamping.

# DIS Send Component Script

- The DIS Send Component handles basic sending DIS functionality its associated DIS Entity.
- Contains various DIS related variables.
- Notable functions:
	- Form Entity State PDU
		- Uses all known information to form an ESPDU for the associated DIS Entity.
    - Send Entity State PDU
		- Default implemented behavior tries to send out an Entity State or Entity State Update PDU based on Entity State PDU Sending Mode variable.
		- Called on update as thresholds need consistently checked.
    - Set Entity Appearance
		- Used to update the entity appearance during runtime.
    - Set Entity Capabilities
		- Used to update the entity capabilities during runtime.
	- Set Dead Reckoning Algorithm
		- Used to update the dead reckoning algorithm during runtime.
	- CalculateECEFLinearVelocityAndAcceleration
		- Calculates the Linear Velocity and Linear Acceleration of the entity that this component is attached to. Calculates them in terms of ECEF world coordinates.
	- CalculateBodyLinearVelocityAndAcceleration
		- Calculates the Linear Velocity and Linear Acceleration of the entity that this component is attached to. Calculates them in terms of entity body coordinates.
	- CalculateAngularVelocity
		- Calculates the angular velocity for the entity that the component is attached to. Angular velocity is in terms of radians per second.
	
![DISSendComponent](Resources/ReadMeImages/DISSendComponentScript.png)

- Has variables for:
    - Most Recent Entity State PDU
    - Dead Reckoning Entity State PDU
        - This is an Entity State PDU whose information has been updated with the most recent Dead Reckoning information.
    - Entity Type
		- This record specifies the kind of entity, the country of design, the domain, the specific identification of the entity, and any extra information necessary for describing the entity.
        - This value should be set on the component and will be used when sending automatic PDU updates.
        - _**NOTE**_: The checkbox to the left of the Entity Type fields has no functionality in the DIS Send component outside of being able to edit the variable. This checkbox is used for Entity Type wildcards and only has functionality when receiving Entity Type info.
    - Current Entity ID
		- This record specifies the ID of the associated entity fields. When sending, this can be combined with the Site ID and Application ID stored in the DIS Game Manager to form the unique identifier of the entity in the exercise.
        - This value should be set on the component and will be used when sending automatic PDU updates.
    - Entity Force ID
		- This field distinguishes the different teams or sides in a DIS exercise.
        - This value should be set on the component and will be used when sending automatic PDU updates.
    - Entity Marking
		- This record is used to specify the friendly name of the entity to be interpreted for display.
        - This value should be set on the component and will be used when sending automatic PDU updates.
    - DIS Heartbeat Seconds
		- How often a new PDU update should be sent out.
		- Utilized if Dead Reckoning Thresholds are not clipped.
	- Entity State PDU Sending Mode
		- Mode that the send component should be in.
			- Options:
				- None
					- Don't send any automatic Entity State or Entity State Update PDU updates.
				- Entity State PDU
					- Automatically send out Entity State PDU updates.
					- Will send out a new PDU when a Dead Reckoning Threshold is clipped, the DIS heartbeat expires, when the Dead Reckoning algoritm is changed, when the entity Capabilities are changed, when the entity Appearance is changed, or when the entity expires in the world.
				- Entity State Update PDU
					- Automatically send out Entity State Update PDU updates.
					- Will send out a new PDU when a Dead Reckoning Threshold is clipped, the DIS heartbeat expires, when the entity Appearance is changed, or when the entity expires in the world.
	- Entity State Calculation Rate
		- The rate at which the Entity State parameters (angular velocity, linear velocity, linear acceleration) should be calculated. The more often, the more accurate sent Entity State PDUs will be.
    - Entity Appearance
        - Represented as an int-32 field. Specifies the dynamic changes to the entities appearance attributes.
        - Refer to DIS Standard IEEE 1278.1 document for a breakdown.
    - Entity Capabilities
        - Represented as an int-32 field. A collection of fields which describe the capabilities of the Entity.
        - Refer to DIS Standard IEEE 1278.1 document for a breakdown.
	- Dead Reckoning Algorithm
		- The dead reckoning algorithm to use.
    - Dead Reckoning Position Threshold Meters
        - The position threshold in meters to use for dead reckoning. If the dead reckoning position deviates more than this value away from the actual position in any axis, a new Entity State PDU will be sent.
    - Dead Reckoning Orientation Threshold Degrees
        - The orientation threshold in degrees to use for dead reckoning. If the dead reckoning orientation deviates more than this value away from the actual orientation, a new Entity State PDU will be sent.

# Georeference System

![GeoreferenceSystem](Resources/ReadMeImages/GeoreferenceSystemScript.png)

- The Georeference System script maps the Unity origin (0, 0, 0) to a specified Latitude, Longitude, and Altitude.
- This information is utilized to convert:
	- ECEF and LLA to Unity coordinates.
	- Unity coordinates to ECEF and LLA.
- Has variables for:
	- Earth Shape
		- Whether the shape of the world in the project is to be a Round Earth or Flat Earth representation.
		- Origin LLA
			- The Latitude in decimal degrees, Longitude in decimal degrees, and Altitude in meters that the Unity Origin represents.
- Notable functions:
	- ECEFToUnity
		- Converts ECEF coordinates to Unity coordinates.
	- UnityToECEF
		- Converts Unity coordinates into ECEF coordinates.
	- LatLonAltToUnity
		- Converts the given Lat, Lon, Alt coordinates to Unity coordinates.
	- UnityToLatLonAlt
		- Converts Unity coordinates into geodetic Lat, Lon, Alt coordinates.
	- LatLonAltToProjected
		- Converts the given Lat and Lon to UTM coordinates.
	- GetNEDVectorsAtEngineLocation
		- Get the North, East, Down vectors at the given Unity location.
	- GetNEDVectorsAtECEFLocation
		- Get the North, East, Down vectors at the given ECEF location.
	- GetENUVectorsAtEngineLocation
		- Gets the East, North, Up vectors at the given Unity location.
	- GetENUVectorsAtECEFLocation
		- Gets the East, North, Up vectors at the given ECEF location.

# DIS Enumeration Mappings

![DISEnumMappingsUAsset](Resources/ReadMeImages/DISEnumerationMappings.png)

- The DIS Enumeration Mapping asset allows for mappings between DIS Enumerations and Unity game objects to be made.
- To create a new DIS Enumeration Mapping asset:
    - Click on the 'Assets' tab
	- Hover 'Create'
	- Hover 'GRILL DIS'
	- Select the 'DIS Enumeration Mapping' asset option.
- Once created, mappings can be created by opening up the created asset.
- The settings it contains are:
    - Entity Type Mappings
        - Mapping of all available Game Object to DIS Enumeration mappings.
    - Friendly Name
        - Friendly name of this mapping for easy user lookup
    - Game Object
        - The Unity game object that this mapping should point to.
        - _**NOTE:**_ Although not required, for proper DIS delegation each game object used here should have a DIS Receive Component script attached to it.
    - Entity Types
        - All desired DIS Enumerations that should point to this game object.
        - _**NOTE**_: The checkbox to the left of the Entity Type fields represents wildcards.
            - If the checkbox is enabled, the specific value given will be referenced.
            - If the checkbox is disabled, the value will be treated as a wildcard.
            - Ex: The below image would use wildcards for the Specific and Extra fields.
        - _**NOTE**_: If duplicate enumerations are found across multiple entities, an appropriate message is logged and the most recent encountered game object to enumeration mapping is used.
        - _**NOTE**_: If a DIS Enumeration is received on the network and no mapping exists for it, an appropriate message is logged and the packet is ignored.
        - For a breakdown of the individual elements of a DIS Enumeration, refer to the [Naval Postgraduate School's Documentation](http://faculty.nps.edu/brutzman/vrtp/mil/navy/nps/disenumerations/jdbehtmlfiles/pdu/28.htm#:~:text=Description%3A%20The%20type%20of%20entity,necessary%20for%20describing%20the%20entity.).

# DIS Conversion Library

- Contains static functions for converting between various geospatial coordinates.

![ConversionsFunctions](Resources/ReadMeImages/ConversionsFunctions.png)

# Dead Reckoning Library

- Contains functions for performing Dead Reckoning

![DeadReckoningLibrary](Resources/ReadMeImages/DeadReckoningLibrary.png)