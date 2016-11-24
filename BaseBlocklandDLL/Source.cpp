#include "stdafx.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Engine function declarations

//Vehicle::resolveCollision(Rigid& collisionObject, CollisionList& collisionList)  
BLFUNC(bool, __thiscall, ResolveCollisionVehicle, Vehicle* obj, Rigid* rigid, CollisionList* collisionList);
MologieDetours::Detour<ResolveCollisionVehicleFn>* Detour_ResolveCollisionVehicle;

BLFUNC(bool, __thiscall, ResolveContactsVehicle, Vehicle* obj, Rigid* rigid, CollisionList* collisionList, F32 dt);
MologieDetours::Detour<ResolveContactsVehicleFn>* Detour_ResolveContactsVehicle;

BLFUNC(bool, __thiscall, QueueCollision, ShapeBase* thisObj, SceneObject* obj, Point3F* vec);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global variables

int* PlayerMoveMask;
int* PlayerCollisionMaskServer;
int* PlayerCollisionMaskClient;

int* VehicleCollisionMask;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions

// NOTE:
// Hooked functions can't access the vehicle data yet.
// We're assuming contactTolerence is always 0.3,
// and collisionTolerence is always 0.25

#define COLTOL 0.1f
#define CONTOL 0.1f

// Hook
bool __fastcall Hooked_ResolveCollisionVehicle(Vehicle* obj, void* nothing, Rigid* ns, CollisionList* cList)
{
	// Apply impulses to resolve collision  
	bool colliding, collided = false;

	int iterations = 0;

	do
	{
		colliding = false;
		for (S32 i = 0; i < cList->count; i++)
		{
			Collision& c = cList->collision[i];

			// TODO: We're getting collisions against ourselves here because  
			// Convex::updateWorkingList() doesn't filter them.  Is there any  
			// good reason why it doesn't skip over these to save performance?  
			if (c.object == obj) {
				continue;
			}

			if (c.distance > CONTOL) {
				continue;
			}

			// Velocity into surface  
			Point3F v, r;
			ns->getOriginVector(c.point, &r);
			ns->getVelocity(r, &v);
			F32 vn = mDot(v, c.normal);

			// Only interested in velocities greater than sContactTol,  
			// velocities less than that will be dealt with as contacts  
			// "constraints".  
			if (mFabs(vn) < CONTOL) {
				continue;
			}

			// Apply impulses to the rigid body to keep it from  
			// penetrating the surface.
			if (c.object->getTypeMask() & VehicleObjectType)
			{
				Vehicle* other = static_cast<Vehicle*>(c.object);
				colliding |= ns->resolveCollision(c.point, c.normal, &other->mRigid);
			}
			else
				colliding |= ns->resolveCollision(c.point, c.normal);

			// This keeps track if we had any collisions during  
			// any of the iteration loops.  
			collided |= colliding;

			// Keep track of objects we collide with  
			if (!obj->isGhost() && c.object->getTypeMask() & ShapeBaseObjectType)
			{
				ShapeBase* col = static_cast<ShapeBase*>(c.object);

				Point3F calcV;

				calcV.x = v.x - col->getVelocity().x;
				calcV.y = v.y - col->getVelocity().y;
				calcV.z = v.z - col->getVelocity().z;

				//QueueCollision(obj, c.object, &calcV);
			}
		}

		iterations++;

		if (iterations > 2000) {
			// fuck it, this one ain't getting resolved any time soon
			break;
		}
	} while (colliding);

	return Detour_ResolveCollisionVehicle->GetOriginalFunction()(obj, ns, cList);
}

// removed - resolveContacts works fine, there's no reason for us to detour it.

// Helpers
static void setPlayerCollidesWith(DWORD* obj, int argc, const char** argv)
{
	Printf("ColMod | Modified player collision mask");

	if (!_stricmp(argv[2], "true") || !_stricmp(argv[2], "1") || (0 != atoi(argv[2])))
	{
		*PlayerMoveMask |= atoi(argv[1]);
		*PlayerCollisionMaskServer |= atoi(argv[1]);
		*PlayerCollisionMaskClient |= atoi(argv[1]);
	}
	else {
		*PlayerMoveMask &= ~atoi(argv[1]);
		*PlayerCollisionMaskServer &= ~atoi(argv[1]);
		*PlayerCollisionMaskClient &= ~atoi(argv[1]);
	}

	//Tell clients with the dll about the change
	Eval("transmitPlayerCollisionInfo();");
}

static bool getPlayerCollidesWith(DWORD* obj, int argc, const char** argv)
{
	if (*PlayerCollisionMaskServer & atoi(argv[1])) {
		return true;
	}
	else {
		return false;
	}
}

static void setVehicleCollidesWith(DWORD* obj, int argc, const char** argv)
{
	Printf("ColMod | Modified vehicle collision mask");

	if (!_stricmp(argv[2], "true") || !_stricmp(argv[2], "1") || (0 != atoi(argv[2])))
	{
		*VehicleCollisionMask |= atoi(argv[1]);
	}
	else {
		*VehicleCollisionMask &= ~atoi(argv[1]);
	}
}

static bool getVehicleCollidesWith(DWORD* obj, int argc, const char** argv)
{
	if (*VehicleCollisionMask & atoi(argv[1])) {
		return true;
	}
	else {
		return false;
	}
}

//Setup our stuff
DWORD WINAPI Init(LPVOID args)
{
	if (!InitTorqueStuff())
		return 0;

	//Find functions for vehicle collision
	BLSCAN(ResolveCollisionVehicle, "\x83\xEC\x40\x8B\x44\x24\x48\x8B\x00", "xxxxxxxxx");
	BLSCAN(ResolveContactsVehicle, "\x83\xEC\x54\x0F\x57\xC0\x53\x55\x8B\x6C\x24\x60", "xxxxxxxxxxxx");

	BLSCAN(QueueCollision, "\x53\x55\x56\x57\x8B\xF1\xE8\x00\x00\x00\x00\x8B\x5C\x24\x14\x8B\xF8\x8B\x43\x20", "xxxxxxx????xxxxxxxxx");

	//Find collision masks
	PlayerMoveMask = (int*)0x71A9B0;
	PlayerCollisionMaskServer = (int*)0x71AA6C;
	PlayerCollisionMaskClient = (int*)0x71AA70;
	VehicleCollisionMask = (int*)0x71D3E4;

	//Register console functions
	ConsoleFunction(NULL, "setPlayerCollidesWith", setPlayerCollidesWith,
		"setPlayerCollidesWith(int typemask, bool collides) - Set whether players should collide with a specific typemask.", 3, 3);

	ConsoleFunction(NULL, "getPlayerCollidesWith", getPlayerCollidesWith,
		"getPlayerCollidesWith(int typemask) - Check if players collide with a specific typemask.", 2, 2);

	ConsoleFunction(NULL, "setVehicleCollidesWith", setVehicleCollidesWith,
		"setVehicleCollidesWith(int typemask, bool enabled) - Set whether vehicles should collide with a specific typemask.", 3, 3);

	ConsoleFunction(NULL, "getVehicleCollidesWith", getVehicleCollidesWith,
		"getVehicleCollidesWith(int typemask) - Check if vehicles collide with a specific typemask.", 2, 2);

	ConsoleVariable("$Physics::PlayerMoveMask", PlayerMoveMask);
	ConsoleVariable("$Physics::PlayerContactMaskServer", PlayerCollisionMaskServer);
	ConsoleVariable("$Physics::PlayerContactMaskClient", PlayerCollisionMaskClient);

	ConsoleVariable("$Physics::VehicleCollisionMask", VehicleCollisionMask);

	//Execute some torquescript to keep client and server with this mod in sync
	Eval("function serverCmdRequestPlayerCollisionInfo(%client){commandToClient(%client, 'TransmitPlayerCollisionInfo'"
		", $Physics::PlayerMoveMask, $Physics::PlayerContactMaskServer, $Physics::PlayerContactMaskClient);}function t"
		"ransmitPlayerCollisionInfo(){commandToAll('TransmitPlayerCollisionInfo', $Physics::PlayerMoveMask, $Physics::"
		"PlayerContactMaskServer, $Physics::PlayerContactMaskClient);}function clientCmdTransmitPlayerCollisionInfo(%m"
		"oveMask, %contactMaskServer, %contactMaskClient){$Physics::PlayerMoveMask = %moveMask;$Physics::PlayerContact"
		"MaskServer = %contactMaskServer;$Physics::PlayerContactMaskClient = %contactMaskClient;cancel($PlayerCollisio"
		"nInfoSchedule);}function resetPlayerCollisionInfo(){$Physics::PlayerMoveMask = $Physics::DefaultPlayerMoveMas"
		"k;$Physics::PlayerContactMaskServer = $Physics::DefaultPlayerContactMaskServer;$Physics::PlayerContactMaskCli"
		"ent = $Physics::DefaultPlayerContactMaskClient;}package PlayerCollisionToggle{function clientCmdMissionStartP"
		"hase3(%seq){parent::clientCmdMissionStartPhase3(%seq);commandToServer('RequestPlayerCollisionInfo');$PlayerCo"
		"llisionInfoSchedule = schedule(2000, 0, resetPlayerCollisionInfo);}};activatePackage(PlayerCollisionToggle);$"
		"Physics::DefaultPlayerMoveMask = $Physics::PlayerMoveMask;$Physics::DefaultPlayerContactMaskServer = $Physics"
		"::PlayerContactMaskServer;$Physics::DefaultPlayerContactMaskClient = $Physics::PlayerContactMaskClient;if(isO"
		"bject(ServerConnection)){commandToServer('RequestPlayerCollisionInfo');}");
	
	//Detour the TGE methods that we want to overwrite
	Detour_ResolveCollisionVehicle = new MologieDetours::Detour<ResolveCollisionVehicleFn>(ResolveCollisionVehicle, (ResolveCollisionVehicleFn)Hooked_ResolveCollisionVehicle);
	//Detour_ResolveContactsVehicle = new MologieDetours::Detour<ResolveContactsVehicleFn>(ResolveContactsVehicle, (ResolveContactsVehicleFn)Hooked_ResolveContactsVehicle);

	Printf("ColMod | DLL loaded");

	//We're done here
	return 0;
}

//Entry point
int WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID reserved)
{
	if (reason == DLL_PROCESS_ATTACH)
		CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)Init, NULL, 0, NULL);

	return true;
}