#include "Torque.h"
#include <Psapi.h>
#include <string>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global variables

//Start of the Blockland.exe module in memory
static DWORD ImageBase;

//Length of the Blockland.exe module
static DWORD ImageSize;

//StringTable pointer
static DWORD StringTable;

//Global Variable dictionary pointer
static DWORD GlobalVars;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Engine function declarations

//Con::printf
PrintfFn Printf;

//Con::lookupNamespace
BLFUNC(DWORD*, , LookupNamespace, const char* ns);

//StringTable::insert
BLFUNC(const char*, __thiscall, StringTableInsert, DWORD stringTablePtr, const char* val, const bool caseSensitive)

//Namespace::addCommand overloads
BLFUNC(void, __thiscall, AddStringCommand, DWORD* ns, const char* name, StringCallback cb, const char *usage, int minArgs, int maxArgs);
BLFUNC(void, __thiscall, AddIntCommand, DWORD* ns, const char* name, IntCallback cb, const char *usage, int minArgs, int maxArgs);
BLFUNC(void, __thiscall, AddFloatCommand, DWORD* ns, const char* name, FloatCallback cb, const char *usage, int minArgs, int maxArgs);
BLFUNC(void, __thiscall, AddVoidCommand, DWORD* ns, const char* name, VoidCallback cb, const char *usage, int minArgs, int maxArgs);
BLFUNC(void, __thiscall, AddBoolCommand, DWORD* ns, const char* name, BoolCallback cb, const char *usage, int minArgs, int maxArgs);

//Exposing variables to torquescript
BLFUNC(void, __thiscall, AddVariable, DWORD dictionaryPtr, const char* name, int type, void* data);

//Executing code and calling torquescript functions
BLFUNC(const char*, , Evaluate, const char* string, bool echo, const char* fileName);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions

//Set the module start and length
void InitScanner(char* moduleName)
{
	//Find the module
	HMODULE module = GetModuleHandleA(moduleName);

	if (module)
	{
		//Retrieve information about the module
		MODULEINFO info;
		GetModuleInformation(GetCurrentProcess(), module, &info, sizeof(MODULEINFO));

		//Store relevant information
		ImageBase = (DWORD)info.lpBaseOfDll;
		ImageSize = info.SizeOfImage;
	}
}

//Compare data at two locations for equality
bool CompareData(PBYTE data, PBYTE pattern, char* mask)
{
	//Iterate over the data, pattern and mask in parallel
	for (; *mask; ++data, ++pattern, ++mask)
	{
		//And check for equality at each unmasked byte
		if (*mask == 'x' && *data != *pattern)
			return false;
	}

	return (*mask) == NULL;
}

//Find a pattern in memory
DWORD FindPattern(DWORD imageBase, DWORD imageSize, PBYTE pattern, char* mask)
{
	//Iterate over the image
	for (DWORD i = imageBase; i < imageBase + imageSize; i++)
	{
		//And check for matching pattern at every byte
		if (CompareData((PBYTE)i, pattern, mask))
			return i;
	}

	return 0;
}

//Scan the module for a pattern
DWORD ScanFunc(char* pattern, char* mask)
{
	//Just search for the pattern in the module
	return FindPattern(ImageBase, ImageSize - strlen(mask), (PBYTE)pattern, mask);
}

//Change a byte at a specific location in memory
void PatchByte(BYTE* location, BYTE value)
{
	//Remove protection
	DWORD oldProtection;
	VirtualProtect(location, 1, PAGE_EXECUTE_READWRITE, &oldProtection);

	//Change value
	*location = value;

	//Restore protection
	VirtualProtect(location, 1, oldProtection, &oldProtection);
}

//Register a torquescript function that returns a string. The function must look like this:
//const char* func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, StringCallback callBack, const char* usage, int minArgs, int maxArgs)
{
	AddStringCommand(LookupNamespace(nameSpace), StringTableInsert(StringTable, name, false), callBack, usage, minArgs, maxArgs);
}

//Register a torquescript function that returns an int. The function must look like this:
//int func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, IntCallback callBack, const char* usage, int minArgs, int maxArgs)
{
	AddIntCommand(LookupNamespace(nameSpace), StringTableInsert(StringTable, name, false), callBack, usage, minArgs, maxArgs);
}

//Register a torquescript function that returns a float. The function must look like this:
//float func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, FloatCallback callBack, const char* usage, int minArgs, int maxArgs)
{
	AddFloatCommand(LookupNamespace(nameSpace), StringTableInsert(StringTable, name, false), callBack, usage, minArgs, maxArgs);
}

//Register a torquescript function that returns nothing. The function must look like this:
//void func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, VoidCallback callBack, const char* usage, int minArgs, int maxArgs)
{
	AddVoidCommand(LookupNamespace(nameSpace), StringTableInsert(StringTable, name, false), callBack, usage, minArgs, maxArgs);
}

//Register a torquescript function that returns a bool. The function must look like this:
//bool func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, BoolCallback callBack, const char* usage, int minArgs, int maxArgs)
{
	AddBoolCommand(LookupNamespace(nameSpace), StringTableInsert(StringTable, name, false), callBack, usage, minArgs, maxArgs);
}

//Expose an integer variable to torquescript
void ConsoleVariable(const char* name, int* data)
{
	AddVariable(GlobalVars, name, 4, data);
}

//Expose a boolean variable to torquescript
void ConsoleVariable(const char* name, bool* data)
{
	AddVariable(GlobalVars, name, 6, data);
}

//Expose a float variable to torquescript
void ConsoleVariable(const char* name, float* data)
{
	AddVariable(GlobalVars, name, 8, data);
}

//Expose a string variable to torquescript
void ConsoleVariable(const char* name, char* data)
{
	AddVariable(GlobalVars, name, 10, data);
}

//Evaluate a torquescript string in global scope
const char* Eval(const char* str)
{
	return Evaluate(str, false, NULL);
}

//Initialize the Torque Interface
bool InitTorqueStuff()
{
	//Init the scanner
	InitScanner("Blockland.exe");

	//Printf is required for debug output, so find it first
	Printf = (PrintfFn)ScanFunc("\x8B\x4C\x24\x04\x8D\x44\x24\x08\x50\x6A\x00\x6A\x00\xE8\x00\x00\x00\x00\x83\xC4\x0C\xC3", "xxxxxxxxxxxxxx????xxxx");

	//Do nothing if we don't find it :(
	if (!Printf)
		return false;

	//First find all the functions
	BLSCAN(LookupNamespace, "\x8B\x44\x24\x04\x85\xC0\x75\x05", "xxxxxxxx");
	BLSCAN(StringTableInsert, "\x53\x8B\x5C\x24\x08\x55\x56\x57\x53", "xxxxxxxxx");

	//These are almost identical. Long sigs required
	BLSCAN(AddStringCommand,
		"\x8B\x44\x24\x04\x56\x50\xE8\x00\x00\x00\x00\x8B\xF0\xA1\x00\x00\x00\x00\x40\xB9\x00\x00\x00\x00\xA3"
		"\x00\x00\x00\x00\xE8\x00\x00\x00\x00\x8B\x4C\x24\x10\x8B\x54\x24\x14\x8B\x44\x24\x18\x89\x4E\x18\x8B"
		"\x4C\x24\x0C\x89\x56\x10\x89\x46\x14\xC7\x46\x0C\x01\x00\x00\x00\x89\x4E\x28\x5E\xC2\x14\x00",
		"xxxxxxx????xxx????xx????x????x????xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

	BLSCAN(AddIntCommand,
		"\x8B\x44\x24\x04\x56\x50\xE8\x00\x00\x00\x00\x8B\xF0\xA1\x00\x00\x00\x00\x40\xB9\x00\x00\x00\x00\xA3"
		"\x00\x00\x00\x00\xE8\x00\x00\x00\x00\x8B\x4C\x24\x10\x8B\x54\x24\x14\x8B\x44\x24\x18\x89\x4E\x18\x8B"
		"\x4C\x24\x0C\x89\x56\x10\x89\x46\x14\xC7\x46\x0C\x02\x00\x00\x00\x89\x4E\x28\x5E\xC2\x14\x00",
		"xxxxxxx????xxx????xx????x????x????xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

	BLSCAN(AddFloatCommand,
		"\x8B\x44\x24\x04\x56\x50\xE8\x00\x00\x00\x00\x8B\xF0\xA1\x00\x00\x00\x00\x40\xB9\x00\x00\x00\x00\xA3"
		"\x00\x00\x00\x00\xE8\x00\x00\x00\x00\x8B\x4C\x24\x10\x8B\x54\x24\x14\x8B\x44\x24\x18\x89\x4E\x18\x8B"
		"\x4C\x24\x0C\x89\x56\x10\x89\x46\x14\xC7\x46\x0C\x03\x00\x00\x00\x89\x4E\x28\x5E\xC2\x14\x00",
		"xxxxxxx????xxx????xx????x????x????xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

	BLSCAN(AddVoidCommand,
		"\x8B\x44\x24\x04\x56\x50\xE8\x00\x00\x00\x00\x8B\xF0\xA1\x00\x00\x00\x00\x40\xB9\x00\x00\x00\x00\xA3"
		"\x00\x00\x00\x00\xE8\x00\x00\x00\x00\x8B\x4C\x24\x10\x8B\x54\x24\x14\x8B\x44\x24\x18\x89\x4E\x18\x8B"
		"\x4C\x24\x0C\x89\x56\x10\x89\x46\x14\xC7\x46\x0C\x04\x00\x00\x00\x89\x4E\x28\x5E\xC2\x14\x00",
		"xxxxxxx????xxx????xx????x????x????xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

	BLSCAN(AddBoolCommand,
		"\x8B\x44\x24\x04\x56\x50\xE8\x00\x00\x00\x00\x8B\xF0\xA1\x00\x00\x00\x00\x40\xB9\x00\x00\x00\x00\xA3"
		"\x00\x00\x00\x00\xE8\x00\x00\x00\x00\x8B\x4C\x24\x10\x8B\x54\x24\x14\x8B\x44\x24\x18\x89\x4E\x18\x8B"
		"\x4C\x24\x0C\x89\x56\x10\x89\x46\x14\xC7\x46\x0C\x05\x00\x00\x00\x89\x4E\x28\x5E\xC2\x14\x00",
		"xxxxxxx????xxx????xx????x????x????xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

	BLSCAN(AddVariable, "\x8B\x44\x24\x04\x56\x8B\xF1\x80\x38\x24\x74\x1A", "xxxxxxxxxxxx");
	BLSCAN(Evaluate, "\x8A\x44\x24\x08\x84\xC0\x56\x57\x8B\x7C\x24\x0C", "xxxxxxxxxxxx");

	//The string table is used in lookupnamespace so we can get it's location
	StringTable = *(DWORD*)(*(DWORD*)((DWORD)LookupNamespace + 15));

	//Get the global variable dictionary pointer
	GlobalVars = *(DWORD*)(ScanFunc("\x8B\x44\x24\x0C\x8B\x4C\x24\x04\x50\x6A\x06", "xxxxxxxxxxx") + 13);

	return true;
}

ShapeBase::CollisionTimeout* ShapeBase::sFreeTimeoutList = 0;
Chunker<ShapeBase::CollisionTimeout> sTimeoutChunker;

namespace Sim {
	SimTime getCurrentTime() {
		std::string time(Eval("getSimTime();"));
		return atoi(time.c_str());
	}
}

// DATACHUNKER
DataChunker::DataChunker(S32 size)
{
	chunkSize = size;
	curBlock = new DataBlock(size);
	curBlock->next = NULL;
	curBlock->curIndex = 0;
}

DataChunker::~DataChunker()
{
	freeBlocks();
}

void *DataChunker::alloc(S32 size)
{
	//AssertFatal(size <= chunkSize, "Data chunk too large.");
	if (!curBlock || size + curBlock->curIndex > chunkSize)
	{
		DataBlock *temp = new DataBlock(chunkSize);
		temp->next = curBlock;
		temp->curIndex = 0;
		curBlock = temp;
	}
	void *ret = curBlock->data + curBlock->curIndex;
	curBlock->curIndex += (size + 3) & ~3; // dword align
	return ret;
}

DataChunker::DataBlock::DataBlock(S32 size)
{
	data = new U8[size];
}

DataChunker::DataBlock::~DataBlock()
{
	delete[] data;
}

void DataChunker::freeBlocks()
{
	while (curBlock)
	{
		DataBlock *temp = curBlock->next;
		delete curBlock;
		curBlock = temp;
	}
}


// RIGID
void Rigid::getOriginVector(const Point3F &p, Point3F* r) {
	r->x = p.x - worldCenterOfMass.x;
	r->y = p.y - worldCenterOfMass.y;
	r->z = p.z - worldCenterOfMass.z;
}

void Rigid::getVelocity(const Point3F& r, Point3F* v) {
	mCross(angVelocity, r, v);

	v->x += linVelocity.x;
	v->y += linVelocity.y;
	v->z += linVelocity.z;
}

bool Rigid::resolveCollision(Point3F& p, Point3F normal, Rigid* rigid)
{
	atRest = false;
	Point3F v1, v2, r1, r2;
	getOriginVector(p, &r1);
	getVelocity(r1, &v1);
	rigid->getOriginVector(p, &r2);
	rigid->getVelocity(r2, &v2);

	// Make sure they are converging  
	F32 nv = mDot(v1, normal);
	nv -= mDot(v2, normal);
	if (nv > -0.001f)
		return false;

	// Compute impulse  
	F32 d, n = -nv * (1 + restitution * rigid->restitution);
	Point3F a1, b1, c1;
	mCross(r1, normal, &a1);
	invWorldInertia.mulV(a1, &b1);
	mCross(b1, r1, &c1);

	Point3F a2, b2, c2;
	mCross(r2, normal, &a2);
	rigid->invWorldInertia.mulV(a2, &b2);
	mCross(b2, r2, &c2);

	Point3F c3;
	c3.x = c1.x + c2.x;
	c3.y = c1.y + c2.y;
	c3.z = c1.y + c2.y;

	d = oneOverMass + rigid->oneOverMass + mDot(c3, normal);
	Point3F impulse;
	impulse.x = normal.x * (n / d);
	impulse.y = normal.y * (n / d);
	impulse.z = normal.z * (n / d);

	applyImpulse(r1, impulse);
	impulse.neg();
	rigid->applyImpulse(r2, impulse);
	return true;
}

bool Rigid::resolveCollision(Point3F &p, Point3F normal) {
	atRest = false;
	Point3F v, r;
	getOriginVector(p, &r);
	getVelocity(r, &v);
	F32 n = -mDot(v, normal);
	if (n < 0)
		return false;

	// Collision impulse, straight forward force stuff.  
	F32 d = getZeroImpulse(r, normal);
	F32 j = n * (1 + restitution) * d;
	Point3F impulse;
	impulse.x = normal.x * j;
	impulse.y = normal.y * j;
	impulse.z = normal.z * j;

	// Friction impulse, calculated as a function of the  
	// amount of force it would take to stop the motion  
	// perpendicular to the normal.  
	Point3F newNormal;
	newNormal.x = normal.x * n;
	newNormal.y = normal.y * n;
	newNormal.z = normal.z * n;

	Point3F uv;
	uv.x = v.x + newNormal.x;
	uv.y = v.y + newNormal.y;
	uv.z = v.z + newNormal.z;

	F32 ul = uv.len();
	if (ul) {
		uv.x /= -ul;
		uv.y /= -ul;
		uv.z /= -ul;

		F32 u = ul * getZeroImpulse(r, uv);
		j *= friction;
		if (u > j)
			u = j;

		impulse.x += uv.x * u;
		impulse.y += uv.y * u;
		impulse.z += uv.z * u;
	}

	// do the deed
	applyImpulse(r, impulse);

	return true;
}

F32 Rigid::getZeroImpulse(const Point3F& r, const Point3F& normal)
{
	// get ready
	Point3F a, b, c;
	mCross(r, normal, &a);

	// do the thing
	b.x = invWorldInertia[0] * invWorldInertia[0] + invWorldInertia[1] * a.y + invWorldInertia[2] * a.z;
	b.y = invWorldInertia[4] * a.x + invWorldInertia[5] * a.y + invWorldInertia[6] * a.z;
	b.z = invWorldInertia[8] * a.x + invWorldInertia[9] * a.y + invWorldInertia[10] * a.z;

	// done the thing, return
	mCross(b, r, &c);
	return 1 / (oneOverMass + mDot(c, normal));
}

void Rigid::applyImpulse(const Point3F &r, const Point3F &impulse)
{
	atRest = false;

	// Linear momentum and velocity
	linMomentum.x += impulse.x;
	linMomentum.y += impulse.y;
	linMomentum.z += impulse.z;

	linVelocity.x = linMomentum.x * oneOverMass;
	linVelocity.y = linMomentum.y * oneOverMass;
	linVelocity.z = linMomentum.z * oneOverMass;

	// Rotational momentum and velocity
	Point3F tv;
	mCross(r, impulse, &tv);

	angMomentum.x += tv.x;
	angMomentum.y += tv.y;
	angMomentum.z += tv.z;

	// do the thing
	angVelocity.x = invWorldInertia[0] * invWorldInertia[0] + invWorldInertia[1] * angMomentum.y + invWorldInertia[2] * angMomentum.z;
	angVelocity.y = invWorldInertia[4] * angMomentum.x + invWorldInertia[5] * angMomentum.y + invWorldInertia[6] * angMomentum.z;
	angVelocity.z = invWorldInertia[8] * angMomentum.x + invWorldInertia[9] * angMomentum.y + invWorldInertia[10] * angMomentum.z;
}

void Rigid::updateVelocity() {
	linVelocity.x = linMomentum.x * oneOverMass;
	linVelocity.y = linMomentum.y * oneOverMass;
	linVelocity.z = linMomentum.z * oneOverMass;

	angVelocity.x = invWorldInertia[0] * invWorldInertia[0] + invWorldInertia[1] * angMomentum.y + invWorldInertia[2] * angMomentum.z;
	angVelocity.y = invWorldInertia[4] * angMomentum.x + invWorldInertia[5] * angMomentum.y + invWorldInertia[6] * angMomentum.z;
	angVelocity.z = invWorldInertia[8] * angMomentum.x + invWorldInertia[9] * angMomentum.y + invWorldInertia[10] * angMomentum.z;
}