#include "Windows.h"
#include <cmath>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Macros

//Typedef an engine function to use it later
#define BLFUNC(returnType, convention, name, ...)         \
	typedef returnType (convention*name##Fn)(__VA_ARGS__); \
	static name##Fn name;

//Typedef an exported engine function to use it later
#define BLFUNC_EXTERN(returnType, convention, name, ...)  \
	typedef returnType (convention*name##Fn)(__VA_ARGS__); \
	extern name##Fn name;

//Search for an engine function in blockland
#define BLSCAN(target, pattern, mask)            \
	target = (target##Fn)ScanFunc(pattern, mask); \
	if(target == NULL)                             \
		Printf("ColMod | Cannot find function "#target"!"); 


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Engine function declarations

//Con::printf
BLFUNC_EXTERN(void, , Printf, const char* format, ...);

#define BIT(x) (1 << (x))

//Callback types for exposing methods to torquescript
typedef const char* (*StringCallback)(DWORD* obj, int argc, const char* argv[]);
typedef int(*IntCallback)(DWORD* obj, int argc, const char* argv[]);
typedef float(*FloatCallback)(DWORD* obj, int argc, const char* argv[]);
typedef void(*VoidCallback)(DWORD* obj, int argc, const char* argv[]);
typedef bool(*BoolCallback)(DWORD* obj, int argc, const char* argv[]);

//Torque types
typedef int S32;
typedef short S16;
typedef float F32;
typedef double F64;
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned int U32;

typedef U32 AUDIOHANDLE;
typedef U32 SimTime;

typedef U32 SimObjectId;

typedef const char* StringTableEntry;

template<class T>

class Vector
{
protected:
	U32 mElementCount;
	U32 mArraySize;
	T*  mArray;
};

class StringHandle
{
	U32 index;
};

class TextureHandle
{
	void *object;
};

// Some structs to help with vehicle collision
struct Point3F {
	F32 x; // +0x0
	F32 y; // +0x4
	F32 z; // +0x8

	F32 Point3F::len() const
	{
		return sqrt(x*x + y*y + z*z);
	}

	void Point3F::neg()
	{
		x = -x;
		y = -y;
		z = -z;
	}
};

struct Point2F {
	F32 x; // +0x0
	F32 y; // +0x4
};

struct QuatF
{
	F32  x, y, z, w;
};

inline void m_matF_x_vectorF(const F32 *m, const F32 *v, F32 *vresult)
{
	vresult[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
	vresult[1] = m[4] * v[0] + m[5] * v[1] + m[6] * v[2];
	vresult[2] = m[8] * v[0] + m[9] * v[1] + m[10] * v[2];
}

struct MatrixF
{
	F32 m[16];

	static U32 idx(U32 i, U32 j) { return (i + j * 4); }

	/// Convenience functions to allow people to treat this like an array.
	F32& operator ()(S32 row, S32 col) { return m[idx(col, row)]; }

	operator F32*() { return (m); }              ///< Allow people to get at m.
	operator F32*() const { return (F32*)(m); }  ///< Allow people to get at m.

	void mulV(const Point3F &v, Point3F *d) const
	{
		// M * v -> d
		m_matF_x_vectorF(*this, &v.x, &d->x);
	}
};

struct Box3F
{
public:
	Point3F min; ///< Minimum extents of box
	Point3F max; ///< Maximum extents of box
};

class SphereF
{
public:
	Point3F center;
	F32     radius;
};

namespace Sim {
	SimTime getCurrentTime();
}

//----------------------------------------------------------------------------
/// Implements a chunked data allocater.
///
/// Calling new/malloc all the time is a time consuming operation. Therefore,
/// we provide the DataChunker, which allocates memory in blockss of
/// chunkSize (by default 16k, see ChunkSize, though it can be set in
/// the constructor), then doles it out as requested, in chunks of up to
/// chunkSize in size.
///
/// It will assert if you try to get more than ChunkSize bytes at a time,
/// and it deals with the logic of allocating new blocks and giving out
/// word-aligned chunks.
///
/// Note that new/free/realloc WILL NOT WORK on memory gotten from the
/// DataChunker. This also only grows (you can call freeBlocks to deallocate
/// and reset things).
class DataChunker
{
public:
	enum {
		ChunkSize = 16376 ///< Default size for chunks.
	};

private:
	/// Block of allocated memory.
	///
	/// <b>This has nothing to do with datablocks as used in the rest of Torque.</b>
	struct DataBlock
	{
		DataBlock *next;
		U8 *data;
		S32 curIndex;
		DataBlock(S32 size);
		~DataBlock();
	};
	DataBlock *curBlock;
	S32 chunkSize;

public:

	/// Return a pointer to a chunk of memory from a pre-allocated block.
	///
	/// This memory goes away when you call freeBlocks.
	///
	/// This memory is word-aligned.
	/// @param   size    Size of chunk to return. This must be less than chunkSize or else
	///                  an assertion will occur.
	void *alloc(S32 size);

	/// Free all allocated memory blocks.
	///
	/// This invalidates all pointers returned from alloc().
	void freeBlocks();

	/// Initialize using blocks of a given size.
	///
	/// One new block is allocated at constructor-time.
	///
	/// @param   size    Size in bytes of the space to allocate for each block.
	DataChunker(S32 size = ChunkSize);
	~DataChunker();
};


//----------------------------------------------------------------------------

template<class T>
class Chunker : private DataChunker
{
public:
	Chunker(S32 size = DataChunker::ChunkSize) : DataChunker(size) {};
	T* alloc() { return reinterpret_cast<T*>(DataChunker::alloc(S32(sizeof(T)))); }
	void clear() { freeBlocks(); };
};

// --------------------------------------------------
// stopped giving a shit about here.

class BitSet32
{
private:
	/// Internal representation of bitset.
	U32 mbits;

public:
	/// Are any of the specified bit(s) set?
	bool test(const U32 m) const { return (mbits & m) != 0; }
};

class ConsoleObject
{
};

class SimObject : public ConsoleObject {
	typedef ConsoleObject Parent;

	//-------------------------------------- Structures and enumerations
private:

	/// Flags for use in mFlags
	enum {
		Deleted = BIT(0),   ///< This object is marked for deletion.
		Removed = BIT(1),   ///< This object has been unregistered from the object system.
		Added = BIT(3),   ///< This object has been registered with the object system.
		Selected = BIT(4),   ///< This object has been marked as selected. (in editor)
		Expanded = BIT(5),   ///< This object has been marked as expanded. (in editor)
		ModStaticFields = BIT(6),    ///< The object allows you to read/modify static fields
		ModDynamicFields = BIT(7)     ///< The object allows you to read/modify dynamic fields
	};
public:
	/// @name Notification
	/// @{
	struct Notify {
		enum Type {
			ClearNotify,   ///< Notified when the object is cleared.
			DeleteNotify,  ///< Notified when the object is deleted.
			ObjectRef,     ///< Cleverness to allow tracking of references.
			Invalid        ///< Mark this notification as unused (used in freeNotify).
		} type;
		void *ptr;        ///< Data (typically referencing or interested object).
		Notify *next;     ///< Next notification in the linked list.
	};

	/// @}

	enum WriteFlags {
		SelectedOnly = BIT(0) ///< Passed to SimObject::write to indicate that only objects
							  ///  marked as selected should be outputted. Used in SimSet.
	};

	SimObjectId getId() { return mId; }

private:
	// dictionary information stored on the object
	StringTableEntry objectName;
	SimObject*       nextNameObject;
	SimObject*       nextManagerNameObject;
	SimObject*       nextIdObject;

	void*   mGroup;  ///< SimGroup we're contained in, if any.
	BitSet32    mFlags;

	/// @name Notification
	/// @{
	Notify*     mNotifyList;
	/// @}

protected:
	SimObjectId mId;         ///< Id number for this object.
	void*  mNameSpace;
	U32         mTypeMask;

protected:
	/// @name Notification
	/// Helper functions for notification code.
	/// @{

	static SimObject::Notify *mNotifyFreeList;
};

template <class T> class SimObjectPtr
{
private:
	SimObject *mObj;
};

class SimDataBlock : public SimObject
{
	typedef SimObject Parent;
public:

	/// @name Datablock Internals
	/// @{

protected:
	S32  modifiedKey;

public:
	static SimObjectId sNextObjectId;
	static S32         sNextModifiedKey;

	/// Assign a new modified key.
	///
	/// Datablocks are assigned a modified key which is updated every time
	/// a static field of the datablock is changed. These are gotten from
	/// a global store.
};

class NetObject : public SimObject
{
	/// Mask indicating which states are dirty and need to be retransmitted on this
	/// object.
	U32 mDirtyMaskBits;

	/// @name Dirty List
	///
	/// Whenever a NetObject becomes "dirty", we add it to the dirty list.
	/// We also remove ourselves on the destructor.
	///
	/// This is done so that when we want to send updates (in NetConnection),
	/// it's very fast to find the objects that need to be updated.
	/// @{

	/// Static pointer to the head of the dirty NetObject list.
	static NetObject *mDirtyList;

	/// Next item in the dirty list...
	NetObject *mPrevDirtyList;

	/// Previous item in the dirty list...
	NetObject *mNextDirtyList;

	/// @}

public:
	bool isGhost() const;         ///< Is this is a ghost?
protected:

	/// Pointer to the server object; used only when we are doing "short-circuited" networking.
	///
	/// When we are running with client and server on the same system (which can happen be either
	/// when we are doing a single player game, or if we're hosting a multiplayer game and having
	/// someone playing on the same instance), we can do some short circuited code to enhance
	/// performance.
	///
	/// This variable is used to make it simpler; if we are running in short-circuited mode, it's set
	/// to the object on the server that this NetObject is ghosting.
	///
	/// @note "Premature optimization is the root of all evil" - Donald Knuth. The current codebase
	///       uses this feature in three small places, mostly for non-speed-related purposes.
	SimObjectPtr<NetObject> mServerObject;

	enum NetFlags
	{
		IsGhost = BIT(1),   ///< This is a ghost.
		ScopeAlways = BIT(6),  ///< Object always ghosts to clients.
		ScopeLocal = BIT(7),  ///< Ghost only to local client.
		Ghostable = BIT(8),  ///< Set if this object CAN ghost.

		MaxNetFlagBit = 15
	};

	BitSet32 mNetFlags;              ///< Flag values from NetFlags
	U32 mNetIndex;                   ///< The index of this ghost in the GhostManager on the server.

	void *mFirstObjectRef;      ///< Head of a linked list storing GhostInfos referencing this NetObject.
};

//-----------------------------------------------------------------------------

inline bool NetObject::isGhost() const
{
	return mNetFlags.test(IsGhost);
}

class LightInfo
{

public:
	enum Type {
		Point = 0,
		Spot = 1,
		Vector = 2,
		Ambient = 3
	};
	Type        mType;

	Point3F     mPos;
	Point3F     mDirection; ///< For spot and vector lights, indicates the direction of the light.
	Point3F      mColor;
	Point3F      mAmbient;
	F32         mRadius;    ///< For point and spot lights, indicates the effective radius of the light.

private:
	S32         mScore;     ///< Used internally to track importance of light.
};
typedef Vector<LightInfo*> LightInfoList;

struct GameBaseData : public SimDataBlock {
private:
	typedef SimDataBlock Parent;

public:
	bool packed;
	StringTableEntry category;
	StringTableEntry className;
};

class SceneObject;
class SceneObjectRef
{
public:
	SceneObject*    object;
	SceneObjectRef* nextInBin;
	SceneObjectRef* prevInBin;
	SceneObjectRef* nextInObj;

	U32             zone;
};

class Container
{
public:
	struct Link
	{
		Link* next;
		Link* prev;
		Link();
		void unlink();
		void linkAfter(Link* ptr);
	};

	struct CallbackInfo
	{
		void* polyList;
		Box3F boundingBox;
		SphereF boundingSphere;
		void *key;
	};

	static const U32 csmNumBins;
	static const F32 csmBinSize;
	static const F32 csmTotalBinSize;
	static const U32 csmRefPoolBlockSize;
	static U32    smCurrSeqKey;

private:
	Link mStart, mEnd;

	SceneObjectRef*         mFreeRefPool;
	Vector<SceneObjectRef*> mRefPoolBlocks;

	SceneObjectRef* mBinArray;
	SceneObjectRef  mOverflowBin;


private:
	Vector<SimObjectPtr<SceneObject>*>  mSearchList;///< Object searches to support console querying of the database.  ONLY WORKS ON SERVER
	S32                                 mCurrSearchPos;
	Point3F                             mSearchReferencePoint;
};
class Convex;

class SceneObject : public NetObject, public Container::Link
{
	typedef NetObject Parent;
	friend class Container;
	friend class SceneGraph;
	friend class SceneState;

	//-------------------------------------- Public constants
public:
	enum
	{
		MaxObjectZones = 128
	};

	enum TraversalState
	{
		Pending = 0,
		Working = 1,
		Done = 2
	};

	enum SceneObjectMasks
	{
		ScaleMask = BIT(0),
		NextFreeMask = BIT(1)
	};

	//-------------------------------------- Public interfaces
	// C'tors and D'tors
private:
	SceneObject(const SceneObject&); ///< @deprecated disallowed

public:
	SceneObject();
	virtual ~SceneObject();

	/// Returns a value representing this object which can be passed to script functions.
	const char* scriptThis();

public:
	/// @name Collision and transform related interface
	///
	/// The Render Transform is the interpolated transform with respect to the
	/// frame rate. The Render Transform will differ from the object transform
	/// because the simulation is updated in fixed intervals, which controls the
	/// object transform. The framerate is, most likely, higher than this rate,
	/// so that is why the render transform is interpolated and will differ slightly
	/// from the object transform.
	///
	/// @{

	/// Disables collisions for this object including raycasts
	virtual void disableCollision();

	/// Enables collisions for this object
	virtual void enableCollision();

	/// Returns true if collisions are enabled
	bool         isCollisionEnabled() const { return mCollisionCount == 0; }

	/// Returns true if this object allows itself to be displaced
	/// @see displaceObject
	virtual bool    isDisplacable() const;

	/// Returns the momentum of this object
	virtual Point3F getMomentum() const;

	/// Sets the momentum of this object
	/// @param   momentum   Momentum
	virtual void    setMomentum(const Point3F &momentum);

	/// Returns the mass of this object
	virtual F32     getMass() const;

	/// Displaces this object by a vector
	/// @param   displaceVector   Displacement vector
	virtual bool    displaceObject(const Point3F& displaceVector);

	/// Returns the transform which can be used to convert object space
	/// to world space
	const MatrixF& getTransform() const { return mObjToWorld; }

	/// Returns the transform which can be used to convert world space
	/// into object space
	const MatrixF& getWorldTransform() const { return mWorldToObj; }

	/// Returns the scale of the object
	const Point3F& getScale() const { return mObjScale; }

	/// Returns the bounding box for this object in local coordinates
	const Box3F&   getObjBox() const { return mObjBox; }

	/// Returns the bounding box for this object in world coordinates
	const Box3F&   getWorldBox() const { return mWorldBox; }

	/// Returns the bounding sphere for this object in world coordinates
	const SphereF& getWorldSphere() const { return mWorldSphere; }

	/// Returns the center of the bounding box in world coordinates
	//Point3F        getBoxCenter() const { return (mWorldBox.min + mWorldBox.max) * 0.5f; }

	/// Sets the Object -> World transform
	///
	/// @param   mat   New transform matrix
	virtual void setTransform(const MatrixF & mat);

	/// Sets the scale for the object
	/// @param   scale   Scaling values
	virtual void setScale(const Point3F & scale);

	/// This sets the render transform for this object
	/// @param   mat   New render transform
	virtual void   setRenderTransform(const MatrixF &mat);

	/// Returns the render transform
	const MatrixF& getRenderTransform() const { return mRenderObjToWorld; }

	/// Returns the render transform to convert world to local coordinates
	const MatrixF& getRenderWorldTransform() const { return mRenderWorldToObj; }

	/// Returns the render world box
	const Box3F&   getRenderWorldBox()  const { return mRenderWorldBox; }

	/// Builds a convex hull for this object.
	///
	/// Think of a convex hull as a low-res mesh which covers, as tightly as
	/// possible, the object mesh, and is used as a collision mesh.
	/// @param   box
	/// @param   convex   Convex mesh generated (out)
	virtual void     buildConvex(const Box3F& box, Convex* convex);

	/// Builds a list of polygons which intersect a bounding volume.
	///
	/// This will use either the sphere or the box, not both, the
	/// SceneObject implimentation ignores sphere.
	///
	/// @see AbstractPolyList
	/// @param   polyList   Poly list build (out)
	/// @param   box        Box bounding volume
	/// @param   sphere     Sphere bounding volume
	//virtual bool     buildPolyList(AbstractPolyList* polyList, const Box3F &box, const SphereF &sphere);

	/// Builds a collision tree of all the polygons which collide with a bounding volume.
	///
	/// @note Not implemented in SceneObject. @see TerrainBlock::buildCollisionBSP
	/// @param   tree     BSP tree built (out)
	/// @param   box      Box bounding volume
	/// @param   sphere   Sphere bounding volume
	//virtual BSPNode* buildCollisionBSP(BSPTree *tree, const Box3F &box, const SphereF &sphere);

	/// Casts a ray and obtain collision information, returns true if RayInfo is modified.
	///
	/// @param   start   Start point of ray
	/// @param   end   End point of ray
	/// @param   info   Collision information obtained (out)
	//virtual bool castRay(const Point3F &start, const Point3F &end, RayInfo* info);

	//virtual bool collideBox(const Point3F &start, const Point3F &end, RayInfo* info);

	/// Returns the position of the object.
	Point3F  getPosition() const;

	/// Returns the render-position of the object.
	///
	/// @see getRenderTransform
	Point3F  getRenderPosition() const;

	/// Sets the position of the object
	void     setPosition(const Point3F &pos);

	/// Gets the velocity of the object
	virtual Point3F getVelocity() const;

	/// Sets the velocity of the object
	/// @param  v  Velocity
	virtual void setVelocity(const Point3F &v);

	/// @}

public:
	/// @name Zones
	///
	/// A zone is a portalized section of an InteriorInstance, and an InteriorInstance can manage more than one zone.
	/// There is always at least one zone in the world, zone 0, which represens the whole world. Any
	/// other zone will be in an InteriorInstance. Torque keeps track of the zones containing an object
	/// as it moves throughout the world. An object can exists in multiple zones at once.
	/// @{

	/// Returns true if this object is managing zones.
	///
	/// This is only true in the case of InteriorInstances which have zones in them.
	bool isManagingZones() const;

	/// Gets the index of the first zone this object manages in the collection of zones.
	U32  getZoneRangeStart() const { return mZoneRangeStart; }

	/// Gets the number of zones containing this object.
	U32  getNumCurrZones() const { return mNumCurrZones; }

	/// Returns the nth zone containing this object.
	U32  getCurrZone(const U32 index) const;

	/// If an object exists in multiple zones, this method will give you the
	/// number and indices of these zones (storing them in the provided variables).
	///
	/// @param   obj      Object in question.
	/// @param   zones    Indices of zones containing the object. (out)
	/// @param   numZones Number of elements in the returned array.  (out)
	virtual bool getOverlappingZones(SceneObject* obj, U32* zones, U32* numZones);

	/// Returns the zone containing p.
	///
	/// @param   p   Point to test.
	virtual U32  getPointZone(const Point3F& p);

	/// This is called on a zone managing object to scope all the zones managed.
	///
	/// @param   rootPosition   Camera position
	/// @param   rootDistance   Camera visible distance
	/// @param   zoneScopeState Array of booleans which line up with the collection of zones, marked true if that zone is scoped (out)
	virtual bool scopeObject(const Point3F&        rootPosition,
		const F32             rootDistance,
		bool*                 zoneScopeState);
	/// @}

	/// Called when the object is supposed to render itself.
	///
	/// @param   state   Current rendering state.
	///                  @see SceneState
	/// @param   image   Image associated with this object to render.
	///                  @see SceneRenderImage
	//virtual void renderObject(SceneState *state, SceneRenderImage *image);

	/// Called when the SceneGraph is ready for the registration of RenderImages.
	///
	/// @see SceneState
	///
	/// @param   state               SceneState
	/// @param   stateKey            State key of the current SceneState
	/// @param   startZone           Base zone index
	/// @param   modifyBaseZoneState If true, the object needs to modify the zone state.
	virtual bool prepRenderImage(SceneState *state, const U32 stateKey, const U32 startZone,
		const bool modifyBaseZoneState = false);

	/// Adds object to the client or server container depending on the object
	void addToScene();

	/// Removes the object from the client/server container
	void removeFromScene();

	//-------------------------------------- Derived class interface
	// Overrides
protected:
	bool onAdd();
	void onRemove();

	// Overrideables
protected:
	/// Called when this is added to the SceneGraph.
	///
	/// @param   graph   SceneGraph this is getting added to
	virtual bool onSceneAdd(SceneGraph *graph);

	/// Called when this is removed from the SceneGraph
	virtual void onSceneRemove();

	/// Called when the size of the object changes
	virtual void onScaleChanged();

	/// @name Portals
	/// @{

	/// This is used by a portal controling object to transform the base-modelview
	/// used by the scenegraph for rendering to the modelview it needs to render correctly.
	///
	/// @see MirrorSubObject
	///
	/// @param   portalIndex   Index of portal in the list of portals controlled by the object.
	/// @param   oldMV         Current modelview matrix used by the SceneGraph (in)
	/// @param   newMV         New modelview to be used by the SceneGraph (out)
	virtual void transformModelview(const U32 portalIndex, const MatrixF& oldMV, MatrixF* newMV);

	/// Used to tranform the position of a point based on a portal.
	///
	/// @param   portalIndex   Index of a portal to transform by.
	/// @param   point         Point to transform.
	virtual void transformPosition(const U32 portalIndex, Point3F& point);

	/// Returns a new view frustum for the portal.
	///
	/// @param   portalIndex   Which portal in the list of portals the object controls
	/// @param   oldFrustum    Current frustum.
	/// @param   nearPlane     Near clipping plane.
	/// @param   farPlane      Far clipping plane.
	/// @param   oldViewport   Current viewport.
	/// @param   newFrustum    New view frustum to use. (out)
	/// @param   newViewport   New viewport to use. (out)
	/// @param   flippedMatrix Should the object should use a flipped matrix to calculate viewport and frustum?
	/*
	virtual bool computeNewFrustum(const U32      portalIndex,
		const F64*     oldFrustum,
		const F64      nearPlane,
		const F64      farPlane,
		const RectI&   oldViewport,
		F64*           newFrustum,
		RectI&         newViewport,
		const bool     flippedMatrix);
		*/

	/// Called before things are to be rendered from the portals point of view, to set up
	/// everything the portal needs to render correctly.
	///
	/// @param   portalIndex   Index of portal to use.
	/// @param   pCurrState    Current SceneState
	/// @param   pParentState  SceneState used before this portal was activated
	virtual void openPortal(const U32   portalIndex,
		SceneState* pCurrState,
		SceneState* pParentState);

	/// Called after rendering of a portal is complete, this resets the states
	/// the previous call to openPortal() changed.
	///
	/// @param   portalIndex    Index of portal to use.
	/// @param   pCurrState     Current SceneState
	/// @param   pParentState   SceneState used before this portal was activated
	virtual void closePortal(const U32   portalIndex,
		SceneState* pCurrState,
		SceneState* pParentState);
public:

	/// Returns the plane of the portal in world space.
	///
	/// @param   portalIndex   Index of portal to use.
	/// @param   plane         Plane of the portal in world space (out)
	virtual void getWSPortalPlane(const U32 portalIndex, Point3F *plane);

	/// @}

protected:
	/// Sets the mLastState and mLastStateKey.
	///
	/// @param   state   SceneState to set as the last state
	/// @param   key     Key to set as the last state key
	void          setLastState(SceneState *state, U32 key);

	/// Returns true if the provided SceneState and key are set as this object's
	/// last state and key.
	///
	/// @param   state    SceneState in question
	/// @param   key      State key in question
	bool          isLastState(SceneState *state, U32 key) const;


	/// @name Traversal State
	///
	/// The SceneGraph traversal is recursive and the traversal state of an object
	/// can be one of three things:
	///           - Pending - The object has not yet been examined for zone traversal.
	///           - Working - The object is currently having its zones traversed.
	///           - Done    - The object has had all of its zones traversed or doesn't manage zones.
	///
	/// @note These states were formerly referred to as TraverseColor, with White, Black, and
	///       Gray; this was changed in Torque 1.2 by Pat "KillerBunny" Wilson. This code is
	///       only used internal to this class
	/// @{

	// These two replaced by TraversalState because that makes more sense -KB
	//void          setTraverseColor(TraverseColor);
	//TraverseColor getTraverseColor() const;
	// ph34r teh muskrat! - Travis Colure

	/// This sets the traversal state of the object.
	///
	/// @note This is used internally; you should not normally need to call it.
	/// @param   s   Traversal state to assign
	void           setTraversalState(TraversalState s);

	/// Returns the traversal state of this object
	TraversalState getTraversalState() const;

	/// @}

	/// @name Lighting
	/// @{

	struct LightingInfo
	{
		LightingInfo();

		bool                       mUseInfo;
		bool                       mDirty;
		Point3F                     mDefaultColor;
		Point3F                     mAlarmColor;

		SimObjectPtr<SceneObject>  mInterior;

		bool                       mHasLastColor;
		Point3F                     mLastColor;
		U32                        mLastTime;

		static LightInfo           smAmbientLight;

		enum
		{
			Interior = 0,
			Terrain,
		};
		U32                        mLightSource;
	};

	/// Sets up lighting for the rendering of this object
	virtual void installLights();

	/// Removes lighting for the rendering of this object
	virtual void uninstallLights();

	/// Gets the color of the ambient light in the area of the object and
	/// stores it in the provided ColorF.
	///
	/// @param   col   Ambient color (out)
	virtual bool getLightingAmbientColor(Point3F * col);

	LightingInfo      mLightingInfo; ///< Lighting info for this object

									 /// @}

protected:

	/// @name Transform and Collision Members
	/// @{

	///
	Container* mContainer;

	MatrixF mObjToWorld;   ///< Transform from object space to world space
	MatrixF mWorldToObj;   ///< Transform from world space to object space (inverse)
	Point3F mObjScale;     ///< Object scale

	Box3F   mObjBox;       ///< Bounding box in object space
	Box3F   mWorldBox;     ///< Bounding box in world space
	SphereF mWorldSphere;  ///< Bounding sphere in world space

	MatrixF mRenderObjToWorld;    ///< Render matrix to transform object space to world space
	MatrixF mRenderWorldToObj;    ///< Render matrix to transform world space to object space
	Box3F   mRenderWorldBox;      ///< Render bounding box in world space
	SphereF mRenderWorldSphere;   ///< Render bounxing sphere in world space

								  /// Regenerates the world-space bounding box and bounding sphere
	void    resetWorldBox();

	/// Regenerates the render-world-space bounding box and sphere
	void    resetRenderWorldBox();

	SceneObjectRef* mZoneRefHead;
	SceneObjectRef* mBinRefHead;

	U32 mBinMinX;
	U32 mBinMaxX;
	U32 mBinMinY;
	U32 mBinMaxY;

	/// @}

	/// @name Container Interface
	///
	/// When objects are searched, we go through all the zones and ask them for
	/// all of their objects. Because an object can exist in multiple zones, the
	/// container sequence key is set to the id of the current search. Then, while
	/// searching, we check to see if an object's sequence key is the same as the
	/// current search key. If it is, it will NOT be added to the list of returns
	/// since it has already been processed.
	///
	/// @{

	U32  mContainerSeqKey;  ///< Container sequence key

							/// Returns the container sequence key
	U32  getContainerSeqKey() const { return mContainerSeqKey; }

	/// Sets the container sequence key
	void setContainerSeqKey(const U32 key) { mContainerSeqKey = key; }
	/// @}

public:

	/// Returns a pointer to the container that contains this object
	Container* getContainer() { return mContainer; }

protected:
	S32     mCollisionCount;

	bool    mGlobalBounds;

public:
	/// Returns the type mask for this object
	U32 getTypeMask() { return(mTypeMask); }

	const bool isGlobalBounds() const
	{
		return mGlobalBounds;
	}

	/// If global bounds are set to be true, then the object is assumed to
	/// have an infinitely large bounding box for collision and rendering
	/// purposes.
	///
	/// They can't be toggled currently.
	void setGlobalBounds()
	{
		// ...
	}


	/// @name Rendering Members
	/// @{
protected:
	SceneGraph* mSceneManager;      ///< SceneGraph that controls this object
	U32         mZoneRangeStart;    ///< Start of range of zones this object controls, 0xFFFFFFFF == no zones

	U32         mNumCurrZones;     ///< Number of zones this object exists in

private:
	TraversalState mTraversalState;  ///< State of this object in the SceneGraph traversal - DON'T MESS WITH THIS
	SceneState*    mLastState;       ///< Last SceneState that was used to render this object.
	U32            mLastStateKey;    ///< Last state key that was used to render this object.

									 /// @}

									 /// @name Persist and console
									 /// @{
public:
	static void initPersistFields();
	void inspectPostApply();
	//DECLARE_CONOBJECT(SceneObject);

	/// @}
};

//--------------------------------------------------------------------------
//-------------------------------------- Inlines
//
inline bool SceneObject::isManagingZones() const
{
	return mZoneRangeStart != 0xFFFFFFFF;
}

inline void SceneObject::setLastState(SceneState* state, U32 key)
{
	mLastState = state;
	mLastStateKey = key;
}

inline bool SceneObject::isLastState(SceneState* state, U32 key) const
{
	return (mLastState == state && mLastStateKey == key);
}

inline void SceneObject::setTraversalState(TraversalState s) {
	mTraversalState = s;
}

inline SceneObject::TraversalState SceneObject::getTraversalState() const {
	return mTraversalState;
}

// For truly it is written: "The wise man extends GameBase for his purposes,
// while the fool has the ability to eject shell casings from the belly of his
// dragon." -- KillerBunny

/// Base class for game objects which use datablocks, networking, are editable,
/// and need to process ticks.
///
/// @section GameBase_process GameBase and ProcessList
///
/// GameBase adds two kinds of time-based updates. Torque works off of a concept
/// of ticks. Ticks are slices of time 32 milliseconds in length. There are three
/// methods which are used to update GameBase objects that are registered with
/// the ProcessLists:
///      - processTick(Move*) is called on each object once for every tick, regardless
///        of the "real" framerate.
///      - interpolateTick(float) is called on client objects when they need to interpolate
///        to match the next tick.
///      - advanceTime(float) is called on client objects so they can do time-based behaviour,
///        like updating animations.
///
/// Torque maintains a server and a client processing list; in a local game, both
/// are populated, while in multiplayer situations, either one or the other is
/// populated.
///
/// You can control whether an object is considered for ticking by means of the
/// setProcessTick() method.
///
/// @section GameBase_datablock GameBase and Datablocks
///
/// GameBase adds support for datablocks. Datablocks are secondary classes which store
/// static data for types of game elements. For instance, this means that all "light human
/// male armor" type Players share the same datablock. Datablocks typically store not only
/// raw data, but perform precalculations, like finding nodes in the game model, or
/// validating movement parameters.
///
/// There are three parts to the datablock interface implemented in GameBase:
///      - <b>getDataBlock()</b>, which gets a pointer to the current datablock. This is
///        mostly for external use; for in-class use, it's better to directly access the
///        mDataBlock member.
///      - <b>setDataBlock()</b>, which sets mDataBlock to point to a new datablock; it
///        uses the next part of the interface to inform subclasses of this.
///      - <b>onNewDataBlock()</b> is called whenever a new datablock is assigned to a GameBase.
///
/// Datablocks are also usable through the scripting language.
///
/// @see SimDataBlock for more details.
///
/// @section GameBase_networking GameBase and Networking
///
/// writePacketData() and readPacketData() are called to transfer information needed for client
/// side prediction. They are usually used when updating a client of its control object state.
///
/// Subclasses of GameBase usually transmit positional and basic status data in the packUpdate()
/// functions, while giving velocity, momentum, and similar state information in the writePacketData().
///
/// writePacketData()/readPacketData() are called <i>in addition</i> to packUpdate/unpackUpdate().
///
/// @nosubgrouping
class GameBase : public SceneObject
{
private:
	typedef SceneObject Parent;

	/// @name Datablock
	/// @{
private:
	GameBaseData*     mDataBlock;
	StringTableEntry  mNameTag;

	/// @}

	/// @name Tick Processing Internals
	/// @{
private:
	struct Link {
		GameBase *next;
		GameBase *prev;
	};
	U32  mProcessTag;                      ///< Tag used to sort objects for processing.
	Link mProcessLink;                     ///< Ordered process queue link.
	SimObjectPtr<GameBase> mAfterObject;
	/// @}

	// Control interface
	void* mControllingClient;
	//GameBase* mControllingObject;

public:
	static bool gShowBoundingBox;    ///< Should we render bounding boxes?
protected:
	bool mProcessTick;
	F32  mLastDelta;
	F32 mCameraFov;

public:
	enum GameBaseMasks {
		InitialUpdateMask = Parent::NextFreeMask,
		DataBlockMask = InitialUpdateMask << 1,
		ExtendedInfoMask = DataBlockMask << 1,
		ControlMask = ExtendedInfoMask << 1,
		NextFreeMask = ControlMask << 1
	};

	/// Returns the datablock for this object.
	GameBaseData* getDataBlock() { return mDataBlock; }
};

struct Rigid {
	Point3F force;
	Point3F torque;

	Point3F linVelocity;          ///< Linear velocity
	Point3F linPosition;          ///< Current position
	Point3F linMomentum;          ///< Linear momentum
	Point3F angVelocity;          ///< Angular velocity
	QuatF   angPosition;          ///< Current rotation
	Point3F angMomentum;          ///< Angular momentum

	MatrixF objectInertia;        ///< Moment of inertia
	MatrixF invObjectInertia;     ///< Inverse moment of inertia
	MatrixF invWorldInertia;      ///< Inverse moment of inertia in world space

	Point3F centerOfMass;         ///< Center of mass in object space
	Point3F worldCenterOfMass;    ///< CofM in world space
	F32 mass;                     ///< Rigid body mass
	F32 oneOverMass;              ///< 1 / mass
	F32 restitution;              ///< Collision restitution
	F32 friction;                 ///< Friction coefficient
	bool atRest;

	F32 getZeroImpulse(const Point3F& r, const Point3F& normal);
	void getOriginVector(const Point3F &p, Point3F* r);
	void getVelocity(const Point3F& r, Point3F* v);

	bool resolveCollision(Point3F &p, Point3F normal);
	bool resolveCollision(Point3F& p, Point3F normal, Rigid* rigid);

	void applyImpulse(const Point3F &r, const Point3F &impulse);

	void updateVelocity();
};

/// Definition of some basic Sim system constants.
///
/// These constants define the range of ids assigned to datablocks
/// (DataBlockObjectIdFirst - DataBlockObjectIdLast), and the number
/// of bits used to store datablock IDs.
///
/// Normal Sim objects are given the range of IDs starting at
/// DynamicObjectIdFirst and going to infinity. Sim objects use
/// a SimObjectId to represent their ID; this is currently a U32.
///
/// The RootGroupId is assigned to gRootGroup, in which most SimObjects
/// are addded as child members. See simManager.cc for details, particularly
/// Sim::initRoot() and following.
enum SimObjectsConstants
{
	DataBlockObjectIdFirst = 3,
	DataBlockObjectIdBitSize = 10,
	DataBlockObjectIdLast = DataBlockObjectIdFirst + (1 << DataBlockObjectIdBitSize) - 1,

	DynamicObjectIdFirst = DataBlockObjectIdLast + 1,
	InvalidEventId = 0,
	RootGroupId = 0xFFFFFFFF,
};

//----------------------------------------------------------------------------

enum VehicleConsts
{
	VC_NUM_DUST_EMITTERS = 1,
	VC_NUM_DAMAGE_EMITTER_AREAS = 2,
	VC_NUM_DAMAGE_LEVELS = 2,
	VC_NUM_BUBBLE_EMITTERS = 1,
	VC_NUM_DAMAGE_EMITTERS = VC_NUM_DAMAGE_LEVELS + VC_NUM_BUBBLE_EMITTERS,
	VC_NUM_SPLASH_EMITTERS = 2,
	VC_BUBBLE_EMITTER = VC_NUM_DAMAGE_EMITTERS - VC_NUM_BUBBLE_EMITTERS,
};

enum MoveConstants {
	MaxTriggerKeys = 6,
	MaxMoveQueueSize = 45,
};

struct Move
{
	// packed storage rep, set in clamp
	S32 px, py, pz;
	U32 pyaw, ppitch, proll;
	F32 x, y, z;          // float -1 to 1
	F32 yaw, pitch, roll; // 0-2PI
	U32 id;               // sync'd between server & client - debugging tool.
	U32 sendCount;

	bool freeLook;
	bool trigger[MaxTriggerKeys];
};


//----------------------------------------------------------------------------

class ConvexFeature
{
public:
	struct Edge {
		S32 vertex[2];
	};
	struct Face {
		Point3F normal;
		S32 vertex[3];
	};

	void* mVertexList;
	void* mEdgeList;
	void* mFaceList;
	S32 material;
	SceneObject* object;
};


//----------------------------------------------------------------------------

enum ConvexType {
	TSConvexType,
	BoxConvexType,
	TerrainConvexType,
	InteriorConvexType,
	ShapeBaseConvexType,
	TSStaticConvexType,
	InteriorMapConvexType
};

//----------------------------------------------------------------------------

struct CollisionStateList;

struct CollisionState
{
	CollisionStateList* mLista;
	CollisionStateList* mListb;
	Convex* a;
	Convex* b;

	F32 dist;            // Current estimated distance
	Point3F v;           // Vector between closest points

						 //
	CollisionState();
	virtual ~CollisionState();
	virtual void swap();
	virtual void set(Convex* a, Convex* b, const MatrixF& a2w, const MatrixF& b2w);
	virtual F32 distance(const MatrixF& a2w, const MatrixF& b2w, const F32 dontCareDist,
		const MatrixF* w2a = NULL, const MatrixF* _w2b = NULL);
	void render();
};

struct CollisionStateList
{
	static CollisionStateList sFreeList;
	CollisionStateList* mNext;
	CollisionStateList* mPrev;
	CollisionState* mState;
};

//----------------------------------------------------------------------------


//----------------------------------------------------------------------------

struct CollisionWorkingList
{
	static CollisionWorkingList sFreeList;
	struct WLink {
		CollisionWorkingList* mNext;
		CollisionWorkingList* mPrev;
	} wLink;
	struct RLink {
		CollisionWorkingList* mNext;
		CollisionWorkingList* mPrev;
	} rLink;
	Convex* mConvex;
};

class Convex {

	/// @name Linked list managent
	/// @{
	Convex* mNext; ///< Next item in linked list of Convex's
	Convex* mPrev; ///< Previous item in linked list of Convex's

	U32 mTag;
	static U32 sTag;

protected:
	CollisionStateList   mList;            ///< Objects we're testing against
	CollisionWorkingList mWorking;         ///< Objects within our bounds
	CollisionWorkingList mReference;       ///< Other convex testing against us
	SceneObject* mObject;                  ///< Object this Convex is built around
	ConvexType mType;                      ///< Type of Convex this is @see ConvexType
};

//----------------------------------------------------------------------------

class ShapeBaseConvex : public Convex
{
	typedef Convex Parent;
	friend class ShapeBase;
	friend class Vehicle;

protected:
	ShapeBase*  pShapeBase;
	MatrixF*    nodeTransform;

public:
	MatrixF*    transform;
	U32         hullId;
	Box3F       box;
};

template <class T> class Resource
{
private:
	void *obj;  ///< Actual resource object
};

class ShapeBaseData : public GameBaseData {
public:
	/// Various constants relating to the ShapeBaseData
	enum Constants {
		NumMountPoints = 32,
		NumMountPointBits = 5,
		MaxCollisionShapes = 8,
		AIRepairNode = 31
	};

	StringTableEntry  shapeName;
	StringTableEntry  cloakTexName;

	/// @name Destruction
	///
	/// Everyone likes to blow things up!
	/// @{
	void *      debris;
	S32               debrisID;
	StringTableEntry  debrisShapeName;
	Resource<void> debrisShape;

	void*    explosion;
	S32               explosionID;

	void*    underwaterExplosion;
	S32               underwaterExplosionID;
	/// @}

	/// @name Physical Properties
	/// @{
	F32 mass;
	F32 drag;
	F32 density;
	F32 maxEnergy;
	F32 maxDamage;
	F32 repairRate;                  ///< Rate per tick.

	F32 disabledLevel;
	F32 destroyedLevel;

	S32 shieldEffectLifetimeMS;
	/// @}

	/// @name 3rd Person Camera
	/// @{
	F32 cameraMaxDist;               ///< Maximum distance from eye
	F32 cameraMinDist;               ///< Minumumistance from eye
									 /// @}

									 /// @name Camera FOV
									 ///
									 /// These are specified in degrees.
									 /// @{
	F32 cameraDefaultFov;            ///< Default FOV.
	F32 cameraMinFov;                ///< Min FOV allowed.
	F32 cameraMaxFov;                ///< Max FOV allowed.
									 /// @}

									 /// @name Data initialized on preload
									 /// @{

	Resource<void> shape;         ///< Shape handle
	U32 mCRC;
	bool computeCRC;

	S32 eyeNode;                         ///< Shape's eye node index
	S32 cameraNode;                      ///< Shape's camera node index
	S32 shadowNode;                      ///< Move shadow center as this node moves
	S32 mountPointNode[NumMountPoints];  ///< Node index of mountPoint
	S32 debrisDetail;                    ///< Detail level used to generate debris
	S32 damageSequence;                  ///< Damage level decals
	S32 hulkSequence;                    ///< Destroyed hulk

	bool              canControl;             // can this object be controlled?
	bool              canObserve;             // may look at object in commander map?
	bool              observeThroughObject;   // observe this object through its camera transform and default fov

											  /// @name HUD
											  ///
											  /// @note This may be only semi-functional.
											  /// @{

	enum {
		NumHudRenderImages = 8,
	};

	StringTableEntry  hudImageNameFriendly[NumHudRenderImages];
	StringTableEntry  hudImageNameEnemy[NumHudRenderImages];
	U32     hudImageFriendly[NumHudRenderImages];
	U32     hudImageEnemy[NumHudRenderImages];

	bool              hudRenderCenter[NumHudRenderImages];
	bool              hudRenderModulated[NumHudRenderImages];
	bool              hudRenderAlways[NumHudRenderImages];
	bool              hudRenderDistance[NumHudRenderImages];
	bool              hudRenderName[NumHudRenderImages];
	/// @}

	/// @name Collision Data
	/// @{
	Vector<S32>   collisionDetails;  ///< Detail level used to collide with.
									 ///
									 /// These are detail IDs, see TSShape::findDetail()
	Vector<Box3F> collisionBounds;   ///< Detail level bounding boxes.

	Vector<S32>   LOSDetails;        ///< Detail level used to perform line-of-sight queries against.
									 ///
									 /// These are detail IDs, see TSShape::findDetail()
									 /// @}

									 /// @name Shadow Settings
									 ///
									 /// These are set by derived classes, not by script file.
									 /// They control when shadows are rendered (and when generic shadows are substituted)
									 ///
									 /// @{

	F32 genericShadowLevel;
	F32 noShadowLevel;
	/// @}

	/// @name Misc. Settings
	/// @{
	bool emap;                       ///< Enable environment mapping?
	bool firstPersonOnly;            ///< Do we allow only first person view of this image?
	bool useEyePoint;                ///< Do we use this object's eye point to view from?
	bool aiAvoidThis;                ///< If set, the AI's will try to walk around this object...
									 ///
									 ///  @note There isn't really any AI code that can take advantage of this.
	bool isInvincible;               ///< If set, object cannot take damage (won't show up with damage bar either)
	bool renderWhenDestroyed;        ///< If set, will not render this object when destroyed.

	bool inheritEnergyFromMount;
};


//----------------------------------------------------------------------------

/// ShapeBase is the renderable shape from which most of the scriptable objects
/// are derived, including the player, vehicle and items classes.  ShapeBase
/// provides basic shape loading, audio channels, and animation as well as damage
/// (and damage states), energy, and the ability to mount images and objects.
///
/// @nosubgrouping
class ShapeBase : public GameBase
{
	typedef GameBase Parent;

public:
	enum PublicConstants {
		ThreadSequenceBits = 6,
		MaxSequenceIndex = (1 << ThreadSequenceBits) - 1,
		EnergyLevelBits = 5,
		DamageLevelBits = 6,
		DamageStateBits = 2,
		// The thread and image limits should not be changed without
		// also changing the ShapeBaseMasks enum values declared
		// further down.
		MaxSoundThreads = 4,            ///< Should be a power of 2
		MaxScriptThreads = 4,            ///< Should be a power of 2
		MaxMountedImages = 4,            ///< Should be a power of 2
		MaxImageEmitters = 3,
		NumImageBits = 3,
		ShieldNormalBits = 8,
		CollisionTimeoutValue = 250      ///< Timeout in ms.
	};

	/// This enum indexes into the sDamageStateName array
	enum DamageState {
		Enabled,
		Disabled,
		Destroyed,
		NumDamageStates,
		NumDamageStateBits = 2,   ///< Should be log2 of the number of states.
	};

private:
	ShapeBaseData*    mDataBlock;                ///< Datablock
												 //GameConnection*   mControllingClient;        ///< Controlling client
	ShapeBase*        mControllingObject;        ///< Controlling object
	bool              mTrigger[MaxTriggerKeys];  ///< What triggers are set, if any.

												 /// @name Scripted Sound
												 /// @{
	struct Sound {
		bool play;                    ///< Are we playing this sound?
		SimTime timeout;              ///< Time until we stop playing this sound.
		void* profile;        ///< Profile on server
		AUDIOHANDLE sound;            ///< Handle on client
	};
	Sound mSoundThread[MaxSoundThreads];
	/// @}

	/// @name Scripted Animation Threads
	/// @{

	struct Thread {
		/// State of the animation thread.
		enum State {
			Play, Stop, Pause
		};
		void* thread; ///< Pointer to 3space data.
		U32 state;        ///< State of the thread
						  ///
						  ///  @see Thread::State
		S32 sequence;     ///< The animation sequence which is running in this thread.
		U32 sound;        ///< Handle to sound.
		bool atEnd;       ///< Are we at the end of this thread?
		bool forward;     ///< Are we playing the thread forward? (else backwards)
	};
	Thread mScriptThread[MaxScriptThreads];

	/// @}

	/// @name Invincibility
	/// @{
	F32 mInvincibleCount;
	F32 mInvincibleTime;
	F32 mInvincibleSpeed;
	F32 mInvincibleDelta;
	F32 mInvincibleEffect;
	F32 mInvincibleFade;
	bool mInvincibleOn;
	/// @}

protected:
	/// @name Mounted Images
	/// @{

	/// An image mounted on a shapebase.
	struct MountedImage {
		void* dataBlock;
		void *state;
		void* nextImage;
		StringHandle skinNameHandle;
		StringHandle nextSkinNameHandle;

		/// @name State
		///
		/// Variables tracking the state machine
		/// representing this specific mounted image.
		/// @{

		bool loaded;                  ///< Is the image loaded?
		bool nextLoaded;              ///< Is the next state going to result in the image being loaded?
		F32 delayTime;                ///< Time till next state.
		U32 fireCount;                ///< Fire skip count.
									  ///
									  /// This is incremented every time the triggerDown bit is changed,
									  /// so that the engine won't be too confused if the player toggles the
									  /// trigger a bunch of times in a short period.
									  ///
									  /// @note The network deals with this variable at 3-bit precision, so it
									  /// can only range 0-7.
									  ///
									  /// @see ShapeBase::setImageState()

		bool triggerDown;             ///< Is the trigger down?
		bool ammo;                    ///< Do we have ammo?
									  ///
									  /// May be true based on either energy OR ammo.

		bool target;                  ///< Have we acquired a targer?
		bool wet;                     ///< Is the weapon wet?

									  /// @}

									  /// @name 3space
									  ///
									  /// Handles to threads and shapeinstances in the 3space system.
									  /// @{

									  ///
		void* shapeInstance;
		void *ambientThread;
		void *visThread;
		void *animThread;
		void *flashThread;
		void *spinThread;
		/// @}

		/// @name Effects
		///
		/// Variables relating to lights, sounds, and particles.
		/// @{


		SimTime lightStart;     ///< Starting time for light flashes.

		LightInfo   mLight;     ///< Passed to the LightManager; filled out with the above information.
								///
								///  registerLights() is responsible for performing
								///  the calculations that prepare mLight.

		bool animLoopingSound;  ///< Are we playing a looping sound?
		AUDIOHANDLE animSound;  ///< Handle to the current image sound.

								/// Represent the state of a specific particle emitter on the image.
		struct ImageEmitter {
			S32 node;
			F32 time;
			SimObjectPtr<void> emitter;
		};
		ImageEmitter emitter[MaxImageEmitters];
	};
	MountedImage mMountedImageList[MaxMountedImages];

	/// @}

	/// @name Render settings
	/// @{

	void* mShapeInstance;
	Convex *         mConvexList;
	void *         mShadow;
	bool             mGenerateShadow;
	StringHandle     mSkinNameHandle;

	StringHandle mShapeNameHandle;   ///< Name sent to client
									 /// @}

									 /// @name Physical Properties
									 /// @{

	F32 mEnergy;                     ///< Current enery level.
	F32 mRechargeRate;               ///< Energy recharge rate (in units/tick).
	bool mChargeEnergy;              ///< Are we currently charging?
									 /// @note Currently unused.

	F32 mMass;                       ///< Mass.
	F32 mOneOverMass;                ///< Inverse of mass.
									 /// @note This is used to optimize certain physics calculations.

									 /// @}

									 /// @name Physical Properties
									 ///
									 /// Properties for the current object, which are calculated
									 /// based on the container we are in.
									 ///
									 /// @see ShapeBase::updateContainer()
									 /// @see ShapeBase::mContainer
									 /// @{
	F32 mDrag;                       ///< Drag.
	F32 mBuoyancy;                   ///< Buoyancy factor.
	U32 mLiquidType;                 ///< Type of liquid (if any) we are in.
	F32 mLiquidHeight;               ///< Height of liquid around us (from 0..1).
	F32 mWaterCoverage;              ///< Percent of this object covered by water

	Point3F mAppliedForce;
	F32 mGravityMod;
	/// @}

	F32 mDamageFlash;
	F32 mWhiteOut;

	bool mFlipFadeVal;
	U32 mLightTime;

	/// Last shield direction (cur. unused)
	Point3F mShieldNormal;

	/// Mounted objects
	struct MountInfo {
		ShapeBase* list;              ///< Objects mounted on this object
		ShapeBase* object;            ///< Object this object is mounted on.
		ShapeBase* link;              ///< Link to next object mounted to this object's mount
		U32 node;                     ///< Node point we are mounted to.
	} mMount;

public:
	/// @name Collision Notification
	/// This is used to keep us from spamming collision notifications. When
	/// a collision occurs, we add to this list; then we don't notify anyone
	/// of the collision until the CollisionTimeout expires (which by default
	/// occurs in 1/10 of a second).
	///
	/// @see notifyCollision(), queueCollision()
	/// @{
	struct CollisionTimeout {
		CollisionTimeout* next;
		ShapeBase* object;
		U32 objectNumber;
		SimTime expireTime;
		Point3F vector;
	};
	CollisionTimeout* mTimeoutList;
	static CollisionTimeout* sFreeTimeoutList;

	void queueCollision(ShapeBase* obj, const Point3F& vec);
protected:

	/// @name Damage
	/// @{
	F32  mDamage;
	F32  mRepairRate;
	F32  mRepairReserve;
	bool mRepairDamage;
	DamageState mDamageState;
	void *mDamageThread;
	void *mHulkThread;
	Point3F damageDir;
	bool blowApart;
	/// @}

	/// @name Cloaking
	/// @{
	bool mCloaked;
	F32  mCloakLevel;
	TextureHandle mCloakTexture;
	bool mHidden; ///< in/out of world

				  /// @}

				  /// @name Fading
				  /// @{
	bool  mFadeOut;
	bool  mFading;
	F32   mFadeVal;
	F32   mFadeElapsedTime;
	F32   mFadeTime;
	F32   mFadeDelay;
	/// @}


	/// @name Control info
	/// @{
	F32  mCameraFov;           ///< Camera FOV(in degrees)
	bool mIsControlled;        ///< Client side controlled flag

							   /// @}

	static U32 sLastRenderFrame;
	U32 mLastRenderFrame;
	F32 mLastRenderDistance;
	U32 mSkinHash;
public:
	/// @name Network state masks
	/// @{

	///
	enum ShapeBaseMasks {
		NameMask,
		DamageMask,
		NoWarpMask,
		MountedMask,
		CloakMask,
		ShieldMask,
		InvincibleMask,
		SkinMask,
		SoundMaskN,       ///< Extends + MaxSoundThreads bits
		ThreadMaskN,
		ImageMaskN,
		NextFreeMask
	};

	enum BaseMaskConstants {
		SoundMask,
		ThreadMask,
		ImageMask
	};

	/// @}

	static bool gRenderEnvMaps; ///< Global flag which turns on or off all environment maps
	static F32  sWhiteoutDec;
	static F32  sDamageFlashDec;
};

struct VehicleData : public ShapeBaseData {
	struct Body {
		enum Sounds {
			SoftImpactSound,
			HardImpactSound,
			MaxSounds,
		};
		void* sound[MaxSounds];
		F32 restitution;
		F32 friction;
	} body;

	enum VehicleConsts
	{
		VC_NUM_DUST_EMITTERS = 1,
		VC_NUM_DAMAGE_EMITTER_AREAS = 2,
		VC_NUM_DAMAGE_LEVELS = 2,
		VC_NUM_BUBBLE_EMITTERS = 1,
		VC_NUM_DAMAGE_EMITTERS = VC_NUM_DAMAGE_LEVELS + VC_NUM_BUBBLE_EMITTERS,
		VC_NUM_SPLASH_EMITTERS = 2,
		VC_BUBBLE_EMITTER = VC_NUM_DAMAGE_EMITTERS - VC_NUM_BUBBLE_EMITTERS,
	};

	enum Sounds {
		ExitWater,
		ImpactSoft,
		ImpactMedium,
		ImpactHard,
		Wake,
		MaxSounds
	};
	void* waterSound[MaxSounds];
	F32 exitSplashSoundVel;
	F32 softSplashSoundVel;
	F32 medSplashSoundVel;
	F32 hardSplashSoundVel;

	F32 minImpactSpeed;
	F32 softImpactSpeed;
	F32 hardImpactSpeed;
	F32 minRollSpeed;
	F32 maxSteeringAngle;

	F32 collDamageThresholdVel;
	F32 collDamageMultiplier;

	bool cameraRoll;           ///< Roll the 3rd party camera
	F32 cameraLag;             ///< Amount of camera lag (lag += car velocity * lag)
	F32 cameraDecay;           ///< Rate at which camera returns to target pos.
	F32 cameraOffset;          ///< Vertical offset

	F32 minDrag;
	F32 maxDrag;
	S32 integration;           ///< # of physics steps per tick
	F32 collisionTol;          ///< Collision distance tolerance
	F32 contactTol;            ///< Contact velocity tolerance
	Point3F massCenter;        ///< Center of mass for rigid body
	Point3F massBox;           ///< Size of inertial box

	F32 jetForce;
	F32 jetEnergyDrain;        ///< Energy drain/tick
	F32 minJetEnergy;

	void* dustEmitter;
	S32 dustID;
	F32 triggerDustHeight;  ///< height vehicle has to be under to kick up dust
	F32 dustHeight;         ///< dust height above ground

	void*   damageEmitterList[VC_NUM_DAMAGE_EMITTERS];
	Point3F damageEmitterOffset[VC_NUM_DAMAGE_EMITTER_AREAS];
	S32 damageEmitterIDList[VC_NUM_DAMAGE_EMITTERS];
	F32 damageLevelTolerance[VC_NUM_DAMAGE_LEVELS];
	F32 numDmgEmitterAreas;

	void* splashEmitterList[VC_NUM_SPLASH_EMITTERS];
	S32 splashEmitterIDList[VC_NUM_SPLASH_EMITTERS];
	F32 splashFreqMod;
	F32 splashVelEpsilon;
};

struct Collision
{
	SceneObject* object;
	Point3F point;
	Point3F normal;
	U32 material;

	// Face and Face dot are currently only set by the extrudedPolyList
	// clipper.  Values are otherwise undefined.
	U32 face;                  // Which face was hit
	F32 faceDot;               // -Dot of face with poly normal
	F32 distance;
};

struct CollisionList
{
	enum {
		MaxCollisions = 64
	};
	int count;
	Collision collision[MaxCollisions];
	F32 t;
	// MaxHeight is currently only set by the extrudedPolyList
	// clipper.  It represents the maximum vertex z value of
	// the returned collision surfaces.
	F32 maxHeight;
};

class Vehicle : public ShapeBase
{
	typedef ShapeBase Parent;

protected:
	enum CollisionFaceFlags {
		BodyCollision = 0x1,
		WheelCollision = 0x2,
	};
	enum MaskBits {
		PositionMask = Parent::NextFreeMask << 0,
		EnergyMask = Parent::NextFreeMask << 1,
		NextFreeMask = Parent::NextFreeMask << 2
	};

	struct StateDelta {
		Move move;                    ///< Last move from server
		F32 dt;                       ///< Last interpolation time
									  // Interpolation data
		Point3F pos;
		Point3F posVec;
		QuatF rot[2];
		// Warp data
		S32 warpTicks;                ///< Number of ticks to warp
		S32 warpCount;                ///< Current pos in warp
		Point3F warpOffset;
		QuatF warpRot[2];
		//
		Point3F cameraOffset;
		Point3F cameraVec;
		Point3F cameraRot;
		Point3F cameraRotVec;
	};

	StateDelta mDelta;
	S32 mPredictionCount;            ///< Number of ticks to predict
	VehicleData* mDataBlock;
	bool inLiquid;
	AUDIOHANDLE waterWakeHandle;

	Point3F mCameraOffset; ///< 3rd person camera

						   // Control
	Point2F mSteering;
	F32 mThrottle;
	bool mJetting;

	// Rigid Body
	bool mDisableMove;

	CollisionList mCollisionList;
	CollisionList mContacts;
	ShapeBaseConvex mConvex;
	int restCount;

	void *mDustEmitterList[VehicleData::VC_NUM_DUST_EMITTERS];
	void *mDamageEmitterList[VehicleData::VC_NUM_DAMAGE_EMITTERS];
	void *mSplashEmitterList[VehicleData::VC_NUM_SPLASH_EMITTERS];


public:
	Rigid mRigid;

	// Test code...
	static void* sPolyList;
	static S32 sVehicleCount;
};

inline void mCross(const Point3F &a, const Point3F &b, Point3F *res)
{
	res->x = (a.y * b.z) - (a.z * b.y);
	res->y = (a.z * b.x) - (a.x * b.z);
	res->z = (a.x * b.y) - (a.y * b.x);
}

inline F32 mDot(const Point3F &p1, const Point3F &p2)
{
	return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
}

inline F32 mFabs(const F32 val)
{
	return (F32)fabs(val);
}

enum SimObjectTypes
{
	/// @name Types used by the SceneObject class
	/// @{
	DefaultObjectType = 0,
	StaticObjectType = BIT(0),
	/// @}

	/// @name Basic Engine Types
	/// @{

	///
	EnvironmentObjectType = BIT(1),
	TerrainObjectType = BIT(2),
	InteriorObjectType = BIT(3),
	WaterObjectType = BIT(4),
	TriggerObjectType = BIT(5),
	MarkerObjectType = BIT(6),
	AtlasObjectType = BIT(7), // Buy TSE.
	InteriorMapObjectType = BIT(8),
	DecalManagerObjectType = BIT(9),
	/// @}

	/// @name Game Types
	/// @{
	GameBaseObjectType = BIT(10),
	ShapeBaseObjectType = BIT(11),
	CameraObjectType = BIT(12),
	StaticShapeObjectType = BIT(13),
	PlayerObjectType = BIT(14),
	ItemObjectType = BIT(15),
	VehicleObjectType = BIT(16),
	VehicleBlockerObjectType = BIT(17),
	ProjectileObjectType = BIT(18),
	ExplosionObjectType = BIT(19),
	CorpseObjectType = BIT(20),
	DebrisObjectType = BIT(22),
	PhysicalZoneObjectType = BIT(23),
	StaticTSObjectType = BIT(24),
	AIObjectType = BIT(25),
	StaticRenderedObjectType = BIT(26),
	/// @}

	/// @name Other
	/// The following are allowed types that can be set on datablocks for static shapes
	/// @{
	DamagableItemObjectType = BIT(27),
	/// @}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public functions

//Scan the module for a pattern
DWORD ScanFunc(char* pattern, char* mask);

//Change a byte at a specific location in memory
void PatchByte(BYTE* location, BYTE value);

//Register a torquescript function that returns a string. The function must look like this:
//const char* func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, StringCallback callBack, const char* usage, int minArgs, int maxArgs);

//Register a torquescript function that returns an int. The function must look like this:
//int func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, IntCallback callBack, const char* usage, int minArgs, int maxArgs);

//Register a torquescript function that returns a float. The function must look like this:
//float func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, FloatCallback callBack, const char* usage, int minArgs, int maxArgs);

//Register a torquescript function that returns nothing. The function must look like this:
//void func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, VoidCallback callBack, const char* usage, int minArgs, int maxArgs);

//Register a torquescript function that returns a bool. The function must look like this:
//bool func(DWORD* obj, int argc, const char* argv[])
void ConsoleFunction(const char* nameSpace, const char* name, BoolCallback callBack, const char* usage, int minArgs, int maxArgs);

//Expose an integer variable to torquescript
void ConsoleVariable(const char* name, int* data);

//Expose a boolean variable to torquescript
void ConsoleVariable(const char* name, bool* data);

//Expose a float variable to torquescript
void ConsoleVariable(const char* name, float* data);

//Expose a string variable to torquescript
void ConsoleVariable(const char* name, char* data);

//Evaluate a torquescript string in global scope
const char* Eval(const char* str);

//Initialize the Torque Interface
bool InitTorqueStuff();

