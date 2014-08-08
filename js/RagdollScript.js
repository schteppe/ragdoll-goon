define([
	'goo/math/Vector3',
	'goo/math/Matrix4x4',
	'goo/math/Matrix3x3',
	'goo/math/Quaternion',
	'goo/math/Transform',
	'goo/math/MathUtils',
	'goo/scripts/Scripts',
	'goo/scripts/ScriptUtils',
	'goo/renderer/Renderer',
	'goo/entities/SystemBus',
	'goo/animation/clip/JointData',
	'goo/shapes/Box',
	'goo/renderer/Material',
	'goo/renderer/shaders/ShaderLib',
	'goo/entities/components/MeshDataComponent',
	'goo/entities/components/MeshRendererComponent',
	'goo/animation/Joint',
], function (
	Vector3,
	Matrix4x4,
	Matrix3x3,
	Quaternion,
	Transform,
	MathUtils,
	Scripts,
	ScriptUtils,
	Renderer,
	SystemBus,
	JointData,
	Box,
	Material,
	ShaderLib,
	MeshDataComponent,
	MeshRendererComponent,
	Joint
) {
	'use strict';

	Matrix4x4.prototype.getRotation = function (store) {
		var d = this.data;
		store.set(
			d[0], d[1], d[2],
			d[4], d[5], d[6],
			d[8], d[9], d[10]
		);
		return this;
	};

	var renderBones = true;
	var BODYPART = 1;
	var SCENE = 2;

	var ammoTransform = new Ammo.btTransform();
	var gooQuaternion = new Quaternion();

	function RagdollScript() {
		var physicsWorld;
		var fixedTime = 1/60;
		var maxSubSteps = 3;
		var paused = false;

		window.addEventListener('keydown', function(event){
			switch(event.keyCode){

			}
			paused = !paused;
		});

		function setup(params, env) {
			env.syncEntities = [];
			env.syncBodies = [];

			var clipSource = params.clipSource;
			var joints = params.joints;
			var entity = params.entity;
			var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
			var dispatcher = new Ammo.btCollisionDispatcher( collisionConfiguration );
			var overlappingPairCache = new Ammo.btDbvtBroadphase();
			var solver = new Ammo.btSequentialImpulseConstraintSolver();
			physicsWorld = new Ammo.btDiscreteDynamicsWorld( dispatcher, overlappingPairCache, solver, collisionConfiguration);
			physicsWorld.setGravity(new Ammo.btVector3(0, -10, 0));

			env.ragdoll = createRagdollFromSkeleton(params, env, clipSource, joints, entity, physicsWorld);
		}

		function cleanup(params, env) {
			for(var i=0; i<bodies.length; i++){
				physicsWorld.removeRigidBody(bodies[i]);
			}
			physicsWorld = null;
		}

		var a = false;
		function update(params, env) {
			if(a) return;
			//a = true;
			//console.log("updateeee")

			if(!paused){
				physicsWorld.stepSimulation(env.world.tpf, maxSubSteps, fixedTime);
			}

			for(var i=0; i < env.syncBodies.length; i++){
				copyPhysicalTransformToVisual(env.syncBodies[i], env.syncEntities[i]);
			}

			// Loop over all joints
			var world = env.world;

			// Get the world transform of each of the joints
			var entity = env.entity;
			var data = entity.animationComponent.getCurrentSourceData();
			var pose = entity.animationComponent._skeletonPose;
			var skeleton = pose._skeleton;
			var clipSource = params.clipSource;

			// Update the skeleton with rigidbody transforms
			for (var i = 0; i < skeleton._joints.length; i++) {
				var joint = skeleton._joints[i];
				var parentIndex = joint._parentIndex;
				var body = joint.body;

				var globalTransform = pose._globalTransforms[i];
				var localTransform = pose._localTransforms[i];
				var parentTransform;
				var globalParentTransform;

				if (parentIndex !== Joint.NO_PARENT) {
					var parentJoint = skeleton._joints[parentIndex];
					var parentBody = parentJoint.body;

					parentTransform = pose._localTransforms[parentIndex];
					globalParentTransform = pose._globalTransforms[parentIndex];

					// Get the ammo orientation, which is the world orientation
					body.getMotionState().getWorldTransform(ammoTransform);
					var ammoQuat = ammoTransform.getRotation();
					var ammoOrigin = ammoTransform.getOrigin();

					// Construct a transform from the ammo data
					var ammoRotation = new Matrix3x3();
					var globalQuat = new Quaternion(ammoQuat.x(), ammoQuat.y(), ammoQuat.z(), ammoQuat.w());
					ammoRotation.copyQuaternion(globalQuat);

					// Get the local transform of the body in the parent frame
					var newLocalRotation = new Matrix3x3();
					var globalParentRotation = new Matrix3x3();
					globalParentTransform.matrix.getRotation(globalParentRotation);
					Matrix3x3.combine(globalParentRotation.invert(), ammoRotation, newLocalRotation);

					var newLocalQuaternion = new Quaternion();
					newLocalQuaternion.fromRotationMatrix(newLocalRotation);

					//if(joint._name.indexOf('head') !== -1)
					//	console.log(joint._name, newLocalQuaternion.data, globalQuat.data);

					clipSource.setRotation(joint._name, newLocalQuaternion);

				} else {
					// No parent
					// This is a root joint. We use the spine body and update its stuff
					globalParentTransform = new Transform();
					globalParentTransform.matrix.setTranslation(params.offset);
					globalTransform = pose._localTransforms[i];

					var spineTranslation = getGooTranslationFromBody(env.spineBody);
					var spineQuaternion = getGooQuaternionFromBody(env.spineBody);
					clipSource.setTranslation(joint._name, spineTranslation);
					clipSource.setRotation(joint._name, spineQuaternion);

				}
			}

		}

		return {
			setup: setup,
			update: update,
			cleanup: cleanup
		};
	}

	function copyPhysicalTransformToVisual(body, entity) {
		var tc = entity.transformComponent;
		body.getMotionState().getWorldTransform(ammoTransform);
		var ammoQuat = ammoTransform.getRotation();
		gooQuaternion.setd(ammoQuat.x(), ammoQuat.y(), ammoQuat.z(), ammoQuat.w());
		tc.transform.rotation.copyQuaternion(gooQuaternion);
		var origin = ammoTransform.getOrigin();
		tc.setTranslation(origin.x(), origin.y(), origin.z());
	}

	var calcTrans = new Transform();
	var calcTrans1 = new Transform();
	var calcVec = new Vector3();
	var calcVec2 = new Vector3();
	var calcQuat = new Quaternion();
	var center = new Vector3();

	function createRagdollFromSkeleton(params, env, clipSource, joints, entity, physicsWorld){
		// Loop over all joints
		var world = env.world;

		// Get the world transform of each of the joints
		var data = entity.animationComponent.getCurrentSourceData();
		var pose = entity.animationComponent._skeletonPose;
		var skeleton = pose._skeleton;

		var joints = skeleton._joints;
		var positions = [], weights = [], jointIds = [],
			indices = [], count = 0, td = calcTrans.matrix.data;

		var material = new Material('DebugMat', ShaderLib.uber);

		// Create spine
		var spineBody = env.spineBody = createLimbBody(params, env, physicsWorld, new Vector3(0,100,0), 60, new Quaternion(), material, 5);

		// Set up initial bodies
		for ( var i = 0; i < skeleton._joints.length; i++) {
			var parentIndex = skeleton._joints[i]._parentIndex;

			// Child
			var globalTransform = pose._globalTransforms[i];
			var localTransform = pose._localTransforms[i];
			var parentTransform;
			var globalParentTransform;

			if (parentIndex !== Joint.NO_PARENT) {
				// Parent
				parentTransform = pose._localTransforms[parentIndex];
				globalParentTransform = pose._globalTransforms[parentIndex];
			} else {
				// No parent so just set global to the local transform
				globalParentTransform = new Transform();
				globalParentTransform.matrix.setTranslation(params.offset);
				globalTransform = pose._localTransforms[i];
			}

			globalTransform.matrix.getTranslation(calcVec);
			globalParentTransform.matrix.getTranslation(calcVec2);

			var length = Vector3.distance(calcVec,calcVec2);
			Vector3.add(calcVec, calcVec2, center);
			Vector3.mul(center, 0.5, center);


			var quat = new Quaternion();//calcQuat.fromVectorToVector(new Vector3(0,1,0), Vector3.sub(center, calcVec));

			// Get orientation of bone
			var mat = new Matrix3x3();
			globalTransform.matrix.getRotation(mat);
			quat.fromRotationMatrix(mat);

			if(skeleton._joints[i]._name.indexOf('head') !== -1){
				console.log(skeleton._joints[i]._name, quat.data);
			}

			// Store body
			skeleton._joints[i].body = createLimbBody(params, env, physicsWorld, center, length, quat, material, 5);
			skeleton._joints[i].length = length;
		}

		// Create constraints
		for ( var i = 0; false && i < skeleton._joints.length; i++) {
			var joint = skeleton._joints[i];
			var parentIndex = joint._parentIndex;
			var parentJoint = skeleton._joints[parentIndex];

			var localA = new Ammo.btTransform();
			var localB = new Ammo.btTransform();
			localA.setIdentity();
			localB.setIdentity();
			var body = joint.body;

			if (parentIndex !== Joint.NO_PARENT) {
				var parentBody = parentJoint.body;

				// Has parent

				// TODO: get the angle between the bodies, set limit accordingly
				//localA.getBasis().setEulerZYX(0,0,Math.PI);
				localA.setOrigin(new Ammo.btVector3(0, -parentJoint.length/2, 0)); // constrain to ends

				var invParentAmmoTransform = getGooTransformFromBody(parentJoint.body).invert();
				var childAmmoTransform = getGooTransformFromBody(joint.body);

				var resultAmmoTrans = Transform.combine(invParentAmmoTransform, childAmmoTransform);
				var resultGooQuat = new Quaternion();
				resultGooQuat.fromRotationMatrix(resultAmmoTrans.rotation);
				var resultAmmoQuat = new Ammo.btQuaternion();

				localA.setRotation(resultAmmoQuat); // Should be set to the quat from parent joint to child
				//localB.getBasis().setEulerZYX(0,0,Math.PI/2);
				localB.setOrigin(new Ammo.btVector3(0, joint.length/2, 0));
				localB.setRotation(new Ammo.btQuaternion(0,0,0,1));
				var coneC = new Ammo.btConeTwistConstraint(parentBody, body, localA, localB);
				//coneC.setLimit(0.1,0.1,0.1);
				physicsWorld.addConstraint(coneC, true);

			} else {
				// No parent so just constrain to the spine
				localA.setOrigin(new Ammo.btVector3(0, -joint.length/2, 0));
				localA.setRotation(new Ammo.btQuaternion(0,0,0,1));
				localB.setOrigin(new Ammo.btVector3(0, joint.length/2, 0));
				localB.setRotation(new Ammo.btQuaternion(0,0,0,1));
				var coneC = new Ammo.btConeTwistConstraint(spineBody, body, localA, localB);
				//coneC.setLimit(0,0,0);
				physicsWorld.addConstraint(coneC, true);
			}
		}

		// Create floor
		createBoxBody(params, env, physicsWorld, new Vector3(100,1,100), new Vector3(0,-10,0), material);
	}

	function getGooTransformFromBody(body, store){
		store = store || new Transform();
		body.getMotionState().getWorldTransform(ammoTransform);
		var ammoQuat = ammoTransform.getRotation();
		var ammoTranslation = ammoTransform.getOrigin();
		store.rotation.copyQuaternion(new Quaternion(ammoQuat.x(),ammoQuat.y(),ammoQuat.z(),ammoQuat.w()));
		store.translation.setd(ammoTranslation.x(),ammoTranslation.y(),ammoTranslation.z());
		store.update();
		return store;
	}

	function getGooQuaternionFromBody(body, store){
		store = store || new Quaternion();
		body.getMotionState().getWorldTransform(ammoTransform);
		var ammoQuat = ammoTransform.getRotation();
		store.setd(ammoQuat.x(),ammoQuat.y(),ammoQuat.z(),ammoQuat.w());
		return store;
	}

	function getGooTranslationFromBody(body, store){
		store = store || new Vector3();
		body.getMotionState().getWorldTransform(ammoTransform);
		var ammoTranslation = ammoTransform.getOrigin();
		store.setd(ammoTranslation.x(),ammoTranslation.y(),ammoTranslation.z());
		return store;
	}

	function createLimbBody(params, env, physicsWorld, center, length, quat, material, thickness){

		// Create mesh
		var jointEntity = env.world.createEntity(center);
		if(renderBones){
			jointEntity.setComponent(new MeshDataComponent(new Box(length,thickness,thickness)));
			jointEntity.setComponent(new MeshRendererComponent(material));
		}
		jointEntity.addToWorld();
		jointEntity.transformComponent.transform.rotation.copyQuaternion(quat);
		jointEntity.transformComponent.setUpdated();

		var transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3(center.x, center.y, center.z));
		//transform.op_mul(offset);
		transform.setRotation(new Ammo.btQuaternion(quat.x,quat.y,quat.z,quat.w));
		var shape = new Ammo.btCapsuleShape(thickness, length);

		var body = createRigidBody(physicsWorld, 10, transform, shape, {
			collisionGroup: BODYPART,
			collidesWith: SCENE
		});

		env.syncEntities.push(jointEntity);
		env.syncBodies.push(body);

		return body;
	}

	function createBoxBody(params, env, physicsWorld, extents, position, material){

		// Create mesh
		var boxEntity = env.world.createEntity(position);
		boxEntity.setComponent(new MeshDataComponent(new Box(2*extents.x, 2*extents.y, 2*extents.z)));
		boxEntity.setComponent(new MeshRendererComponent(material));
		boxEntity.addToWorld();
		//boxEntity.transformComponent.transform.rotation.copyQuaternion(quat);
		boxEntity.transformComponent.setUpdated();

		var shape = new Ammo.btBoxShape(new Ammo.btVector3(extents.x, extents.y, extents.z));
		var transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3(position.x, position.y, position.z));
		//transform.op_mul(offset);
		//transform.setRotation(new Ammo.btQuaternion(quat.x,quat.y,quat.z,quat.w));

		var body = createRigidBody(physicsWorld, 0, transform, shape, {
			collisionGroup: SCENE,
			collidesWith: BODYPART
		});

		env.syncEntities.push(boxEntity);
		env.syncBodies.push(body);

	}

	function createRagdoll(world, positionOffset){
		var BODYPART_PELVIS=0;
		var BODYPART_SPINE=1;
		var BODYPART_HEAD=2;
		var BODYPART_LEFT_UPPER_LEG=3;
		var BODYPART_LEFT_LOWER_LEG=4;
		var BODYPART_RIGHT_UPPER_LEG=5;
		var BODYPART_RIGHT_LOWER_LEG=6;
		var BODYPART_LEFT_UPPER_ARM=7;
		var BODYPART_LEFT_LOWER_ARM=8;
		var BODYPART_RIGHT_UPPER_ARM=9;
		var BODYPART_RIGHT_LOWER_ARM=10;
		var BODYPART_COUNT=11;

		var JOINT_PELVIS_SPINE=0;
		var JOINT_SPINE_HEAD=1;
		var JOINT_LEFT_HIP=2;
		var JOINT_LEFT_KNEE=3;
		var JOINT_RIGHT_HIP=4;
		var JOINT_RIGHT_KNEE=5;
		var JOINT_LEFT_SHOULDER=6;
		var JOINT_LEFT_ELBOW=7;
		var JOINT_RIGHT_SHOULDER=8;
		var JOINT_RIGHT_ELBOW=9;
		var JOINT_COUNT=10;

		var shapes = [];
		var bodies = [];
		var joints = [];

		// Setup the geometry
		shapes[BODYPART_PELVIS] = new Ammo.btCapsuleShape((0.15), (0.20));
		shapes[BODYPART_SPINE] = new Ammo.btCapsuleShape((0.15), (0.28));
		shapes[BODYPART_HEAD] = new Ammo.btCapsuleShape((0.10), (0.05));
		shapes[BODYPART_LEFT_UPPER_LEG] = new Ammo.btCapsuleShape((0.07), (0.45));
		shapes[BODYPART_LEFT_LOWER_LEG] = new Ammo.btCapsuleShape((0.05), (0.37));
		shapes[BODYPART_RIGHT_UPPER_LEG] = new Ammo.btCapsuleShape((0.07), (0.45));
		shapes[BODYPART_RIGHT_LOWER_LEG] = new Ammo.btCapsuleShape((0.05), (0.37));
		shapes[BODYPART_LEFT_UPPER_ARM] = new Ammo.btCapsuleShape((0.05), (0.33));
		shapes[BODYPART_LEFT_LOWER_ARM] = new Ammo.btCapsuleShape((0.04), (0.25));
		shapes[BODYPART_RIGHT_UPPER_ARM] = new Ammo.btCapsuleShape((0.05), (0.33));
		shapes[BODYPART_RIGHT_LOWER_ARM] = new Ammo.btCapsuleShape((0.04), (0.25));

		// Setup all the rigid bodies
		var offset = new Ammo.btTransform();
		offset.setIdentity();
		offset.setOrigin(positionOffset);

		var transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.), (1.), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_PELVIS] = createRigidBody(world, (1.),transform,shapes[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.), (1.2), (0.)));
		transform.op_mul(offset)
		bodies[BODYPART_SPINE] = createRigidBody(world, (1.), transform, shapes[BODYPART_SPINE]);
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.), (1.6), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_HEAD] = createRigidBody(world, (1.), transform, shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((-0.18), (0.65), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_LEFT_UPPER_LEG] = createRigidBody(world, (1.), transform, shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((-0.18), (0.2), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_LEFT_LOWER_LEG] = createRigidBody(world, (1.), transform, shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.18), (0.65), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_RIGHT_UPPER_LEG] = createRigidBody(world, (1.), transform, shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.18), (0.2), (0.)));
		transform.op_mul(offset);
		bodies[BODYPART_RIGHT_LOWER_LEG] = createRigidBody(world, (1.), transform, shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((-0.35), (1.45), (0.)));
		transform.getBasis().setEulerZYX(0,0,Math.PI/2);
		transform.op_mul(offset);
		bodies[BODYPART_LEFT_UPPER_ARM] = createRigidBody(world, (1.), transform, shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((-0.7), (1.45), (0.)));
		transform.getBasis().setEulerZYX(0,0,Math.PI/2);
		transform.op_mul(offset);
		bodies[BODYPART_LEFT_LOWER_ARM] = createRigidBody(world, (1.), transform, shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.35), (1.45), (0.)));
		transform.getBasis().setEulerZYX(0,0,-Math.PI/2);
		transform.op_mul(offset);
		bodies[BODYPART_RIGHT_UPPER_ARM] = createRigidBody(world, (1.), transform, shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3((0.7), (1.45), (0.)));
		transform.getBasis().setEulerZYX(0,0,-Math.PI/2);
		transform.op_mul(offset);
		bodies[BODYPART_RIGHT_LOWER_ARM] = createRigidBody(world, (1.), transform, shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the bodies
		for(var i=0; i<BODYPART_COUNT; i++){
			bodies[i].setDamping(0.05, 0.85);
			bodies[i].setDeactivationTime(0.8);
			bodies[i].setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		//var hingeC = new Ammo.btHingeConstraint();
		//var coneC = new Ammo.btConeTwistConstraint();

		var localA = new Ammo.btTransform();
		var localB = new Ammo.btTransform();
		/*
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (0.15), (0.)));
		localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (-0.15), (0.)));
		var hingeC = new Ammo.btHingeConstraint(bodies[BODYPART_PELVIS], bodies[BODYPART_SPINE], localA, localB);
		hingeC.setLimit((-Math.PI/4), (Math.PI/2));
		*/

		var hingeC = new Ammo.btHingeConstraint(bodies[BODYPART_PELVIS],
							 bodies[BODYPART_SPINE],
							 new Ammo.btVector3(0,0.15,0),
							 new Ammo.btVector3(0,-0.15,0),
							 new Ammo.btVector3(0,0,0.15),
							 new Ammo.btVector3(0,0,0.15));
		joints[JOINT_PELVIS_SPINE] = hingeC;

		world.addConstraint(joints[JOINT_PELVIS_SPINE], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,Math.PI/2); localA.setOrigin(new Ammo.btVector3((0.), (0.30), (0.)));
		localB.getBasis().setEulerZYX(0,0,Math.PI/2); localB.setOrigin(new Ammo.btVector3((0.), (-0.14), (0.)));
		var coneC = new Ammo.btConeTwistConstraint(bodies[BODYPART_SPINE], bodies[BODYPART_HEAD], localA, localB);
		coneC.setLimit(Math.PI/4, Math.PI/4, Math.PI/2);
		joints[JOINT_SPINE_HEAD] = coneC;

		world.addConstraint(joints[JOINT_SPINE_HEAD], true);

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-Math.PI/4*5);
		localA.setOrigin(new Ammo.btVector3((-0.18), (-0.10), (0.)));
		localB.getBasis().setEulerZYX(0,0,-Math.PI/4*5);
		localB.setOrigin(new Ammo.btVector3((0.), (0.225), (0.)));
		coneC = new Ammo.btConeTwistConstraint(bodies[BODYPART_PELVIS], bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC.setLimit(Math.PI/4, Math.PI/4, 0);
		joints[JOINT_LEFT_HIP] = coneC;

		world.addConstraint(joints[JOINT_LEFT_HIP], true);

		/*localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (-0.225), (0.)));
		localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (0.185), (0.)));
		hingeC =  new Ammo.btHingeConstraint(bodies[BODYPART_LEFT_UPPER_LEG], bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC.setLimit((0), (Math.PI/2));
		*/
		hingeC = new Ammo.btHingeConstraint(
			bodies[BODYPART_LEFT_UPPER_LEG],
			bodies[BODYPART_LEFT_LOWER_LEG],
			new Ammo.btVector3(0,-0.225,0),
			new Ammo.btVector3(0,0.185,0),
			new Ammo.btVector3(0,0,0.225),
			new Ammo.btVector3(0,0,0.185)
		);

		joints[JOINT_LEFT_KNEE] = hingeC;

		world.addConstraint(joints[JOINT_LEFT_KNEE], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,Math.PI/4); localA.setOrigin(new Ammo.btVector3((0.18), (-0.10), (0.)));
		localB.getBasis().setEulerZYX(0,0,Math.PI/4); localB.setOrigin(new Ammo.btVector3((0.), (0.225), (0.)));
		coneC = new Ammo.btConeTwistConstraint(bodies[BODYPART_PELVIS], bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC.setLimit(Math.PI/4, Math.PI/4, 0);
		joints[JOINT_RIGHT_HIP] = coneC;

		world.addConstraint(joints[JOINT_RIGHT_HIP], true);

		/*
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (-0.225), (0.)));
		localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (0.185), (0.)));
		hingeC =  new Ammo.btHingeConstraint(bodies[BODYPART_RIGHT_UPPER_LEG], bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC.setLimit((0), (Math.PI/2));
		*/
		hingeC =  new Ammo.btHingeConstraint(
			bodies[BODYPART_RIGHT_UPPER_LEG],
			bodies[BODYPART_RIGHT_LOWER_LEG],
			new Ammo.btVector3(0,-0.225,0),
			new Ammo.btVector3(0,0.185,0),
			new Ammo.btVector3(0,0,0.225),
			new Ammo.btVector3(0,0,0.185)
		);

		joints[JOINT_RIGHT_KNEE] = hingeC;

		world.addConstraint(joints[JOINT_RIGHT_KNEE], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,Math.PI);
		localA.setOrigin(new Ammo.btVector3((-0.2), (0.15), (0.)));
		localB.getBasis().setEulerZYX(0,0,Math.PI/2);
		localB.setOrigin(new Ammo.btVector3((0.), (-0.18), (0.)));
		coneC = new Ammo.btConeTwistConstraint(bodies[BODYPART_SPINE], bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC.setLimit(Math.PI/2, Math.PI/2, 0);

		joints[JOINT_LEFT_SHOULDER] = coneC;
		world.addConstraint(joints[JOINT_LEFT_SHOULDER], true);

		/*
			localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (0.18), (0.)));
		localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (-0.14), (0.)));
		hingeC =  new Ammo.btHingeConstraint(bodies[BODYPART_LEFT_UPPER_ARM],
								 bodies[BODYPART_LEFT_LOWER_ARM],
								 localA, localB);
		//		hingeC.setLimit((-Math.PI/2), (0));
		hingeC.setLimit((0), (Math.PI/2));
		*/
		hingeC =  new Ammo.btHingeConstraint(bodies[BODYPART_LEFT_UPPER_ARM],
								 bodies[BODYPART_LEFT_LOWER_ARM],
								 new Ammo.btVector3(0,0.18,0),
								 new Ammo.btVector3(0,-0.14,0),
								 new Ammo.btVector3(0,0,0.18),
								 new Ammo.btVector3(0,0,0.14));
		//		hingeC.setLimit((-Math.PI/2), (0));
		hingeC.setLimit(-Math.PI/2,0);
		joints[JOINT_LEFT_ELBOW] = hingeC;

		world.addConstraint(joints[JOINT_LEFT_ELBOW], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(new Ammo.btVector3((0.2), (0.15), (0.)));
		localB.getBasis().setEulerZYX(0,0,Math.PI/2); localB.setOrigin(new Ammo.btVector3((0.), (-0.18), (0.)));
		coneC = new Ammo.btConeTwistConstraint(bodies[BODYPART_SPINE], bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC.setLimit(Math.PI/2, Math.PI/2, 0);
		joints[JOINT_RIGHT_SHOULDER] = coneC;

		world.addConstraint(joints[JOINT_RIGHT_SHOULDER], true);

		/*
		var hingeC1_localA = new Ammo.btTransform();
		hingeC1_localA.setIdentity();
		var b1 = hingeC1_localA.getBasis();
		b1.setEulerZYX(0,Math.PI/2,0);
		hingeC1_localA.setBasis(b1);
		hingeC1_localA.setOrigin(new Ammo.btVector3(0., 0.18, 0.));
		var hingeC1_localB = new Ammo.btTransform();
		hingeC1_localB.setIdentity();
		var b2 = hingeC1_localB.getBasis();
		b2.setEulerZYX(0,Math.PI/2,0);
		hingeC1_localB.setBasis(b2);
		hingeC1_localB.setOrigin(new Ammo.btVector3(0., -0.14, 0.));
		var hingeC1 = new Ammo.btHingeConstraint(bodies[BODYPART_RIGHT_UPPER_ARM],
							 bodies[BODYPART_RIGHT_LOWER_ARM],
							 hingeC1_localA,
							 hingeC1_localB);
		*/
		/*hingeC1.setFrames(hingeC1_localA,
			hingeC1_localB);*/

		var hingeC1 = new Ammo.btHingeConstraint(bodies[BODYPART_RIGHT_UPPER_ARM],
							 bodies[BODYPART_RIGHT_LOWER_ARM],
							 new Ammo.btVector3(0,0.18,0),
							 new Ammo.btVector3(0,-0.14,0),
							 new Ammo.btVector3(0,0,0.18),
							 new Ammo.btVector3(0,0,0.14));
		hingeC1.setLimit(0.0, Math.PI/2);
		//hingeC1.setLimit((-Math.PI/2), (0.0));
		//hingeC.setLimit((0), (Math.PI/2));
		joints[JOINT_RIGHT_ELBOW] = hingeC1;

		world.addConstraint(joints[JOINT_RIGHT_ELBOW], true);

		return {
			shapes:shapes,
			bodies:bodies,
			joints:joints
		}
	}

	function createRigidBody(world, mass, startTransform, shape, options){
		options = options || {};
		if(!shape)
			return null;

		// rigidbody is dynamic if and only if mass is non zero, otherwise static
		var isDynamic = (mass != 0.0);

		var localInertia = new Ammo.btVector3(0,0,0);
		if(isDynamic)
			shape.calculateLocalInertia(mass,localInertia);

		var myMotionState = new Ammo.btDefaultMotionState(startTransform);
		var cInfo = new Ammo.btRigidBodyConstructionInfo(mass,myMotionState,shape,localInertia);
		var body = new Ammo.btRigidBody(cInfo);
		body.setLinearVelocity(new Ammo.btVector3(0,0,0));
		body.setAngularVelocity(new Ammo.btVector3(0,0,0));
		//body.setContactProcessingThreshold(this.m_defaultContactProcessingThreshold);
		if(options.collisionGroup && options.collidesWith)
			world.addRigidBody(body, options.collisionGroup, options.collidesWith);
		else
			world.addRigidBody(body);
		return body;
	};

	RagdollScript.externals = {
		key: 'RagdollScript',
		name: 'RagdollScript',
		description: '',
		parameters: [{
			key: 'whenUsed',
			type: 'boolean',
			'default': true
		}]
	};


	// BUg fixed!
	Quaternion.prototype.fromVectorToVector = function (from, to) {
		var a = from;
		var b = to;
		var factor = a.length() * b.length();
		if (Math.abs(factor) > MathUtils.EPSILON) {
			// Vectors have length > 0
			var pivotVector = new Vector3();
			var dot = a.dot(b) / factor;
			var theta = Math.acos(Math.max(-1.0, Math.min(dot, 1.0)));
			Vector3.cross(a, b, pivotVector);
			if (dot < 0.0 && pivotVector.length() < MathUtils.EPSILON) {
				// Vectors parallel and opposite direction, therefore a rotation of 180 degrees about any vector
				// perpendicular to this vector will rotate vector a onto vector b.

				// The following guarantees the dot-product will be 0.0.
				var dominantIndex;
				if (Math.abs(a.x) > Math.abs(a.y)) {
					if (Math.abs(a.x) > Math.abs(a.z)) {
						dominantIndex = 0;
					} else {
						dominantIndex = 2;
					}
				} else {
					if (Math.abs(a.y) > Math.abs(a.z)) {
						dominantIndex = 1;
					} else {
						dominantIndex = 2;
					}
				}
				pivotVector.setValue(dominantIndex, -a[((dominantIndex + 1) % 3)]);
				pivotVector.setValue((dominantIndex + 1) % 3, a[dominantIndex]);
				pivotVector.setValue((dominantIndex + 2) % 3, 0.0);
			}
			return this.fromAngleAxis(theta, pivotVector);
		} else {
			return this.set(Quaternion.IDENTITY);
		}
	};

	return RagdollScript;
});