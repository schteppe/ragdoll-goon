require([
	'goo/entities/GooRunner',
	'goo/fsmpack/statemachine/StateMachineSystem',
	'goo/entities/systems/HtmlSystem',
	'goo/timelinepack/TimelineSystem',
	'goo/loaders/DynamicLoader',

	'js/CanvasWrapper',
	'js/checkBrowser',

	'goo/fsmpack/StateMachineComponentHandler',
	'goo/fsmpack/MachineHandler',
	'goo/timelinepack/TimelineComponentHandler',
	'goo/quadpack/QuadComponentHandler',

	'goo/animation/layer/AnimationLayer',
	'goo/animation/state/SteadyState',
	'goo/animation/blendtree/ManagedTransformSource',
	'goo/animation/layer/LayerLERPBlender',

	'goo/entities/components/ScriptComponent',

	'goo/math/Vector3',
	'goo/math/Quaternion',
	'goo/math/Matrix3x3',

	'js/RagdollScript',
	'goo/scripts/Scripts',
	'goo/entities/SystemBus'

], function (
	GooRunner,
	StateMachineSystem,
	HtmlSystem,
	TimelineSystem,
	DynamicLoader,
	CanvasWrapper,
	checkBrowser,

	StateMachineComponentHandler,
	MachineHandler,
	TimelineComponentHandler,
	QuadComponentHandler,

	AnimationLayer,
	SteadyState,
	ManagedTransformSource,
	LayerLERPBlender,

	ScriptComponent,

	Vector3,
	Quaternion,
	Matrix3x3,

	RagdollScript,
	Scripts,
	SystemBus
) {
	'use strict';

	function setup(gooRunner, loader) {
		var world = gooRunner.world;
		var allEntities = world.getEntities();
		var goonEntity = world.by.name('Goon').first();
		var cameraEntity = world.by.name('Default Camera').first();
		var goonMaterial = goonEntity.children().first().children().first().meshRendererComponent.materials[0];

		/*
		var debug = new DebugRenderSystem();
		world.setSystem(debug);
		debug.doRender.SkeletonPose = true;
		debug.passive = false;
		*/

		goonMaterial.blendState.blending = 'CustomBlending';
		goonMaterial.uniforms.opacity = 0.2;

		var clip = goonEntity.animationComponent.layers[0]._currentState._sourceTree._clip; // Todo: get clip the proper way?
		setupAnimationControl(goonEntity, clip);

		SystemBus.addListener('goo.scriptError', function(evt){
			console.error(evt);
		});
	}


	/**
	 * Creates a {@link ManagedTransformSource} and a control script for looking
	 * at the mouse pointer
	 * @param {Entity} Entity
	 * @param {AnimationClip} clip
	 */
	function setupAnimationControl(entity, clip) {

		var jointNames = clip._channels.map(function(a){
			return a._jointName;
		});
		jointNames.sort();

		// A list of joints with names and other properties
		var joints = [];
		for(var i=0; i!==jointNames.length; i++){
			joints.push({
				name: jointNames[i]
			});
		}

		jointNames = joints.map(function(joint) {
			return joint.name;
		});

		// Add an animationlayer and get the source to control
		var clipSource = addManagedLayer(entity, clip, jointNames);

		Scripts.register(RagdollScript);
		var script = Scripts.create('RagdollScript', {
			clipSource: clipSource,
			joints: joints,
			entity: entity,
			offset: new Vector3(0,60,0)
		});

		// Add the controlScript
		entity.setComponent(
			new ScriptComponent(script)
		);
	}

	/**
	 * Creates an {@link AnimationLayer} with a playing {@link ManagedTransformSource}
	 * and adds it to the entity's animationComponent
	 * @param {Entity} entity
	 * @param {AnimationClip} clip
	 * @param {string[]} jointNames Names of jointChannels which the managedTransformSource
	 * will control
	 * @returns {ManagedTransformSource}
	 */
	function addManagedLayer(entity, clip, jointNames) {
		// Clipsource
		var clipSource = new ManagedTransformSource('Managed Clipsource');
		clipSource.initFromClip(clip, 'Include', jointNames);

		// State
		var state = new SteadyState('Managed State');
		state.setClipSource(clipSource);

		// Animation Layer
		var layer = entity.animationComponent.layers[0];
		layer.setState('managed', state);
		layer.setCurrentStateByName('managed');

		return clipSource;
	}

	function init() {
		// Prevent browser peculiarities to mess with our controls.
		document.body.addEventListener('touchstart', function(event) {
			event.preventDefault();
		}, false);

		// Check that the browser supports webGL
		checkBrowser();

		// Init the GooEngine
		var gooRunner = initGoo();

		// Load the project
		loadProject(gooRunner).then(function(loader) {
			gooRunner.world.process();
			return setup(gooRunner, loader);
		}).then(function() {
			// Hide the loading overlay.
			document.getElementById('loading-overlay').style.display = 'none';
			CanvasWrapper.show();
			gooRunner.world.process();
			CanvasWrapper.resize();
			// Start the rendering loop!
			gooRunner.startGameLoop();
			gooRunner.renderer.domElement.focus();
		}).then(null, function(e) {
			// If something goes wrong, 'e' is the error message from the engine.
			alert('Failed to load project: ' + e);
		});
	}

	function initGoo() {
		// Create typical Goo application.
		var gooRunner = new GooRunner({
			antialias: true,
			manuallyStartGameLoop: true,
			useDevicePixelRatio: true
		});

		gooRunner.world.add(new StateMachineSystem(gooRunner));
		gooRunner.world.add(new HtmlSystem(gooRunner.renderer));
		gooRunner.world.add(new TimelineSystem());

		return gooRunner;
	}


	function loadProject(gooRunner) {
		/**
		 * Callback for the loading screen.
		 *
		 * @param  {number} handled
		 * @param  {number} total
		 */
		var progressCallback = function (handled, total) {
			var loadedPercent = (100 * handled / total).toFixed();
			var loadingOverlay = document.getElementById("loading-overlay");
			var progressBar = document.getElementById("progress-bar");
			var progress = document.getElementById("progress");
			var loadingMessage = document.getElementById("loading-message");

			loadingOverlay.style.display = "block";
			loadingMessage.style.display = "block";
			progressBar.style.display = "block";
			progress.style.width = loadedPercent + "%";
		};

		// The loader takes care of loading the data.
		var loader = new DynamicLoader({
			world: gooRunner.world,
			rootPath: 'res'
		});

		return loader.load('root.bundle', {
			preloadBinaries: true,
			progressCallback: progressCallback
		}).then(function(result) {
			var project = null;

			// Try to get the first project in the bundle.
			for (var key in result) {
				if (/\.project$/.test(key)) {
					project = result[key];
					break;
				}
			}

			if (!project || !project.id) {
				alert('Error: No project in bundle'); // Should never happen.
				return null;
			}

			// Setup the canvas configuration (sizing mode, resolution, aspect
			// ratio, etc).
			var scene = result[project.mainSceneRef];
			var canvasConfig = scene ? scene.canvas : {};
			CanvasWrapper.setup(gooRunner.renderer.domElement, canvasConfig);
			CanvasWrapper.add();
			CanvasWrapper.hide();

			return loader.load(project.id);
		});
	}
	init();
});
