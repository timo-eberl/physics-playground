#include <cmath>
#include <iostream>
#include <chrono>

#include <tics.h>
#include <ron.h>

struct Sphere {
	const std::shared_ptr<tics::Object> object;
	const std::shared_ptr<tics::ObjectTransform> transform;
	const std::shared_ptr<tics::SphereCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct GroundPlane {
	const std::shared_ptr<tics::Object> object;
	const std::shared_ptr<tics::ObjectTransform> transform;
	const std::shared_ptr<tics::PlaneCollider> collider;
};

struct ProgramState {
	std::vector<Sphere> spheres;
	GroundPlane ground_plane;

	ron::PerspectiveCamera camera;
	ron::CameraViewportControls camera_controls;
	ron::Scene render_scene;
	std::unique_ptr<ron::OpenGLRenderer> renderer;

	tics::World physics_world;
	std::shared_ptr<tics::ImpulseSolver> impulse_solver;
	std::shared_ptr<tics::PositionSolver> position_solver;
};

static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
static ProgramState initialize(GLFWwindow* window);
static void process(GLFWwindow* window, ProgramState& state);
static void render(GLFWwindow* window, ProgramState& state);

static glm::mat4 transform_to_model_matrix(const tics::ObjectTransform &transform) {
	auto model_matrix = glm::identity<glm::mat4>();

	model_matrix = glm::translate(model_matrix, glm::vec3(
		transform.position.x, transform.position.y, transform.position.z
	));

	model_matrix = glm::scale(model_matrix, glm::vec3(
		transform.scale.x, transform.scale.y, transform.scale.z
	));

	return model_matrix;
}

static Sphere create_sphere(
	geomath::Vector3D position, geomath::Vector3D velocity, const ron::Scene &scene,
	glm::vec3 color = glm::vec3(1.0), float scale = 1.0f, float mass = 1.0f
) {
	static const auto sphere_mesh = ron::gltf::import("default/models/icosphere/icosphere.glb")
		.get_mesh_nodes()
		.front();
	Sphere sphere = Sphere({
		std::make_shared<tics::Object>(),
		std::make_shared<tics::ObjectTransform>(),
		std::make_shared<tics::SphereCollider>(),
		std::make_shared<ron::MeshNode>(*sphere_mesh) // copy
	});
	sphere.collider->radius = 1.0;
	sphere.object->collider = sphere.collider;
	sphere.object->transform = sphere.transform;
	sphere.object->mass = mass;
	sphere.transform->position = position;
	sphere.transform->scale = { scale, scale, scale };
	sphere.object->velocity = velocity;

	// copy the default material and modify it
	const auto material = std::make_shared<ron::Material>(*scene.default_material);
	material->uniforms["albedo_color"] = ron::make_uniform(glm::vec4(color, 1.0));

	// copy the mesh node to set its color
	const auto cloned_mesh_node = std::make_shared<ron::MeshNode>(
		std::make_shared<ron::Mesh>(*sphere.mesh_node->get_mesh()),
		transform_to_model_matrix(*sphere.transform)
	);
	cloned_mesh_node->get_mesh()->sections.front().material = material;

	sphere.mesh_node = cloned_mesh_node;

	return sphere;
}

int main() {
	glfwInit();

	// tell GLFW we are using OpenGL 4.6
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	// tell GLFW we want to use the core-profile -> no backwards-compatible features
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1280, 720, "Physics Playground", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	// glfwSwapInterval(0); // request to disable vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	{ // this scope ensures that "state" is destroyed before the opengl context is destroyed (glfwTerminate)
		ProgramState state = initialize(window);
		glfwSetWindowUserPointer(window, static_cast<void *>(&state));

		while (!glfwWindowShouldClose(window)) {
			auto start_time_point = std::chrono::high_resolution_clock::now();

			process(window, state);

			auto after_process_time_point = std::chrono::high_resolution_clock::now();

			render(window, state);

			auto after_render_time_point = std::chrono::high_resolution_clock::now();

			const auto physics_time = after_process_time_point - start_time_point;
			const auto render_time = after_render_time_point - after_process_time_point;

			std::cout
					<< std::endl
				<< "number of balls: " << state.spheres.size()
					<< std::endl
				<< "physics_time: "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(physics_time)
					<< std::endl
				<< "render_time: "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(render_time)
					<< std::endl;

			glfwSwapBuffers(window);
			glfwPollEvents();
		}
	}

	glfwTerminate();
	return 0;
}

ProgramState initialize(GLFWwindow* window) {
	auto scene = ron::Scene();

	auto renderer = std::make_unique<ron::OpenGLRenderer>(1280, 720);
	renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));

	const auto initial_camera_rotation = glm::vec2(glm::radians(-24.2f), glm::radians(63.6f));
	auto camera_controls = ron::CameraViewportControls(initial_camera_rotation);
	camera_controls.set_target(glm::vec3(0.0f, 0.0f, 0.0f));

	tics::World physics_world;
	physics_world.set_gravity({ 0.0, -10.0, 0.0 });

	std::vector<Sphere> spheres {};

	auto sphere_1 = create_sphere(
		{ 0.0, 1.5, 0.25 }, { 0.0, 8.0, 0.0 }, scene, glm::vec3(0.2, 0.2, 0.8), 0.75f, 0.75f
	);
	spheres.emplace_back(sphere_1);
	physics_world.add_object(sphere_1.object);
	scene.add(sphere_1.mesh_node);

	auto sphere_2 = create_sphere(
		{ -3.0, 1.0, 0.0 }, { 2.0, 12.0, 0.0 }, scene, glm::vec3(0.4, 0.3, 0.1), 1.25f, 1.25f
	);
	spheres.emplace_back(sphere_2);
	physics_world.add_object(sphere_2.object);
	scene.add(sphere_2.mesh_node);

	GroundPlane ground_plane {
		std::make_shared<tics::Object>(),
		std::make_shared<tics::ObjectTransform>(),
		std::make_shared<tics::PlaneCollider>(),
	};
	ground_plane.collider->normal = { 0.0, 1.0, 0.0 };
	ground_plane.object->collider = ground_plane.collider;
	ground_plane.object->transform = ground_plane.transform;
	ground_plane.object->is_dynamic = false;
	physics_world.add_object(ground_plane.object);
	scene.add(ron::gltf::import("models/ground.glb"));

	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	auto position_solver = std::make_shared<tics::PositionSolver>();
	physics_world.add_solver(impulse_solver);
	physics_world.add_solver(position_solver);

	renderer->preload(scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState {
		spheres,
		ground_plane,
		ron::PerspectiveCamera(60.0f, 1280.0f/720.0f, 0.1f, 1000.0f),
		camera_controls,
		scene,
		std::move(renderer),
		physics_world,
		impulse_solver,
		position_solver
	};
}

void process(GLFWwindow* window, ProgramState& state) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}

	// create spheres with random size and color
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT == GLFW_PRESS)) {
		const float size = 0.5 + 0.5 * static_cast<double>(std::rand()) / RAND_MAX;
		auto sphere = create_sphere(
			{ // position
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX) + 3.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0
			},
			{ // velocity
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5,
				(    static_cast<double>(std::rand()) / RAND_MAX)*5,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5
			},
			state.render_scene,
			glm::vec3( // color
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX
			),
			size, // scale
			size * size // weight
		);
		state.spheres.emplace_back(sphere);
		state.physics_world.add_object(sphere.object);
		state.render_scene.add(sphere.mesh_node);
	}

	float time_scale = 1.0f;
	state.physics_world.update(time_scale/60.0f);

	for (const auto &sphere : state.spheres) {
		sphere.mesh_node->set_model_matrix(transform_to_model_matrix(*sphere.transform));
	}

	state.camera_controls.update(*window, state.camera);
}

void render(GLFWwindow* window, ProgramState& state) {
	state.renderer->render(state.render_scene, state.camera);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	auto state = static_cast<ProgramState *>(glfwGetWindowUserPointer(window));
	assert(state);

	state->renderer->resolution = glm::uvec2(width, height);

	state->camera.set_aspect_ratio(static_cast<float>(width) / static_cast<float>(height));
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	auto state = static_cast<ProgramState *>(glfwGetWindowUserPointer(window));

	if (state) {
		if (yoffset == 1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::UP);
		}
		else if (yoffset == -1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::DOWN);
		}
	}
}
