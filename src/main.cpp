#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>

#include <tics.h>
#include <ron.h>

#include "game_loop.h"

using namespace std::chrono_literals;

auto accumulated_render_time = 0ns;
unsigned int accumulated_render_time_count = 0;
auto accumulated_physics_time = 0ns;
unsigned int accumulated_physics_time_count = 0;

struct Sphere {
	const std::shared_ptr<tics::RigidBody> rigid_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::SphereCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct GroundPlane {
	const std::shared_ptr<tics::StaticBody> static_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::PlaneCollider> collider;
};

struct AreaTrigger {
	const std::shared_ptr<tics::CollisionArea> area;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::SphereCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct ProgramState {
	std::vector<Sphere> spheres;
	GroundPlane ground_plane;
	AreaTrigger area_trigger;

	ron::PerspectiveCamera camera;
	ron::CameraViewportControls camera_controls;
	ron::Scene render_scene;
	std::unique_ptr<ron::OpenGLRenderer> renderer;

	tics::World physics_world;
	std::shared_ptr<tics::ImpulseSolver> impulse_solver;
	std::shared_ptr<tics::PositionSolver> position_solver;
	std::shared_ptr<tics::CollisionAreaSolver> collision_area_solver;
};

static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
static ProgramState initialize(GLFWwindow* window);
static void process(GLFWwindow* window, ProgramState& state);
static void render(GLFWwindow* window, ProgramState& state);

static glm::mat4 transform_to_model_matrix(const tics::Transform &transform) {
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
	gm::Vector3 position, gm::Vector3 velocity, const ron::Scene &scene,
	glm::vec3 color = glm::vec3(1.0), float scale = 1.0f, float mass = 1.0f
) {
	static const auto sphere_mesh = ron::gltf::import("default/models/icosphere/icosphere.glb")
		.get_mesh_nodes()
		.front();
	Sphere sphere = Sphere({
		std::make_shared<tics::RigidBody>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::SphereCollider>(),
		std::make_shared<ron::MeshNode>(*sphere_mesh) // copy
	});
	sphere.collider->radius = 1.0;
	sphere.rigid_body->set_collider(sphere.collider);
	sphere.rigid_body->set_transform(sphere.transform);
	sphere.rigid_body->mass = mass;
	sphere.transform->position = position;
	sphere.transform->scale = gm::Vector3(scale, scale, scale);
	sphere.rigid_body->velocity = velocity;

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

		GameLoop game_loop(
			[window, &state](const float delta, const float fixed_delay) { render(window, state); },
			[window, &state](const float delta) { process(window, state); },
			1.0f/60.0f, 0.0f, 1.0f/60.0f
		);

		auto last_title_update_time_point = std::chrono::high_resolution_clock::now();

		while (!glfwWindowShouldClose(window)) {
			game_loop.update();

			static const auto update_interval = 250ms;
			const auto now = std::chrono::high_resolution_clock::now();
			if (now - last_title_update_time_point >= update_interval) {
				last_title_update_time_point = now;

				const auto avg_render_time = std::chrono::duration<double>(
					accumulated_render_time / accumulated_render_time_count
				);
				const double avg_render_time_ms = avg_render_time.count() * 1000.0;

				accumulated_render_time = 0ns;
				accumulated_render_time_count = 0;

				const auto avg_physics_time = std::chrono::duration<double>(
					accumulated_physics_time / accumulated_physics_time_count
				);
				const double avg_physics_time_ms = avg_physics_time.count() * 1000.0;

				accumulated_physics_time = 0ns;
				accumulated_physics_time_count = 0;

				std::stringstream title_stream;
				title_stream
					<< "Physics Playground"
					<< " - Render Time: "
					<< std::fixed << std::setprecision(2) // specify decimal places
					<< avg_render_time_ms << "ms"
					<< " - Render FPS: "
					<< static_cast<int>(1000.0 / avg_render_time_ms)
					<< " - Phyiscs Time: "
					<< std::fixed << std::setprecision(2) // specify decimal places
					<< avg_physics_time_ms << "ms"
					<< " - Physics FPS: "
					<< static_cast<int>(1000.0 / avg_physics_time_ms);
				glfwSetWindowTitle(window,  + title_stream.str().c_str());
			}
		}
	}

	glfwTerminate();
	return 0;
}

ProgramState initialize(GLFWwindow* window) {
	auto scene = ron::Scene();

	ron::DirectionalLight light = {};
	light.use_custom_shadow_target_world_position = true;
	light.custom_shadow_target_world_position = glm::vec3(0.0f, 0.0f, 0.0f);
	light.world_direction = glm::normalize(glm::vec3(0.1f, 1.0f, 0.0f));
	light.shadow.enabled = true;
	light.shadow.map_size = glm::uvec2(2048);
	light.shadow.bias = 0.05f;
	light.shadow.far = 30.0f;
	light.shadow.frustum_size = 50.0f;

	scene.set_directional_light(light);

	auto renderer = std::make_unique<ron::OpenGLRenderer>(1280, 720);
	renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));

	const auto initial_camera_rotation = glm::vec2(glm::radians(-24.2f), glm::radians(63.6f));
	auto camera_controls = ron::CameraViewportControls(initial_camera_rotation);
	camera_controls.set_target(glm::vec3(0.0f, 0.0f, 0.0f));

	tics::World physics_world;
	physics_world.set_gravity(gm::Vector3(0.0, -10.0, 0.0));

	std::vector<Sphere> spheres {};

	auto sphere_1 = create_sphere(
		gm::Vector3(0.0, 1.5, 0.25), gm::Vector3(0.0, 8.0, 0.0), scene, glm::vec3(0.2, 0.2, 0.8), 0.75f, 0.75f
	);
	spheres.emplace_back(sphere_1);
	physics_world.add_object(sphere_1.rigid_body);
	scene.add(sphere_1.mesh_node);

	auto sphere_2 = create_sphere(
		gm::Vector3(-3.0, 1.0, 0.0), gm::Vector3(2.0, 12.0, 0.0), scene, glm::vec3(0.4, 0.3, 0.1), 1.25f, 1.25f
	);
	spheres.emplace_back(sphere_2);
	physics_world.add_object(sphere_2.rigid_body);
	scene.add(sphere_2.mesh_node);

	AreaTrigger area_trigger = {
		std::make_shared<tics::CollisionArea>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::SphereCollider>(),
		ron::gltf::import("default/models/icosphere/icosphere.glb").get_mesh_nodes().front()
	};
	area_trigger.area->set_collider(area_trigger.collider);
	area_trigger.area->set_transform(area_trigger.transform);
	area_trigger.area->on_collision_enter = [](const auto &other) { std::cout << "enter\n"; };
	area_trigger.area->on_collision_exit = [](const auto &other) { std::cout << "exit\n"; };
	area_trigger.collider->radius = 1.0;
	area_trigger.transform->position = gm::Vector3(0.0, 2.0, 0.0);
	area_trigger.transform->scale = gm::Vector3(1.0, 1.0, 1.0);
	area_trigger.mesh_node->set_model_matrix(transform_to_model_matrix(*area_trigger.transform));
	physics_world.add_object(area_trigger.area);
	scene.add(area_trigger.mesh_node);

	const auto fasdk = gm::Vector<10000>();
	for (size_t i = 0; i < 10000; i++) {
		assert(fasdk[i] == 0.0f);
	}

	const gm::Vector3 fjasdlj = gm::Vector3( 1, 2, 3 );
	const auto fdsgdsaf = gm::Vector<1, unsigned int>(1.3);
	std::cout << "fdsgdsaf: " << fdsgdsaf << std::endl;
	const auto gdasf = gm::Vector2(1,2);
	const auto fasdgdasf = gm::Vector3(1,2,3);
	const auto fasdfas = gm::Vector4(1,2,3,4);
	std::cout << "fasdgdasf: " << fasdgdasf << std::endl;
	std::cout << "fasdgdasf.xy: " << fasdgdasf.xy << std::endl;
	std::cout << "fasdgdasf.x: " << fasdgdasf.x << std::endl;
	std::cout << "fasdgdasf.y: " << fasdgdasf.y << std::endl;
	std::cout << fasdgdasf[0] << "," << fasdgdasf[1] << "," << fasdgdasf[2] << std::endl;
	std::cout << "xyz: " << fasdgdasf.x << "," << fasdgdasf.y << "," << fasdgdasf.z << std::endl;

	auto fdas = gm::Vector<5>(1,2,3,4,5);
	assert(fdas[0] == 1.0f && fdas[1] == 2.0f && fdas[2] == 3.0f && fdas[3] == 4.0f && fdas[4] == 5.0f);
	auto gfds = gm::Vector<5>({ 1.0f, 2.0f, 3.0f, 4.0f, 5.0f });
	assert(gfds == fdas);
	auto gfhdshfsd = gm::Vector<5>(435,2,3,4,5);
	assert(gfhdshfsd != fdas);
	std::cout << "gfhdshfsd: " << gfhdshfsd << std::endl;

	auto mat2 = gm::Matrix<2,2>();
	mat2.data[0][0] = 0;
	mat2.data[0][1] = 1;
	mat2.data[0][2] = 3;
	mat2.data[0][3] = 4;
	std::cout << "mat2: " << mat2.data[0][0] << "," << mat2.data[0][1] << ","
		<< mat2.data[1][0] << "," << mat2.data[1][1] << std::endl;
	std::cout << "mat2: " << mat2.contiguous_data[0] << "," << mat2.contiguous_data[1] << ","
		<< mat2.contiguous_data[2] << "," << mat2.contiguous_data[3] << std::endl;

	GroundPlane ground_plane {
		std::make_shared<tics::StaticBody>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::PlaneCollider>(),
	};
	ground_plane.collider->normal = gm::Vector3(0.0, 1.0, 0.0);
	ground_plane.static_body->set_collider(ground_plane.collider);
	ground_plane.static_body->set_transform(ground_plane.transform);
	physics_world.add_object(ground_plane.static_body);
	scene.add(ron::gltf::import("models/ground.glb"));

	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	auto position_solver = std::make_shared<tics::PositionSolver>();
	auto collision_area_solver = std::make_shared<tics::CollisionAreaSolver>();
	physics_world.add_solver(impulse_solver);
	physics_world.add_solver(position_solver);
	physics_world.add_solver(collision_area_solver);

	renderer->preload(scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState {
		spheres,
		ground_plane,
		area_trigger,
		ron::PerspectiveCamera(60.0f, 1280.0f/720.0f, 0.1f, 1000.0f),
		camera_controls,
		scene,
		std::move(renderer),
		physics_world,
		impulse_solver,
		position_solver,
		collision_area_solver
	};
}

void process(GLFWwindow* window, ProgramState& state) {
	glfwPollEvents();

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}

	auto start_time_point = std::chrono::high_resolution_clock::now();

	// create spheres with random size and color
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT == GLFW_PRESS)) {
		const float size = 0.5 + 0.5 * static_cast<double>(std::rand()) / RAND_MAX;
		auto sphere = create_sphere(
			gm::Vector3( // position
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX) + 3.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0
			),
			gm::Vector3( // velocity
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5,
				(    static_cast<double>(std::rand()) / RAND_MAX)*5,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5
			),
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
		state.physics_world.add_object(sphere.rigid_body);
		state.render_scene.add(sphere.mesh_node);
	}

	float time_scale = 1.0f;
	state.physics_world.update(time_scale/60.0f);

	const auto physics_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_physics_time_count;
	accumulated_physics_time += physics_time;

	for (const auto &sphere : state.spheres) {
		sphere.mesh_node->set_model_matrix(transform_to_model_matrix(*sphere.transform));
	}

	state.camera_controls.update(*window, state.camera);
}

void render(GLFWwindow* window, ProgramState& state) {
	auto start_time_point = std::chrono::high_resolution_clock::now();

	state.renderer->render(state.render_scene, state.camera);

	const auto render_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_render_time_count;
	accumulated_render_time += render_time;

	glfwSwapBuffers(window);
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
