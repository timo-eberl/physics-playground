#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>

#include "game_loop.h"
#include "utils.h"

#include <tics.h>
#include <ron.h>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <TSVector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

using namespace std::chrono_literals;

auto accumulated_render_time = 0ns;
unsigned int accumulated_render_time_count = 0;
auto accumulated_physics_time = 0ns;
unsigned int accumulated_physics_time_count = 0;

struct AreaTrigger {
	const std::shared_ptr<tics::CollisionArea> area;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct RaycastTarget {
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct ProgramState {
	std::chrono::_V2::system_clock::time_point start_time_point;

	std::shared_ptr<std::vector<Sphere>> spheres;
	std::shared_ptr<std::vector<StaticObject>> static_geometry;
	AreaTrigger area_trigger;
	RaycastTarget raycast_target;
	RaycastTarget raycast_target_2;

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

			// measure performance
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
					// << " - Render FPS: "
					// << static_cast<int>(1000.0 / avg_render_time_ms)
					<< " - Phyiscs Time: "
					<< std::fixed << std::setprecision(2) // specify decimal places
					<< avg_physics_time_ms << "ms"
					// << " - Physics FPS: "
					// << static_cast<int>(1000.0 / avg_physics_time_ms)
					;
				glfwSetWindowTitle(window,  + title_stream.str().c_str());
			}
		}
	}

	glfwTerminate();
	return 0;
}

ProgramState initialize(GLFWwindow* window) {
	// renderer setup
	auto scene = ron::Scene();
	scene.set_directional_light(create_generic_light());
	auto renderer = std::make_unique<ron::OpenGLRenderer>(1280, 720);
	renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));
	// camera setup
	const auto initial_camera_rotation = glm::vec2(glm::radians(-24.2f), glm::radians(63.6f));
	auto camera_controls = ron::CameraViewportControls(initial_camera_rotation);
	camera_controls.set_target(glm::vec3(0.0f, 0.0f, 0.0f));

	tics::World physics_world;
	physics_world.set_gravity(Terathon::Vector3D(0.0, -10.0, 0.0));

	auto spheres = std::make_shared<std::vector<Sphere>>();

	// initial sphere
	auto sphere_1 = create_sphere(
		Terathon::Vector3D(0.0, 1.5, 0.25), Terathon::Vector3D(0.0, 8.0, 0.0), scene, glm::vec3(0.2, 0.2, 0.8), 0.75f, 0.75f
	);
	// spheres->emplace_back(sphere_1);
	// physics_world.add_object(sphere_1.rigid_body);
	// scene.add(sphere_1.mesh_node);

	// add static geometry
	auto static_geometry = create_static_objects("models/ground.glb");
	for (const auto &static_object : *static_geometry) {
		physics_world.add_object(static_object.static_body);
		scene.add(static_object.mesh_node);
	}

	// area trigger that turns objects red that are inside it
	AreaTrigger area_trigger = {
		std::make_shared<tics::CollisionArea>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/icosphere_lowres.glb").get_mesh_nodes().front()
	};
	area_trigger.area->set_collider(area_trigger.collider);
	area_trigger.area->set_transform(area_trigger.transform);
	area_trigger.area->on_collision_enter = [spheres](const auto &other) {
		for (const auto &sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"]
					= ron::make_uniform(glm::vec4(glm::vec3(1.0, 0.1, 0.1), 1.0));
			}
		}
	};
	area_trigger.area->on_collision_exit = [spheres](const auto &other) {
		for (const auto &sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"]
					= ron::make_uniform(glm::vec4(sphere.color, 1.0));
			}
		}
	};
	// apply scale to mesh
	for (auto &section : area_trigger.mesh_node->get_mesh()->sections) {
		for (auto &position : section.geometry->positions) {
			position *= glm::vec3(2.0, 1.0, 4.0);
		}
	}
	area_trigger.transform->position = Terathon::Vector3D(-2.0, 2.0, 0.0);
	area_trigger.mesh_node->set_model_matrix(transform_to_model_matrix(*area_trigger.transform));
	const auto area_trigger_geometry = area_trigger.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	area_trigger.collider->indices = area_trigger_geometry->indices;
	for (const auto &vertex_pos : area_trigger_geometry->positions) {
		area_trigger.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}
	physics_world.add_object(area_trigger.area);
	scene.add(area_trigger.mesh_node);

	// raycasting setup - first target
	RaycastTarget raycast_target = {
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/triangle_0.glb").get_mesh_nodes().front()
	};
	const auto raycast_target_geometry = raycast_target.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	raycast_target.collider->indices = raycast_target_geometry->indices;
	for (const auto &vertex_pos : raycast_target_geometry->positions) {
		raycast_target.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}
	scene.add(raycast_target.mesh_node);

	// raycasting setup - second target
	RaycastTarget raycast_target_2 = {
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/triangle_1.glb").get_mesh_nodes().front()
	};
	const auto raycast_target_2_geometry = raycast_target_2.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	raycast_target_2.collider->indices = raycast_target_2_geometry->indices;
	for (const auto &vertex_pos : raycast_target_2_geometry->positions) {
		raycast_target_2.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}
	scene.add(raycast_target_2.mesh_node);

	// applies forces
	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	// fixes intersections
	auto position_solver = std::make_shared<tics::PositionSolver>();
	// alerts collision areas
	auto collision_area_solver = std::make_shared<tics::CollisionAreaSolver>();
	physics_world.add_solver(impulse_solver);
	physics_world.add_solver(position_solver);
	physics_world.add_solver(collision_area_solver);

	renderer->preload(scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState {
		std::chrono::high_resolution_clock::now(),
		spheres,
		static_geometry,
		area_trigger,
		raycast_target,
		raycast_target_2,
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

	static std::vector<std::chrono::nanoseconds> raycast_times;
	const auto pga = true;

	const auto time_since_start = std::chrono::high_resolution_clock::now() - state.start_time_point;
	const auto time_since_start_s = time_since_start.count() / 1000000000.0;
	const auto cos_time = cos( 2 * time_since_start_s );

	const auto target_glm = state.camera_controls.get_target();
	
	auto glm_model_mat = state.raycast_target.mesh_node->get_model_matrix();
	glm_model_mat[3][0] = target_glm.x + cos_time * 0.5;
	glm_model_mat[3][1] = target_glm.y + cos_time * 0.5;
	glm_model_mat[3][2] = target_glm.z;
	state.raycast_target.mesh_node->set_model_matrix(glm_model_mat);
	const auto model_mat = Terathon::Transform3D(
		glm_model_mat[0][0], glm_model_mat[1][0], glm_model_mat[2][0], glm_model_mat[3][0],
		glm_model_mat[0][1], glm_model_mat[1][1], glm_model_mat[2][1], glm_model_mat[3][1],
		glm_model_mat[0][2], glm_model_mat[1][2], glm_model_mat[2][2], glm_model_mat[3][2]
	);
	auto model_motor = Terathon::Motor3D();
	model_motor.SetTransformMatrix(model_mat);

	auto glm_model_mat_2 = state.raycast_target_2.mesh_node->get_model_matrix();
	glm_model_mat_2[3][0] = target_glm.x + cos_time * 0.5;
	glm_model_mat_2[3][1] = target_glm.y + cos_time * 0.5;
	glm_model_mat_2[3][2] = target_glm.z;
	state.raycast_target_2.mesh_node->set_model_matrix(glm_model_mat_2);
	const auto model_mat_2 = Terathon::Transform3D(
		glm_model_mat_2[0][0], glm_model_mat_2[1][0], glm_model_mat_2[2][0], glm_model_mat_2[3][0],
		glm_model_mat_2[0][1], glm_model_mat_2[1][1], glm_model_mat_2[2][1], glm_model_mat_2[3][1],
		glm_model_mat_2[0][2], glm_model_mat_2[1][2], glm_model_mat_2[2][2], glm_model_mat_2[3][2]
	);
	auto model_motor_2 = Terathon::Motor3D();
	model_motor_2.SetTransformMatrix(model_mat_2);

	const auto cam_model_matrix = state.camera.get_model_matrix();
	const auto cam_pos_glm = cam_model_matrix * glm::vec4(0,0,0,1);
	const auto cam_pos = Terathon::Point3D(cam_pos_glm.x, cam_pos_glm.y, cam_pos_glm.z);

	// intersect only one triangle test
	{
		auto target = Terathon::Point3D(target_glm.x, target_glm.y, target_glm.z);

		const auto direction = Terathon::Normalize( target - Terathon::Point3D(cam_pos_glm.x, cam_pos_glm.y, cam_pos_glm.z) );
		target = cam_pos + direction;

		const bool pga_raycast_1_hit = tics::pga_raycast(model_motor, *state.raycast_target.collider, cam_pos, target);
		const bool pga_raycast_2_hit = tics::pga_raycast(model_motor_2, *state.raycast_target_2.collider, cam_pos, target);

		const bool raycast_1_hit = tics::raycast(model_mat, *state.raycast_target.collider, cam_pos, direction);
		const bool raycast_2_hit = tics::raycast(model_mat_2, *state.raycast_target_2.collider, cam_pos, direction);

		std::cout << "    raycast_hit: " << raycast_1_hit << ", " << raycast_2_hit << std::endl;
		// assert(raycast_1_hit || raycast_2_hit);
		static int raycast_fail = 0;
		if (!(raycast_1_hit || raycast_2_hit)) {
			raycast_fail++;
		}
		std::cout << "pga_raycast_hit: " << pga_raycast_1_hit << ", " << pga_raycast_2_hit << std::endl;
		// assert(pga_raycast_1_hit || pga_raycast_2_hit);
		static int pga_raycast_fail = 0;
		if (!(pga_raycast_1_hit || pga_raycast_2_hit)) {
			pga_raycast_fail++;
		}
		std::cout << "***** conventional " << raycast_fail << ":" << pga_raycast_fail << " pga *****" << std::endl;
	}

	// performance measuring
	if (pga) {
		// without transform:
		// suzanne.glb: ~ 0.50 ms
		// suzanne_high_res.glb: : ~ 90 ms

		// with transform:
		// suzanne.glb: ~ 1.21 ms (+0.71 ms)
		// suzanne_high_res.glb: : ~ 274 ms (+184 ms)
		const auto cam_target_glm = state.camera_controls.get_target();
		const auto cam_target = Terathon::Point3D(cam_target_glm.x, cam_target_glm.y, cam_target_glm.z);
		// const bool pga_raycast_hit = tics::pga_raycast(model_motor, *state.raycast_target.collider, cam_pos, cam_target);
		const auto ray_cast_time = std::chrono::high_resolution_clock::now() - start_time_point;
		raycast_times.push_back(ray_cast_time);
	} else {
		// without transform:
		// suzanne.glb: ~ 0.33 ms
		// suzanne_high_res.glb: ~ 53 ms

		// with transform:
		// suzanne.glb: ~ 0.75 ms (+0.42 ms)
		// suzanne_high_res.glb: : ~ 140 ms (+87 ms)
		const auto target_pos_glm = cam_model_matrix * glm::vec4(0,0,-1,1);
		const auto direction_glm = target_pos_glm - cam_pos_glm;
		const auto direction = Terathon::Vector3D(direction_glm.x, direction_glm.y, direction_glm.z);

		const bool raycast_hit = tics::raycast(model_mat, *state.raycast_target.collider, cam_pos, direction);
		const auto ray_cast_time = std::chrono::high_resolution_clock::now() - start_time_point;
		raycast_times.push_back(ray_cast_time);
	}
	std::chrono::nanoseconds total = 0ns;
	for (const auto &t : raycast_times) {
		total += t;
	}
	// std::cout << "avg time: " << total / raycast_times.size() << "\n";

	// create spheres with random size and color
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT == GLFW_PRESS)) {
		const float size = 0.5 + 0.5 * static_cast<double>(std::rand()) / RAND_MAX;
		auto sphere = create_sphere(
			Terathon::Vector3D( // position
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX) + 3.0,
				(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0
			),
			Terathon::Vector3D( // velocity
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
			size * size // weight
		);
		state.spheres->emplace_back(sphere);
		state.physics_world.add_object(sphere.rigid_body);
		state.render_scene.add(sphere.mesh_node);
	}

	float time_scale = 1.0f;
	state.physics_world.update(time_scale/60.0f);

	const auto physics_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_physics_time_count;
	accumulated_physics_time += physics_time;

	for (const auto &sphere : *state.spheres) {
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
