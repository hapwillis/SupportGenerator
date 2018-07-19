#include <glad/glad.h>
#include <GLFW/glfw3.h>



//#define GLM_ENABLE_EXPERIMENTAL
//#include <glm/gtx/string_cast.hpp>

#include <Camera.h>
#include <DefaultShader.h>
#include <Octree.h>
#include <Model.h>
#include <NavigationMesh.h>
#include <ConnectionPoints.h>
#include <SupportPaths.h>

#include <iostream>
#include <thread>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void drop_callback(GLFWwindow* window, int count, const char** paths);
void processInput(GLFWwindow *window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// Settings
// TODO: collect these into a struct (ScreenProperties)
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const float FOV = glm::radians(45.0f);

// camera
// TODO: move into Camera
int scrWidth = SCR_WIDTH;
int scrHeight = SCR_HEIGHT;
Camera camera(FOV, scrWidth, scrHeight, glm::vec3(0.0f, 0.0f, 3.0f));

// context:
// TODO: collect these into a struct (ProcessSettings)
int processStep = 0;
std::thread processThread;
bool selectSupportFlag = false;
const float SupportOffset = 2.0f;
const float SupportWidth = 5.0f;
const float maxOverhang = 0.785398f;

Model *model = {0};
NavigationMesh *navMesh = { 0 };
Graph *navGraph = { 0 };
ConnectionPoints *connections = { 0 };
SupportPaths *paths = { 0 };

//TODO: add context for UI

int main()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(scrWidth, scrHeight, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetDropCallback(window, drop_callback);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);

	// build and compile shader 
	// ------------------------------------
	DefaultShader shader("default.vs", "default.fs");

	// TODO: create bed stand-in and axis markers.
	float vertices[] = {
		-0.5f, -0.5f, 0.0f,
		0.5f, -0.5f, 0.0f,
		0.0f,  0.5f, 0.0f
	};
	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// how to parse position attributes inside the VBO
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		processInput(window);
		
		// render
		// ------
		glClearColor(0.718f, 0.718f, 0.718f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// create transformations
		glm::mat4 modelLoc(1.0f); // Turns out these *don't* initialize to an identity by default
		glm::mat4 proj;
		// Convert to right-handed space (z is not up)
		modelLoc = glm::rotate(modelLoc, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		proj = glm::perspective(glm::radians(45.0f), (float)scrWidth / (float)scrHeight, 0.1f, 1000.0f);

		// camera/view transformation
		glm::mat4 view = camera.GetViewMatrix();

		shader.use();
		shader.setMat4("projection", proj);
		shader.setMat4("view", view);
		shader.setMat4("model", modelLoc);

		shader.setVec3("lightOneDir", glm::vec3(0.0f, -1.0f, 0.0f));
		shader.setFloat("lightOneInten", 1.0f);

		shader.setVec3("camLightDir", camera.Direction());
		shader.setFloat("camLightInten", 0.8f);
		shader.setVec4("color", glm::vec4(0.565f, 0.565f, 0.565f, 1.0f));

		if (model)
			model->Draw(shader);
		if (navMesh)
			navMesh->Draw(shader); 
		if (connections)
			connections->Draw(shader);
		if (paths)
			paths->Draw(shader);

		//NB that spreading supports by radius is mathematically equivalent to formlabs method:
		//https://youtu.be/5VlprrdGYKM?t=22m33s
		//Formlabs method is slower but can be adapted to nonlinear distributions

		// get matrix's uniform location and set matrix

		//glBindVertexArray(VAO);
		//glDrawArrays(GL_TRIANGLES, 0, 3);

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
	return 0;
}


// Time to run when called: 795 milliseconds
void updateConnectionPoints()
{
	double time = glfwGetTime();

	if (navMesh)
		delete(navMesh);
	navMesh = new NavigationMesh(*model);

	if (connections)
		delete(connections);
	connections = new ConnectionPoints(navMesh->graph, maxOverhang);
	std::cout << "Time to create connections: " << glfwGetTime() - time << std::endl;
}


// Time to run when called: 2709 milliseconds
void updateNavMesh()
{
	double time = glfwGetTime();
	navGraph = navMesh->getSimpleGraph(1.0f); // (0.5f * SupportWidth) + SupportOffset
	navMesh->mesh = navMesh->convertToMesh(navGraph, SupportOffset);
	std::cout << "Time to create navigation mesh: " << glfwGetTime() - time << std::endl;
}


void updateSupportPaths()
{
	double time = glfwGetTime();
	if (paths)
		delete(paths);
	paths = new SupportPaths(navMesh->graph, navGraph, connections->points, maxOverhang, SupportOffset);
	paths->FindPaths();
	paths->Geometry(6, 0.0f);
	std::cout << "Time to pathfind supports: " << glfwGetTime() - time << std::endl;
}


void updateSupportGeometry()
{
	double time = glfwGetTime();
	// TODO: updateSupportGeometry
	std::cout << "Time to create support geometry: " << glfwGetTime() - time << std::endl;
}


void exportSupportedModel()
{
	double time = glfwGetTime();
	// TODO: exportSupportedModel
	std::cout << "Time to export supported model: " << glfwGetTime() - time << std::endl;
}

// Coordinates and advances the support process when called
void processIncrement()
{
	// TODO: skip this if no model is loaded
	// TODO: thread these steps
	//if (processThread.joinable())
		//processThread.join();
	switch (processStep) { 
	case 0: //processThread = std::thread();
		updateConnectionPoints(); // 903 milliseconds spent calling
		break;
	case 1: //processThread = std::thread();
		updateNavMesh(); // 2748 milliseconds spent calling
		break;
	case 2: //processThread = std::thread();
		updateSupportPaths(); 
		break;
	case 3: //processThread = std::thread();
		updateSupportGeometry();
		break;
	case 4: //processThread = std::thread();
		exportSupportedModel();
		break;
	}
	
	if (processStep > 3) {
		processStep = 0;
	} else {
		processStep++;
	}
}


void selectSupport(double x, double y)
{
	// TODO: select support
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) 
		processIncrement();

	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	camera.ProcessMouseMovement(xpos / scrWidth, ypos / scrHeight, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	scrWidth = width;
	scrHeight = height;
	glViewport(0, 0, width, height);
}


void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	
	switch (button) {
		case GLFW_MOUSE_BUTTON_LEFT : 
			// TODO: make sure FOV is up to date
			camera.StartPan(action == GLFW_PRESS, xpos / scrWidth, ypos / scrHeight);
			if (selectSupportFlag)
				selectSupport(xpos / scrWidth, ypos / scrHeight);
			break;
		case GLFW_MOUSE_BUTTON_RIGHT : // call movement tracker
			camera.StartMov(action == GLFW_PRESS, xpos / scrWidth, ypos / scrHeight);
			break;
	}
}


void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

// The callback function receives an array of paths encoded as UTF-8.
void drop_callback(GLFWwindow* window, int count, const char** paths)
{
	// The path array and its strings are only valid until the file drop callback returns, 
	// as they may have been generated specifically for that event. You need to make a 
	// deep copy of the array if you want to keep the paths.
	int i;
	for (i = 0; i < count; i++)
		std::cout << "file detected" << std::endl;

	std::string p(paths[0]);
	if (model) {
		delete model;
		if (navMesh) {
			delete(navMesh);
			navMesh = NULL;
		}
	}
	float time = glfwGetTime();
	model = new Model(p);
	std::cout << "Time to load model: " << glfwGetTime() - time << std::endl;
	camera.TargetModel(model);
	processStep = 0;
}
