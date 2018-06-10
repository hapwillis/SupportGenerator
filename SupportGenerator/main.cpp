#include <glad/glad.h>
#include <GLFW/glfw3.h>

//#define GLM_ENABLE_EXPERIMENTAL
//#include <glm/gtx/string_cast.hpp>

#include <Camera.h>
#include <DefaultShader.h>
#include <Octree.h>
#include <Heap.h>
#include <Model.h>

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void drop_callback(GLFWwindow* window, int count, const char** paths);
void processInput(GLFWwindow *window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const float FOV = glm::radians(45.0f);

// camera
Camera camera(FOV, SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 3.0f));
int scrWidth = SCR_WIDTH;
int scrHeight = SCR_HEIGHT;

// Model
Model* model = {0};

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
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
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
		// input
		// -----
		//std::cout << "Camera Pos: " << camera.Position.x << ", " << camera.Position.y << ", " << camera.Position.z << std::endl;
		/*std::cout << "Target Pos: " << camera.Target.x << ", " << camera.Target.y << ", " << camera.Target.z << std::endl;
		std::cout << "Pitch: " << camera.Pitch << " Yaw: " << camera.Yaw << std::endl;
		std::cout << "Up vector: " << camera.Up.x << ", " << camera.Up.y << ", " << camera.Up.z << std::endl;
		std::cout << "WorldUp vector: " << camera.WorldUp.x << ", " << camera.WorldUp.y << ", " << camera.WorldUp.z << std::endl;
		std::cout << "Right vector: " << camera.Right.x << ", " << camera.Right.y << ", " << camera.Right.z << std::endl;*/
		//std::cout << "Distance: " << camera.Distance << std::endl;
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
		proj = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);

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

		if (model)
			model->Draw(shader);

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

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

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
			camera.StartPan(action == GLFW_PRESS, xpos / scrWidth, ypos / scrHeight);
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
	if (model)
		delete model;

	model = new Model(p);
	camera.TargetModel(model);
}
