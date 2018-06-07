#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

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

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
int scrWidth = SCR_WIDTH;
int scrHeight = SCR_HEIGHT;

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
	/*	std::cout << "Camera Pos: " << camera.Position.x << ", " << camera.Position.y << ", " << camera.Position.z << std::endl;
		std::cout << "Target Pos: " << camera.Target.x << ", " << camera.Target.y << ", " << camera.Target.z << std::endl;
		std::cout << "Up vector: " << camera.Up.x << ", " << camera.Up.y << ", " << camera.Up.z << std::endl;
		std::cout << "Distance: " << camera.Distance << std::endl;*/
		processInput(window);
		
		// render
		// ------
		glClearColor(0.718f, 0.718f, 0.718f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// create transformations
		glm::mat4 model(1.0f); // Turns out these *don't* initialize to an identity by default
		glm::mat4 proj;
		model = glm::rotate(model, glm::radians(-55.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		proj = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

		// camera/view transformation
		// glm::mat4 view(1.0f);
		//glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
		glm::mat4 view = camera.GetViewMatrix();
		//std::cout << glm::to_string(view) << std::endl;

		shader.use();
		shader.setMat4("projection", proj);
		shader.setMat4("view", view);
		shader.setMat4("model", model);

		// get matrix's uniform location and set matrix

		glBindVertexArray(VAO);
		glDrawArrays(GL_TRIANGLES, 0, 3);

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
	// TODO: scale by screen area
	//std::cout << "x/y coords: " << xpos / scrWidth << ", " << ypos / scrHeight << std::endl;
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

// The callback function receives an array of paths encoded as UTF-8.
void drop_callback(GLFWwindow* window, int count, const char** paths)
{
	// The path array and its strings are only valid until the file drop callback returns, 
	// as they may have been generated specifically for that event. You need to make a 
	// deep copy of the array if you want to keep the paths.
	int i;
	for (i = 0; i < count; i++)
		std::cout << "file detected" << std::endl;
	//handle_dropped_file(paths[i]);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	// TODO: scale by screen area
	
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


