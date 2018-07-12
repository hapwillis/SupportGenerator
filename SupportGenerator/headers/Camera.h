#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <Model.h>

// Default camera values
const float YAW = -0.25f;
const float PITCH = 0.17f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;

class Camera
{
public:
	// Camera Attributes
	glm::vec3 Target;
	glm::vec3 Position;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	// Euler Angles
	float Yaw;
	float Pitch;
	// Camera options
	float MouseSensitivity;
	bool TrackPan;
	bool TrackMov;
	float XPos, YPos;
	float Distance;
	float Hfov;
	float Vfov;

	Camera(float fov, int screenWidth, int screenHeight, glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH);

	~Camera();

	glm::mat4 GetViewMatrix();

	void StartPan(bool start, float xpos, float ypos);

	void StartMov(bool start, float xpos, float ypos);

	void ProcessMouseMovement(float xpos, float ypos, GLboolean constrainPitch = true);

	void ProcessMouseScroll(float yoffset);

	glm::vec3 Direction();

	// Need a new function to update position for loading a model
	void TargetModel(Model* model);

private:
	glm::vec3 updateCameraVectors();
};

