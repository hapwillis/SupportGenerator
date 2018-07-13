#include "Camera.h"

Camera::Camera(float fov, int screenWidth, int screenHeight, glm::vec3 position, glm::vec3 up, float yaw, float pitch) :
	Target(glm::vec3(0.0f, 0.0f, 0.0f)), MouseSensitivity(SENSITIVITY), 
	TrackPan(false), TrackMov(false), XPos(0), YPos(0)
{
	WorldUp = up;
	Yaw = yaw; 
	Pitch = pitch;
	Hfov = fov;
	Vfov = fov * screenHeight / screenWidth;
	//log base 2 of distance keeps scroll speed reasonable
	Distance = glm::log2(glm::length(Target - position)); 
	Position = Target + updateCameraVectors();
}

Camera::~Camera()
{
}

// Returns a viewMatrix to transform world coordinates to perspective coordinates
glm::mat4 Camera::GetViewMatrix() {
	glm::vec3 pos = Target + updateCameraVectors();
	Position = pos;
	return glm::lookAt(pos, Target, WorldUp);
}

// Initiates a pan (camera swing) and tracks mouse coordinates
void Camera::StartPan(bool start, float xpos, float ypos) {
	if (!TrackMov) {
		TrackPan = start;
		XPos = xpos - Yaw;
		YPos = ypos - Pitch;
	}
}

// Initiates a camera movement and tracks mouse coordinates
void Camera::StartMov(bool start, float xpos, float ypos) {
	if (!TrackPan) {
		TrackMov = start;
		XPos = xpos;
		YPos = ypos;
	}
}

// called each frame to update camera position if needed
void Camera::ProcessMouseMovement(float xpos, float ypos, GLboolean constrainPitch) {
	if (TrackMov && TrackPan) {
		XPos = xpos;
		YPos = ypos;
		// Not useful to rotate and move at the same time, return early
		return;
	}
	
	if (TrackPan) {
		Yaw = xpos - XPos;
		Pitch = ypos - YPos;

		if (constrainPitch)
		{
			if (Pitch > 0.5f)
				Pitch = 0.5f;
			if (Pitch < -0.499f)
				Pitch = -0.499f;
		}
	}

	if (TrackMov) {
		// Update Target and Position by right and up vectors
		glm::vec3 deltaH = Right * (xpos - XPos) * glm::exp2(Distance) * tan(Hfov);
		glm::vec3 deltaV = Up * (ypos - YPos) * glm::exp2(Distance) * tan(Vfov);
		glm::vec3 delta = deltaH + deltaV;
		Target = Target + delta;
		XPos = xpos;
		YPos = ypos;
	}
}

// Controls the camera's distance from the target
void Camera::ProcessMouseScroll(float yoffset) {
	// Move Position closer to Target
	Distance = Distance + MouseSensitivity * yoffset;
}

// Returns the direction the camera is looking at
glm::vec3 Camera::Direction() {
	return glm::normalize(updateCameraVectors());
}

// Sets the camera to look at a model from a pleasant angle
void Camera::TargetModel(Model* model) {
	float modelRadius = model->BoundingSphere();
	Target = glm::vec3(0, .66*modelRadius, 0);
	Distance = glm::log2(2.5 * modelRadius);
}

// converts pitch and yaw values (which are created from mouse movement) to a new position
glm::vec3 Camera::updateCameraVectors() {
	// Calculate the new position
	glm::vec3 newPos;
	newPos.x = cos(glm::pi<float>() * 2 * (Yaw)) * cos(glm::pi<float>() * (-Pitch + 1.0f));
	newPos.y = sin(glm::pi<float>() * (-Pitch + 1.0f));
	newPos.z = sin(glm::pi<float>() * 2 * (Yaw)) * cos(glm::pi<float>() * (-Pitch + 1.0f));

	// Also re-calculate the Right and Up vector
	Right = glm::normalize(glm::cross(newPos, WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	Up = glm::normalize(glm::cross(Right, newPos));

	return newPos * glm::exp2(Distance);
}