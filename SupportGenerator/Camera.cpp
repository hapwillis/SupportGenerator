#include "Camera.h"

Camera::Camera(float fov, int screenWidth, int screenHeight, glm::vec3 position, glm::vec3 up, float yaw, float pitch) :
	Target(glm::vec3(0.0f, 0.0f, 0.0f)), MouseSensitivity(SENSITIVITY), 
	TrackPan(false), TrackMov(false), XPos(0), YPos(0)
{
	Position = position;
	WorldUp = up;
	Yaw = yaw; //compute these from the target location
	Pitch = pitch;
	Hfov = fov;
	Vfov = fov * screenHeight / screenWidth;
	Distance = glm::log2(glm::length(Target - Position)); //log base 2 of distance
	updateCameraVectors();
}

Camera::~Camera()
{
}

glm::mat4 Camera::GetViewMatrix()
{
	return glm::lookAt(Position, Target, Up);
}

void Camera::StartPan(bool start, float xpos, float ypos)
{
	if (!TrackMov) {
		TrackPan = start;
		XPos = xpos - Yaw;
		YPos = ypos - Pitch;
	}
}

void Camera::StartMov(bool start, float xpos, float ypos)
{
	if (!TrackPan) {
		TrackMov = start;
		XPos = xpos;
		YPos = ypos;
	}
}

void Camera::ProcessMouseMovement(float xpos, float ypos, GLboolean constrainPitch)
{
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

		Position = glm::length(Position) * updateCameraVectors();
	}

	if (TrackMov) {
		// Update Target and Position by right and up vectors
		float d = glm::length(Target - Position);
		glm::vec3 deltaH = Right * (XPos - xpos) * d * tan(Hfov);
		glm::vec3 deltaV = Up * (ypos - YPos) * d * tan(Vfov);
		glm::vec3 delta = deltaH + deltaV;
		Target = Target + delta;
		Position = Position + delta;
		XPos = xpos;
		YPos = ypos;
	}
}

void Camera::ProcessMouseScroll(float yoffset)
{
	// Move Position closer to Target
	Distance = Distance + MouseSensitivity * yoffset;
	Position = Target + glm::normalize(Position - Target) * glm::exp2(Distance);
}

glm::vec3 Camera::Direction()
{
	return glm::normalize(Target - Position);
}

void Camera::TargetModel(Model* model) 
{
	float modelRadius = model->BoundingSphere();
	Target = glm::vec3(0, modelRadius, 0);
	Position = glm::vec3(0, 1.5f * modelRadius, 1.5f * modelRadius);

	updateCameraVectors(); //update pan and pitch to match vectors instead
	// also update distance
}

// For perspective panning, apply this*glm::length(Target - Position) to the target
glm::vec3 Camera::updateCameraVectors()
{
	// Calculate the new position
	glm::vec3 newPos;
	newPos.x = cos(glm::pi<float>() * 2 * (Yaw)) * cos(glm::pi<float>() * (-Pitch + 1.0f));
	newPos.y = sin(glm::pi<float>() * (-Pitch + 1.0f));
	newPos.z = sin(glm::pi<float>() * 2 * (Yaw)) * cos(glm::pi<float>() * (-Pitch + 1.0f));

	// Also re-calculate the Right and Up vector
	Right = glm::normalize(glm::cross(Target - Position, WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	Up = glm::normalize(glm::cross(Right, Target - Position));
	
	return newPos;
}