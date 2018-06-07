#include "Camera.h"

Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch) : 
	Target(glm::vec3(0.0f, 0.0f, 0.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), 
	TrackPan(false), TrackMov(false), XPos(0), YPos(0)
{
	Position = position;
	WorldUp = up;
	Yaw = yaw;
	Pitch = pitch;
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
		// Can't rotate and move at the same time, return early
		return;
	}
	
	if (TrackMov || TrackPan) {
		Yaw = xpos - XPos;
		Pitch = ypos - YPos;

		if (constrainPitch)
		{
			if (Pitch > 0.5f)
				Pitch = 0.5f;
			if (Pitch < -0.499f)
				Pitch = -0.499f;
		}

		if (TrackPan) {
			Position = updateCameraVectors();
			return;
		}
		Target = updateCameraVectors();
	}
}

void Camera::ProcessMouseScroll(float yoffset)
{
	// Move Position closer to Target
	Distance = Distance + MouseSensitivity * yoffset;
	Position = Target + glm::normalize(Position - Target) * glm::exp2(Distance);
}

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
	
	return newPos * glm::length(Target - Position);
}