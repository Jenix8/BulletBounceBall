#pragma once

#ifndef BALL_H
#define BALL_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

//// Default ball values
//const glm::vec3 BallPosition(-3.0f, 5.0f, -3.0f);     
//const glm::vec3 BallVelocity(0.5f, 0.0f, 0.5f);    

class Ball
{
public:
    // Ball Attributes
    glm::vec3 center;
    float radius;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    float elasticity;

    // constructor with vectors
    Ball(glm::vec3 cen, float rad, glm::vec3 vel, glm::vec3 acc, float ela) : 
        center(cen), radius(rad), velocity(vel), acceleration(acc), elasticity(ela) {};

    void UpdateBall(float deltaTime) {
        bool is_hit = hitBox();
        updateBallVectors(is_hit, deltaTime);
    }

private:
    // Hit check
    bool hitBox() {

    }

    // calculates the front vector from the Ball's values
    void updateBallVectors(bool is_hit, float deltaTime) {
        if (is_hit)
            velocity.y *= -elasticity; // 일단 y축만..

        center += deltaTime * velocity;
        velocity += deltaTime * acceleration;
    }
};
#endif