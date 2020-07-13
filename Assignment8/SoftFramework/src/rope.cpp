#include <iostream>
#include <vector>

#include "../CGL/src/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL 
{
    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, 
            float k, vector<int> pinned_nodes)
    {
        // DONE (Part 1): 
        // Create a rope starting at `start`, ending at `end`, 
        // and containing `num_nodes` nodes.

        Vector2D step_vec = (end - start) / (num_nodes - 1.0);

        Vector2D curr_node = start;
        Mass*    curr_mass = new Mass(curr_node, node_mass, false); masses.push_back(curr_mass);
        Spring*  curr_spring = nullptr;

        Vector2D prev_node;
        Mass*    prev_mass = nullptr;

        for (auto cnt = 1; cnt < num_nodes; cnt++)
        {
            curr_node = cnt != num_nodes - 1 ? start + cnt  * step_vec : end;
            
            prev_mass = curr_mass;
            curr_mass = new Mass(curr_node, node_mass, false);
            masses.push_back(curr_mass);

            curr_spring = new Spring(prev_mass, curr_mass, k);
            springs.push_back(curr_spring);
        }
        
        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) 
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // DONE (Part 2): Use Hooke's law to calculate the force on a node
            auto vec_b2a = s->m2->position - s->m1->position;
            auto f_b2a   =  s->k * vec_b2a / vec_b2a.norm() * (vec_b2a.norm() - s->rest_length); 
            s->m1->forces += f_b2a;
            s->m2->forces -= f_b2a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // DONE (Part 2): Add the force due to gravity, 
                // then compute the new velocity and position
                // DONE (Part 2): Add global damping

                m->forces += gravity;
                float damping_factor = 0.05;
                m->forces -= damping_factor * m->velocity;

                auto a = m->forces / m->mass;

                // For explicit method, 
                // update position with previous velocity
                // m->position += delta_t * m->velocity;
                // m->velocity += delta_t * a;

                // For semi-implicit method,
                // Use derivatives in the future, for the current step
                m->velocity += delta_t * a;
                m->position += delta_t * m->velocity;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope 
            // using explicit Verlet （solving constraints)
            auto vec_b2a = s->m2->position - s->m1->position;
            auto f_b2a   = s->k * vec_b2a / vec_b2a.norm() * (vec_b2a.norm() - s->rest_length);
            s->m1->forces += f_b2a;
            s->m2->forces -= f_b2a;
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                auto temp_position = m->position;
                // DONE (Part 3.1): Set the new position of the rope mass
                // DONE (Part 4): Add global Verlet damping
                m->forces += gravity;
                auto a = m->forces / m->mass;

                float damping_factor = 0.00005;
                m->position = m->position 
                            + (1 - damping_factor) * (m->position - m->last_position) 
                            + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}   
