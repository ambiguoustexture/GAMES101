//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() 
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) 
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) 
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) 
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) 
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }
    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // DONE: Implement Path Tracing Algorithm here
    
    Vector3f color(0);
    
    // sampleLight(inter, pdf_light)
    Intersection inter_ray = intersect(ray);
    if (inter_ray.emit.norm() > 0) 
    {
        color = Vector3f(1);
    }
    else if (inter_ray.happened) 
    {
        Vector3f  p  = inter_ray.coords;
        Vector3f  wo = -ray.direction;
        Vector3f  N  = normalize(inter_ray.normal);
        Material* m  = inter_ray.m;

        Intersection inter; 
        float pdf_light; sampleLight(inter, pdf_light);
    
        // Get x, ws, NN, emit from inter 
        Vector3f x    = inter.coords;
        Vector3f ws   = normalize(x - p);
        Vector3f NN   = normalize(inter.normal);
        Vector3f emit = inter.emit;
    
        // Shoot a ray from p to x
        Intersection inter_dir = intersect(Ray(p, ws));
    
        // If the ray is not blocked in the middle
        Vector3f L_dir(0);
        if ((float)(inter_dir.coords - inter.coords).norm() < 0.01) 
        {
            L_dir = emit * m->eval(ws, wo, N) * dotProduct(ws, N) / dotProduct(-ws, NN) 
                  / dotProduct(x - p, x - p) / pdf_light;
        }
    
        // L_indir = 0.0
        Vector3f L_indir(0);
    
        // Test Russian Roulette with probability RussianRoulette
        // If ray r hit a non-emitting object
        if (get_random_float() < RussianRoulette) 
        {
            // wi = sample(wo, N)
            Vector3f wi = m->sample(wo, N);
            
            // Trace a ray r(p, wi)
            L_indir = castRay(Ray(p, wi), depth) * m->eval(wi, wo, N) 
                    * dotProduct(wi, N) / m->pdf(wi, wo, N) / RussianRoulette;
        }
        L_indir = L_indir * 1.0 / RussianRoulette;
        color = L_dir + L_indir;
    } 
    // Return L_dir + L_indir
    return color;
    // DONE
}
