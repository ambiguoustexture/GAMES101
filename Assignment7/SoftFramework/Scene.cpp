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

    Intersection inter_ray = intersect(ray);
    if (inter_ray.emit.norm() > 0) 
    {
        color = Vector3f(1);
    }
    else if (inter_ray.happened)
    {
        // Attributes of shading point
        Vector3f p  = inter_ray.coords,
                 wo = -ray.direction,
                 N  = normalize(inter_ray.normal);
        Material* m = inter_ray.m;

        // Sample light
        Intersection inter_light;
        float pdf_light;
        sampleLight(inter_light, pdf_light);
        Vector3f x = inter_light.coords,
                 ws = normalize(x - p),
                 NN = normalize(inter_light.normal);
        
        // Direct illumination
        Vector3f L_dir(0);
        Intersection inter_dir = intersect(Ray(p, ws));
        if ((float)(inter_dir.coords - inter_light.coords).norm() < 0.01)
        {
            L_dir = inter_light.emit 
                  * m->eval(ws, wo, N) 
                  * dotProduct(ws, N) 
                  * dotProduct(-ws, NN)
                  / dotProduct(x - p, x - p) 
                  / pdf_light;
        }

        // Indirect illumination
        Vector3f L_indir(0);
        if (get_random_float() < RussianRoulette)
        {
            Vector3f wi = m->sample(wo, N),
                     indir_shade_color = castRay(Ray(p, wi), depth);
            L_indir = indir_shade_color 
                    * m->eval(wi, wo, N) 
                    * dotProduct(wi, N) 
                    / m->pdf(wi, wo, N) 
                    / RussianRoulette;
        }
        L_indir = L_indir * 1.0 / RussianRoulette;
        color = L_dir + L_indir;
    }
    return color; 
    // DONE
}
