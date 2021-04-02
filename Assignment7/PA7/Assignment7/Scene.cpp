//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#define EPSILON 1e-4

void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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
    // TODO Implement Path Tracing Algorithm here
    Intersection its;
    its = intersect(ray);
    float pdf;
    Vector3f p(its.coords);
    
    Vector3f L_dir(0, 0, 0);
    Vector3f L_indir(0, 0, 0);

    //light source
    if (its.happened)
    {
        if (its.m->hasEmission())
        {
            return its.m->getEmission();
        }
        Intersection light_sample;
        sampleLight(light_sample, pdf);
        Vector3f x(light_sample.coords);
        Vector3f ws = normalize(light_sample.coords - its.coords);
        Vector3f wo = (-ray.direction).normalized();
        Ray ray_p2x = Ray(p, ws);

        Intersection collode = Scene::intersect(ray_p2x);
        // if the ray not blocked in the middle

        // 不同判断是否与光源相交的方法，会导致有很大区别
        // bool hasBlocked = ((p-x).norm() - collode.distance) > EPSILON;
        if (collode.happened && collode.m->hasEmission())//directed light
        {   
            // return light_sample.emit;
            L_dir = light_sample.emit * its.m->eval(wo, ws, its.normal) 
            * dotProduct(ws, its.normal) * dotProduct(-ws, light_sample.normal)
            / dotProduct((p-x), (p-x))
            / pdf;
        }
        else //RR
        {
            // float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            bool RR_fire = get_random_float() > Scene::RussianRoulette;

            if (!RR_fire)
            {
                Vector3f wi = its.m->sample(wo, its.normal).normalized();
                Intersection indir_inter =  Scene::intersect(Ray(p, wi));
                if (indir_inter.happened && !indir_inter.m->hasEmission())
                {
                    L_indir = Scene::castRay(Ray(indir_inter.coords, wi), 0)
                        * indir_inter.m->eval(wo, wi, its.normal)
                        * dotProduct(wi, its.normal)
                        / indir_inter.m->pdf(wo, wi, its.normal)
                        / Scene::RussianRoulette;
                }
            }
        }
    }
    // hit test
    // Intersection intersection = Scene::intersect(ray);
    // Vector3f hitColor = this->backgroundColor;
    // Material *m = intersection.m;
    // Object *hitObject = intersection.obj;
    // if (intersection.happened) {
    //     return Vector3f(1.0f);
    // }
    // else {
    //     return Vector3f(0.0f);
    // }

    return L_dir + L_indir;
}