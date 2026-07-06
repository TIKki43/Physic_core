#include "RayTracer.h"

void RayTracer::AddRay(const Ray &Ray)
{
    Rays.push_back(Ray);
}

void RayTracer::RaySimulationStep(float Deltatime)
{
    for (Ray& ray : Rays) {
        if (ray.Active){
            ray.AdvanceByTime(Deltatime);
        }
    }
}
