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

void RayTracer::SchwarzschildSimulationStep(Body& MassiveBody)
{
    for (Ray& ray : Rays) {
        if (ray.Active){
            ray.SchwarzschildStep(MassiveBody);
        }
    }
}
