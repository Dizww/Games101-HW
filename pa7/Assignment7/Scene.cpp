//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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



Vector3f Scene::shade(const Intersection& hit,Vector3f wo) const
{
    if(hit.m->hasEmission()){
        return hit.m->getEmission();
    }
    const float EPSILON = 0.0005f;

    Vector3f p = hit.coords;
    Vector3f normal = normalize(hit.normal); 
    Material* m = hit.m;

    Vector3f L_dir;
    {
        Intersection inter_dir;
        float light_pdf;

        sampleLight(inter_dir,light_pdf);
        
        Vector3f& x = inter_dir.coords;
        Vector3f inter_normal = normalize(inter_dir.normal);
        
        Vector3f emit = inter_dir.emit;
        Vector3f ws = normalize(x - p);
        Intersection pws = intersect(Ray(p,ws));

        if((x - p).norm() - pws.distance < EPSILON){
            Vector3f f_r = m->eval(ws,wo,normal);
            float cosA = std::max(0.0f,dotProduct(ws,normal));
            float cosB = std::max(0.0f,dotProduct(-ws,inter_normal));
            float r2 = dotProduct((x-p),(x-p));
            L_dir = inter_dir.emit * f_r * cosA * cosB / r2 / light_pdf;
            //std::cout << "L_dir.x:"<<L_dir.x<<std::endl;         
            if(L_dir.x < 0) L_dir.x = 0;
            if(L_dir.y < 0) L_dir.y = 0;
            if(L_dir.z < 0) L_dir.z = 0;
        }
    }
    
    Vector3f L_indir(0.0,0.0,0.0);
    {
        if(get_random_float() < RussianRoulette){
            Vector3f wi = normalize(m->sample(wo,normal));
            float pdf_hemi = m->pdf(wi,wo,normal);

            Intersection inter =  intersect(Ray(p,wi));
            if(inter.happened && !m->hasEmission()){
                Vector3f f_r = m->eval(wi,wo,normal);
                float cosA = std::max(0.0f,dotProduct(wi,normal));
                L_indir = shade(inter,-wi)*f_r*cosA/pdf_hemi/RussianRoulette;
                //std::cout << "L_indir.x:"<<L_indir.x<<std::endl;      
                if(L_indir.x < 0) L_indir.x = 0;
                if(L_indir.y < 0) L_indir.y = 0;
                if(L_indir.z < 0) L_indir.z = 0;
            }
        }
    }
    //std::cout << "L_indir.x:"<<L_indir.x << " L_dir.x" << L_dir.x << std::endl;
    //std::cout << "L_indir.y:"<<L_indir.y << " L_dir.y" << L_dir.y << std::endl;
    //std::cout << "L_indir.z:"<<L_indir.z << " L_dir.z" << L_dir.z << std::endl;
    return L_indir + L_dir;
}

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if(depth > this->maxDepth){
        return Vector3f(0.0,0.0,0.0);
    }

    Intersection hit = intersect(ray);
    
    if(!hit.happened)
        return Vector3f(0.0,0.0,0.0);
    
    return shade(hit,normalize(-ray.direction));
}