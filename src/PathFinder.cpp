
#include "PathFinder.hpp"
#include "Map.hpp"
#include "scenes/GameScene.hpp"

#include "resource/ResourceManager.hpp"

using namespace nook;

PathFinder* PathFinder::s_instance;

PathFinder::PathFinder() {
    s_instance = this;
    m_size = map()->size();
    
    m_jpsPlus = memoryManager().createOnStack<JPSplus>();
    m_wallTracing = memoryManager().createOnStack<WallTracing>();
    
    // Visual Path
    m_pathObject.partCount = 1;
    m_pathObject.parts = memoryManager().allocOnStack<RenderParam>();
    m_pathObject.boundingSphere.pos = 0.0f;
    m_pathObject.boundingSphere.radius = m_size.width / 2;
    
    Mesh* mesh = resourceManager().findResource<Mesh>(SH("color-mesh"));
    MaterialDesc* md = (MaterialDesc*)mesh->materials();
    
    m_pathObject.parts->material.init(md);
    m_pathObject.parts->material.setUniform(SH("uModel"), &m_pathTransform);
    
    DynamicBuffer* buf = DynamicBuffer::create(KB(8), KB(1));
    m_pathObject.parts->vao = DynamicVAO::create(mesh->submeshes()->input, buf, 0);
    m_pathObject.parts->indexOffset = 0;
    m_pathObject.parts->mode = RenderMode::LineStrip;
    m_pathTransform = mat4::identity();
    
    U16* inds = buf->mapIbo();
    for (U16 i = 0; i < 512; i++)
        inds[i] = i;
    buf->unmapIbo();
}

PathFinder::~PathFinder() {
    m_wallTracing->~WallTracing();
    s_instance = nullptr;
    DynamicBuffer* buf = ((DynamicVAO*)m_pathObject.parts->vao)->buffer();
    memoryManager().remove(m_pathObject.parts->vao);
    memoryManager().remove(buf);
}

void PathFinder::findRough(vec2 start, vec2 end, Array<vec2>& path) {
    Point2 s = map()->getCoord(start);
    Point2 e = map()->getCoord(end);
    s.x = clamp(s.x, 1, m_size.width - 2);
    s.y = clamp(s.y, 1, m_size.height - 2);
    m_jpsPlus->find(Coord(s.x, s.y), Coord(e.x, e.y), path);
}

void PathFinder::showPath(Array<vec2>& path) {
    if (path.count() == 0)
        return;
    
    DynamicBuffer* buf = ((DynamicVAO*)m_pathObject.parts->vao)->buffer();
    vec3* verts = (vec3*)buf->mapVbo();
    for (vec2 p : path)
        *verts++ = vec3(p.x, 0.01f, p.y);
    buf->unmapVbo();
    m_pathObject.parts->indexCount = path.count();
    
    if (!m_pathObject.node)
        gameScene()->renderTree().add(&m_pathObject);
}
