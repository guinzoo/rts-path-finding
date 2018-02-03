
namespace nook {
    
    enum class IntersectResult {
        Outside,
        Inside,
        Intersecting
    };
    
    struct Point2 {
        S32 x;
        S32 y;
        
        Point2() = default;
        Point2(const Point2&) = default;
        Point2(S32 x, S32 y) : x(x), y(y) {}
        Point2(const vec2& p) : x(p.x), y(p.y) {}
        void set(S32 x, S32 y) { Point2::x = x; Point2::y = y; }
        
        bool operator==(Point2 p) { return x == p.x && y == p.y; }
        bool operator!=(Point2 p) { return x != p.x || y != p.y; }
        
        Point2& operator=(const Point2& p) = default;
        Point2& operator=(const vec2& p) { x = p.x; y = p.y; return *this; }
        
        Point2& operator+=(const Point2& p) { x += p.x; y += p.y; return *this; }
        Point2& operator-=(const Point2& p) { x -= p.x; y -= p.y; return *this; }
        Point2& operator+=(int d) { x += d; y += d; return *this; }
        Point2& operator-=(int d) { x -= d; y -= d; return *this; }
        Point2& operator*=(int d) { x *= d; y *= d; return *this; }
        Point2& operator/=(int d) { x /= d; y /= d; return *this; }
    };
    
    struct Point3 {
        S32 x;
        S32 y;
        S32 z;
        
        Point3() {}
        Point3(S32 x, S32 y, S32 z) : x(x), y(y), z(z) {}
        void set(S32 x, S32 y, S32 z) { Point3::x = x; Point3::y = y; Point3::z = z; }
    };
    
    struct Size {
        S32 width;
        S32 height;
        
        Size() {}
        Size(S32 w, S32 h) { width = w; height = h; }
        void set(S32 w, S32 h) { width = w; height = h; }
        
        bool operator==(const Size& size) { return width == size.width && height == size.height; }
        bool operator!=(const Size& size) { return width != size.width || height != size.height; }
        Size& operator=(int i) { width = height = i; return *this; }
        Size& operator+=(int d) { width += d; height += d; return *this; }
        Size& operator-=(int d) { width -= d; height -= d; return *this; }
        Size& operator+=(Size s) { width += s.width; height += s.height; return *this; }
        Size& operator-=(Size s) { width -= s.width; height -= s.height; return *this; }
        Size& operator*=(int d) { width *= d; height *= d; return *this; }
        Size& operator/=(int d) { width /= d; height /= d; return *this; }
    };
    
    inline Size operator+(Size sz, int d);
    inline Size operator-(Size sz, int d);
    inline Size operator+(Size s1, Size s2);
    inline Size operator-(Size s1, Size s2);
    inline Size operator*(Size sz, int d);
    inline Size operator/(Size sz, int d);
    
    struct Circle {
        vec2 pos;
        float radius;
        
        Circle() = default;
        Circle(const vec2& pos, float radius) : pos(pos), radius(radius) {}
    };
    
    struct Rect {
        S32 x;
        S32 y;
        S32 width;
        S32 height;
        
        Rect() {}
        Rect(S32 x, S32 y, S32 width, S32 height) : x(x), y(y), width(width), height(height) {}
        
        void set(S32 x, S32 y, S32 width, S32 height);
        bool contains(const Rect& rect) const;
    };
    
    struct Rectf {
        vec2 origin;
        vec2 end;
        
        Rectf() {}
        Rectf(float x, float y, float endX, float endY) : origin(x, y), end(endX, endY) {}
        Rectf(const vec2& origin, const vec2& end) : origin(origin), end(end) {}
        
        void set(float x, float y, float endX, float endY);
        
        bool intersects(const Rectf& rect) const;
        bool contains(vec2 p) const;
        bool contains(const Rectf& rect) const;
        bool contains(const Circle& circle) const;
    };
    
    struct Ray {
        vec3 origin;
        vec3 dir;
        vec3 inv_dir;
        int sign[3];
        
        Ray() = default;
        Ray(const vec3& origin, const vec3& dir);
    };
    
    struct Box {
        vec3 bounds[2];
        
        Box() = default;
        Box(const vec3& bmin, const vec3& bmax);
        
        vec3 center() const { return (bounds[0] + bounds[1]) * 0.5f; }
        vec2 centerXZ() const { return vec2((bounds[0].x + bounds[1].x) * 0.5f, (bounds[0].z + bounds[1].z) * 0.5f); }
        Rectf boundsXZ() const { return Rectf(bounds[0].x, bounds[0].z, bounds[1].x, bounds[1].z); }
        
        bool intersects(const Ray& ray, float t0, float t1) const;
    };
    
    struct Triangle {
        vec3 vert[3];
        
        Triangle() = default;
        Triangle(const vec3& v1, const vec3& v2, const vec3& v3);
        
        const vec3& operator[](int i) const { return vert[i]; }
        vec3& operator[](int i) { return vert[i]; }
        
        vec3 barycentric(vec3 p) const;
        float getY(vec2 p) const;
        bool intersects(const Ray& ray, float& t, float& u, float& v) const;
    };
    
    struct Sphere {
        vec3 pos;
        float radius;
        
        Sphere() = default;
        Sphere(const vec3& pos, float radius) : pos(pos), radius(radius) {}
        
        void combine(const vec3& p);
        void combine(const Sphere& s);
        bool intersects(const Ray& ray) const;
        
        static Sphere fromPoints(vec3* pts, U32 count);
    };
    
    struct Geometry {
        enum class Type {
            Box,
            Sphere,
        } type;
        
        union {
            Box box;
            Sphere sphere;
        };
        
        void setBox(const Box& box);
        void setSphere(const vec3& pos, float radius);
        
    };
    
    inline float sign(vec2 p1, vec2 p2, vec2 p3);
    inline bool pointInTriangle(vec2 pt, vec2 v1, vec2 v2, vec2 v3);
    inline vec2 pointToLine(vec2 p, vec2 l1, vec2 l2);
    inline bool lineIntersect(vec2 p0, vec2 p1, vec2 p2, vec2 p3, vec2& i);
    inline bool circleLineIntersect(const Circle& c, vec2 l1, vec2 l2);
    
    inline void frustumPlanes(const mat4& m, vec4 planes[6]);
    inline IntersectResult planeAABBIntersect(const vec4& plane, const Box& box);
    inline IntersectResult frustumAABBIntersect(const vec4 planes[6], const Box& box);
    inline bool frustumSphereIntersect(const vec4 planes[6], const Sphere& sphere);
    
    
#pragma mark - Implementation
    
    inline Size operator+(Size sz, int d) {
        return sz += d;
    }
    
    inline Size operator-(Size sz, int d) {
        return sz -= d;
    }
    
    inline Size operator+(Size s1, Size s2) {
        return s1 += s2;
    }
    
    inline Size operator-(Size s1, Size s2) {
        return s1 -= s2;
    }
    
    inline Size operator*(Size sz, int d) {
        return sz *= d;
    }
    
    inline Size operator/(Size sz, int d) {
        return sz /= d;
    }
    
    inline void Rect::set(S32 x, S32 y, S32 width, S32 height) {
        Rect::x = x;
        Rect::y = y;
        Rect::width = width;
        Rect::height = height;
    }
    
    inline bool Rect::contains(const Rect& rect) const {
        return (rect.x >= x) && (rect.y >= y) && (rect.x + rect.width <= x + width) &&
               (rect.y + rect.height <= y + height);
    }
    
    inline void Rectf::set(float x, float y, float endX, float endY) {
        origin.set(x, y);
        end.set(endX, endY);
    }
    
    inline bool Rectf::intersects(const Rectf& rect) const {
        return origin.x <= rect.end.x && rect.origin.x <= end.x && origin.y <= rect.end.y && rect.origin.y <= end.y;
    }
    
    inline bool Rectf::contains(vec2 p) const {
        return origin.x <= p.x && end.x >= p.x && origin.y <= p.y && end.y >= p.y;
    }
    
    inline bool Rectf::contains(const Rectf& rect) const {
        return origin.x <= rect.origin.x && origin.y <= rect.origin.y && end.x >= rect.end.x && end.y >= rect.end.y;
    }
    
    inline bool Rectf::contains(const Circle& circle) const {
        return contains(Rectf(circle.pos - circle.radius, circle.pos + circle.radius));
    }
    
    inline Ray::Ray(const vec3& origin, const vec3& dir) {
        Ray::origin = origin;
        Ray::dir = dir;
        inv_dir = 1.0f / dir;
        sign[0] = dir.x < 0.0f;
        sign[1] = dir.y < 0.0f;
        sign[2] = dir.z < 0.0f;
    }
    
    inline Box::Box(const vec3& bmin, const vec3& bmax) {
        bounds[0] = bmin;
        bounds[1] = bmax;
    }
    
    inline bool Box::intersects(const Ray& ray, float t0, float t1) const {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        tmin = (bounds[ray.sign[0]].x - ray.origin.x) * ray.inv_dir.x;
        tmax = (bounds[1 - ray.sign[0]].x - ray.origin.x) * ray.inv_dir.x;
        tymin = (bounds[ray.sign[1]].y - ray.origin.y) * ray.inv_dir.y;
        tymax = (bounds[1 - ray.sign[1]].y - ray.origin.y) * ray.inv_dir.y;
        if ((tmin > tymax) || (tymin > tmax))
            return false;
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        tzmin = (bounds[ray.sign[2]].z - ray.origin.z) * ray.inv_dir.z;
        tzmax = (bounds[1 - ray.sign[2]].z - ray.origin.z) * ray.inv_dir.z;
        if ((tmin > tzmax) || (tzmin > tmax))
            return false;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        return ((tmin < t1) && (tmax > t0));
    }
    
    inline Triangle::Triangle(const vec3& v1, const vec3& v2, const vec3& v3) {
        vert[0] = v1;
        vert[1] = v2;
        vert[2] = v3;
    }
    
    inline vec3 Triangle::barycentric(vec3 p) const {
        vec3 v0 = vert[1] - vert[0];
        vec3 v1 = vert[2] - vert[0];
        vec3 v2 = p - vert[0];
        float d00 = v0.dot(v0);
        float d01 = v0.dot(v1);
        float d11 = v1.dot(v1);
        float d20 = v2.dot(v0);
        float d21 = v2.dot(v1);
        float invDenom = 1.0f / (d00 * d11 - d01 * d01);
        float v = (d11 * d20 - d01 * d21) * invDenom;
        float w = (d00 * d21 - d01 * d20) * invDenom;
        float u = 1.0f - v - w;
        return vec3(v, w, u);
    }
    
    inline float Triangle::getY(vec2 p) const {
        float invDet = 1.0f / ((vert[1].z - vert[2].z) * (vert[0].x - vert[2].x) +
                               (vert[2].x - vert[1].x) * (vert[0].z - vert[2].z));
        
        float l1 = ((vert[1].z - vert[2].z) * (p.x - vert[2].x) + (vert[2].x - vert[1].x) * (p.y - vert[2].z)) * invDet;
        float l2 = ((vert[2].z - vert[0].z) * (p.x - vert[2].x) + (vert[0].x - vert[2].x) * (p.y - vert[2].z)) * invDet;
        float l3 = 1.0f - l1 - l2;
        
        return l1 * vert[0].y + l2 * vert[1].y + l3 * vert[2].y;
    }
    
    inline bool Triangle::intersects(const Ray& ray, float& t, float& u, float& v) const {
        vec3 edge1, edge2, tvec, pvec, qvec;
        float det, inv_det;
        
        edge1 = vert[1] - vert[0];
        edge2 = vert[2] - vert[0];
        pvec = ray.dir.cross(edge2);
        det = edge1.dot(pvec);
        tvec = ray.origin - vert[0];
        inv_det = 1.0f / det;
        
//            if (det > 0.00001f) { //CW
//                u = tvec.dot(pvec);
//                if (u < 0.0f || u > det)
//                    return false;
//                qvec = tvec.cross(edge1);
//                v = ray.dir.dot(qvec);
//                if (v < 0.0f || u + v > det)
//                    return false;
//            }
//            else
        if (det < -0.00001f) { //CCW
            u = tvec.dot(pvec);
            if (u > 0.0f || u < det)
                return false;
            qvec = tvec.cross(edge1);
            v = ray.dir.dot(qvec) ;
            if (v > 0.0f || u + v < det)
                return false;
        }
        else
            return false;
        
        t = edge2.dot(qvec) * inv_det;
        u *= inv_det;
        v *= inv_det;
        return true;
    }
    
    inline void Sphere::combine(const vec3& p) {
        vec3 v = p - pos;
        float d = v.length();
        if (d > radius) {
            float a = (d - radius) * 0.5f;
            pos += v * (a / d);
            radius += a;
        }
    }
    
    inline void Sphere::combine(const Sphere& s) {
        const Sphere& maxSphere = s.radius > radius ? s : *this;
        const Sphere& minSphere = s.radius > radius ? *this : s;
        
        float newRadius = (maxSphere.radius + minSphere.radius + vec3(minSphere.pos - maxSphere.pos).length()) * 0.5f;
        if (newRadius <= maxSphere.radius) {
            pos = maxSphere.pos;
            radius = maxSphere.radius;
        }
        else {
            vec3 norm = (minSphere.pos - maxSphere.pos).normalized();
            pos = maxSphere.pos + norm * (newRadius - maxSphere.radius);
            radius = newRadius;
        }
    }
    
    inline bool Sphere::intersects(const nook::Ray& ray) const {
        vec3 l = pos - ray.origin;
        float s = l.dot(ray.dir);
        float l2 = l.dot(l);
        float r2 = radius * radius;
        if (s < 0 && l2 > r2)
            return false;
        float m2 = l2 - s * s;
        return m2 <= r2;
    }
    
    inline Sphere Sphere::fromPoints(vec3* pts, U32 count) {
        vec3* end = pts + count;
        vec3 a = *pts;
        vec3 b = *pts;
        float d2 = 0.0f;
        for (vec3* p = pts + 1; p != end; p++) {
            float pd = (*p - a).length2();
            if (pd > d2) {
                d2 = pd;
                b = *p;
            }
        }
        for (vec3* p = pts; p != end; p++) {
            float pd = (*p - b).length2();
            if (pd > d2) {
                d2 = pd;
                a = *p;
            }
        }
        vec3 pos = (a + b) * 0.5f;
        float r = sqrt(d2) * 0.5f;
        float r2 = r * r;
        for (vec3* p = pts; p != end; p++) {
            vec3 v = *p - pos;
            float d2 = v.length2();
            if (d2 > r2) {
                float d = sqrt(d2);
                float a = (d - r) * 0.5f;
                pos += v * (a / d);
                r += a;
                r2 = r * r;
            }
        }
        return Sphere(pos, r);
    }
    
    inline void Geometry::setBox(const Box& box) {
        type = Type::Box;
        Geometry::box = box;
    }
    
    inline void Geometry::setSphere(const vec3& pos, float radius) {
        type = Type::Sphere;
        sphere.pos = pos;
        sphere.radius = radius;
    }
    
    // >0 is left
    inline float sign(vec2 p, vec2 v1, vec2 v2) {
        return (p.x - v2.x) * (v1.y - v2.y) - (v1.x - v2.x) * (p.y - v2.y);
    }
    
    inline bool pointInTriangle(vec2 pt, vec2 v1, vec2 v2, vec2 v3) {
        bool b1, b2, b3;
        b1 = sign(pt, v1, v2) < 0.0f;
        b2 = sign(pt, v2, v3) < 0.0f;
        b3 = sign(pt, v3, v1) < 0.0f;
        return ((b1 == b2) && (b2 == b3));
    }
    
    inline vec2 pointToLine(vec2 p, vec2 l1, vec2 l2) {
        vec2 v = l2 - l1;
        vec2 w = p - l1;
        
        float c1 = w.dot(v);
        if (c1 <= 0)
            return l1 - p;
        
        float c2 = v.dot(v);
        if (c2 <= c1)
            return l2 - p;
        
        float b = c1 / c2;
        vec2 pb = l1 + b * v;
        return pb - p;
    }
    
    inline void frustumPlanes(const mat4& m, vec4 planes[6]) {
        planes[0] = (m.row(3) + m.row(0)).normalize();  // left
        planes[1] = (m.row(3) - m.row(0)).normalize();  // right
        planes[2] = (m.row(3) + m.row(1)).normalize();  // bottom
        planes[3] = (m.row(3) - m.row(1)).normalize();  // top
        planes[4] = (m.row(3) + m.row(2)).normalize();  // near
        planes[5] = (m.row(3) - m.row(2)).normalize();  // far
    }
    
    inline bool lineIntersect(vec2 p0, vec2 p1, vec2 p2, vec2 p3, vec2& i) {
        vec2 s1 = p1 - p0;
        vec2 s2 = p3 - p2;
        float d = -s2.x * s1.y + s1.x * s2.y;
        if (d == 0.0f)
            return false;
        d = 1.0f / d;
        float s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) * d;
        float t = ( s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) * d;
        if (s >= 0.0f && s <= 1.0f && t >= 0.0f && t <= 1.0f) {
            i.x = p0.x + (t * s1.x);
            i.y = p0.y + (t * s1.y);
            return true;
        }
        return false;
    }
    
    // works only when l1 is outside
    inline bool circleLineIntersectOutside(const Circle& circle, vec2 l1, vec2 l2, vec2& i) {
        vec2 d = l2 - l1;
        vec2 f = l1 - circle.pos;
        float a = d.length2();
        float b = 2.0f * f.dot(d);
        float c = f.length2() - circle.radius * circle.radius;
        
        float dis = b * b - 4.0f * a * c;
        if (dis < 0.0f)
            return false;
        
        dis = std::sqrt(dis);
        float t = (-b - dis) / (a * 2.0f);
        if (t >= 0.0f && t <= 1.0f) {
            i = l1 + d * t;
            return true;
        }
        return false;
    }
    
    inline bool circleLineIntersect(const Circle& circle, vec2 l1, vec2 l2, vec2& i) {
        vec2 d = l2 - l1;
        vec2 f = l1 - circle.pos;
        float a = d.length2();
        float b = 2.0f * f.dot(d);
        float c = f.length2() - circle.radius * circle.radius;
        
        float dis = b * b - 4.0f * a * c;
        if (dis < 0.0f)
            return false;
        
        dis = std::sqrt(dis);
        a = 0.5f / a;
        float t1 = (-b - dis) * a;
        float t2 = (-b + dis) * a;
        
        if (t1 >= 0.0f && t1 <= 1.0f) {
            i = l1 + d * t1;
            return true;
        }
        if (t2 >= 0.0f && t2 <= 1.0f) {
            i = l1 + d * t2;
            return true;
        }
        return false;
    }
    
    inline IntersectResult planeAABBIntersect(const vec4& plane, const Box& box) {
        int sx = plane.x > 0.0f;
        int sy = plane.y > 0.0f;
        int sz = plane.z > 0.0f;
        
        float dot = plane.x * box.bounds[sx].x + plane.y * box.bounds[sy].y + plane.z * box.bounds[sz].z;
        if (dot < -plane.w)
            return IntersectResult::Outside;
        
        float dot2 = plane.x * box.bounds[1-sx].x + plane.y * box.bounds[1-sy].y + plane.z * box.bounds[1-sz].z;
        if (dot2 < -plane.w)
            return IntersectResult::Intersecting;
        
        return IntersectResult::Inside;
    }
    
    inline IntersectResult frustumAABBIntersect(const vec4 planes[6], const Box& box) {
        IntersectResult result = IntersectResult::Inside;
        for (int i = 0; i < 6; i++) {
            const vec4& plane = planes[i];
            int sx = plane.x > 0.0f;
            int sy = plane.y > 0.0f;
            int sz = plane.z > 0.0f;
            
            float dot = plane.x * box.bounds[sx].x + plane.y * box.bounds[sy].y + plane.z * box.bounds[sz].z;
            if (dot < -plane.w)
                return IntersectResult::Outside;
            
            float dot2 = plane.x * box.bounds[1-sx].x + plane.y * box.bounds[1-sy].y + plane.z * box.bounds[1-sz].z;
            if (dot2 < -plane.w)
                result = IntersectResult::Intersecting;
        }
        return result;
    }
    
    inline bool frustumSphereIntersect(const vec4 planes[6], const Sphere& sphere) {
        for (int i = 0; i < 6; i++) {
            const vec4& plane = planes[i];
            float side = sphere.pos.dot(plane) + plane.w;
            if (side < -sphere.radius)
                return false;
        }
        return true;
    }
    
}
