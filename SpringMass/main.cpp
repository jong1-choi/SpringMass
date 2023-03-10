#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"
#include <glm/gtx/quaternion.hpp>

#define WIDTH 20
#define HEIGHT 20
#define SIZE 2


const glm::vec3 G = {0,-980,0};
const float k_drag = 0.05f;
const float alpha = 0.8f;
const float mu = 10;
const float k_d = 0.01;
const float default_K_spring = 800;

struct plane {
    glm::vec3 p;
    glm::vec3 n;
    void render() const {
        drawQuad(p,n,{10000,10000});
    }
};
struct sphere {
    glm::vec3 c;
    float r;
    void render() const {
        drawSphere(c,r);
    }
};

const plane ground = {{0,0,0},{0,1,0}};
const sphere obstacle = {{0,30,-5},30};

struct mass {
    float m;
    glm::vec3 x, v, f;
    bool fixed = false;
    
    void fix() {
        fixed = !fixed;
    }
    void clearForce() {
        f = glm::vec3(0);
    }
    
    void addForce( const glm::vec3& force) {
        f += force;
    }
    
    void step( float dt ) {
        if( !fixed ) {
            v += (f/m)*dt;
            x += v*dt;
        }
        else {
            v = glm::vec3(0);
        }
    }
    bool isContacting( const plane& p) const {
        if( dot(x-p.p,p.n)<0.001 && abs(dot(v,p.n))< 30.f)
            return true;
        return false;
    }
    bool isContacting( const sphere& s) const {
        if( length(s.c-x) < s.r+0.0001f && abs(dot(v,normalize(x - s.c))) < 30.f)
            return true;
        return false;
    }
    void resolveContact( const plane& p, float dt ) {
        if( isContacting(p) ) {
            glm::vec3 fN = dot(f,p.n)*p.n;
            glm::vec3 vN = dot(v,p.n)*p.n;
            glm::vec3 vT = v-vN;
            glm::vec3 fF = -mu * length(fN) * normalize(vT);
            if( length(fF)*dt > length(vT)*m )
                fF = -vT/dt*m;
            addForce(fF);
            addForce(-fN);
        }
    }
    void resolveContact( const sphere& s, float dt ) {
        if( isContacting(s) ) {
            glm::vec3 n = normalize(x - s.c);
            glm::vec3 fN = dot(f,n)*n;
            glm::vec3 vN = dot(v,n)*n;
            glm::vec3 vT = v-vN;
            glm::vec3 fF = -mu * length(fN) * normalize(vT);
            if( length(fF)*dt > length(vT)*m )
                fF = -vT/dt*m;
            addForce(fF);
            addForce(-fN);
        }
    }
    void resolveCollision( const plane& p) {
        if( dot(x-p.p,p.n)<0.00001 && dot(v,p.n)< 0 ) {
            glm::vec3 vN = dot(v,p.n)*p.n;
            glm::vec3 vT = v-vN;
            v = vT - alpha*vN;
            x = x - dot(x-p.p, p.n)*p.n;
        }
    }
    void resolveCollision( const sphere& s) {
        if( length(s.c-x) < s.r+0.0001f && dot(v,x - s.c) < 0) {
            glm::vec3 n = normalize(x - s.c);
            glm::vec3 vN = dot(v,n)*n;
            glm::vec3 vT = v-vN;
            v = vT - 0.8f*vN;
        }
    }
    void render() const {
        drawSphere(x,1);
    }
};

struct spring {
    mass& m1;
    mass& m2;
    float r;
    float k;
    
    spring( mass& mm1, mass& mm2, float K = default_K_spring )
    : m1(mm1), m2(mm2), r(length(mm1.x - mm2.x)), k(K) {}
    void addForce() {
        glm::vec3 F = -(k*(length(m2.x-m1.x)-r)+k_d*dot(m2.v-m1.v,normalize(m2.x-m1.x)))*normalize(m2.x-m1.x);
        m1.addForce( -F );
        m2.addForce( F );
    }
    void render() const {
        drawCylinder(m1.x,m2.x,0.5);
    }
};

std::vector<mass> masses;
std::vector<spring> springs;

float randf() {
    return rand()/(float)RAND_MAX;
}

void keyFunc(int key) {
    if( key == '1' )
        masses[0].fix();
    if( key == '2' )
        masses[19].fix();
}
    
void init() {
    masses.clear();
    springs.clear();
    
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH; x++){
            masses.push_back({0.01,{(x-1)*SIZE,100-(y-1)*SIZE,randf()*0.1},{0,0,0}});
        }
    }
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH-1; x++){
            springs.push_back(spring(masses[y*HEIGHT+x],masses[y*HEIGHT+x+1]));
        }
    }
    for(int y = 0; y < HEIGHT-1; y++){
        for(int x = 0; x < WIDTH; x++){
            springs.push_back(spring(masses[y*HEIGHT+x],masses[(y+1)*HEIGHT+x]));
        }
    }
    for(int y = 0; y < HEIGHT-1; y++){
        for(int x = 0; x < WIDTH-1; x++){
            springs.push_back(spring(masses[(y)*HEIGHT+x],masses[(y+1)*HEIGHT+x+1],200));
        }
    }
    for(int y = 0; y < HEIGHT-1; y++){
        for(int x = 0; x < WIDTH-1; x++){
            springs.push_back(spring(masses[(y+1)*HEIGHT+x],masses[(y)*HEIGHT+x+1],200));
        }
    }
    for(int y = 0; y < HEIGHT-1; y++){
        for(int x = 0; x < WIDTH-1; x++){
            springs.push_back(spring(masses[(y)*HEIGHT+x],masses[(y+1)*HEIGHT+x+1],200));
        }
    }
    for(int y = 0; y < HEIGHT-1; y++){
        for(int x = 0; x < WIDTH-1; x++){
            springs.push_back(spring(masses[(y+1)*HEIGHT+x],masses[(y)*HEIGHT+x+1],200));
        }
    }
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH-2; x++){
            springs.push_back(spring(masses[y*HEIGHT+x],masses[y*HEIGHT+x+2],100));
        }
    }
    for(int y = 0; y < HEIGHT-2; y++){
        for(int x = 0; x < WIDTH; x++){
            springs.push_back(spring(masses[y*HEIGHT+x],masses[(y+2)*HEIGHT+x],100));
        }
    }
    masses[0].fix();
    masses[19].fix();
}

void frame( float dt ) {
    for( int i = 0; i < 100; i++){
        for( auto& m: masses )
            m.clearForce();
        for( auto& m: masses ) {
            m.addForce( m.m*G );
            m.addForce( -k_drag*m.v );
        }
        for( auto& s: springs) {
            s.addForce();
        }
        for( auto& m: masses ) {
            m.resolveContact( ground, dt/100);
            m.resolveContact( obstacle, dt/100);
        }
        for( auto& m: masses ) {
            m.step(dt/100);
            m.resolveCollision( ground );
            m.resolveCollision( obstacle );
        }
    }
}

void render() {
    ground.render();
    obstacle.render();
    
    for( auto& s: springs)
        s.render();
    for( auto& m: masses)
        m.render();
}

int main(int argc, const char * argv[]) {
    JGL::Window* window = new JGL::Window(800,600,"simulation");
    window->alignment(JGL::ALIGN_ALL);
    AnimView* animView = new AnimView(0,0,800,600);
    animView->renderFunction = render;
    animView->frameFunction = frame;
    animView->initFunction = init;
    animView->keyFunction = keyFunc;
    init();
    window->show();
    JGL::_JGL::run();
    return 0;
}
