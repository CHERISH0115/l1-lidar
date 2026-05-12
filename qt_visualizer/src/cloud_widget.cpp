#include "cloud_widget.hpp"
#include <QWheelEvent>
#include <QMouseEvent>
#include <QMutexLocker>

#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdio>

// ── shader source (GLES 2.0 / desktop GL 2.1) ───────────────────────

static const char *VS_CLOUD = R"(
attribute vec3 a_pos;
attribute vec3 a_color;
uniform mat4 u_mvp;
varying vec3 v_color;
void main() {
    v_color = a_color;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    gl_PointSize = 2.0;
}
)";

static const char *FS_CLOUD = R"(
varying vec3 v_color;
void main() {
    gl_FragColor = vec4(v_color, 1.0);
}
)";

static const char *VS_LINE = R"(
attribute vec3 a_pos;
uniform mat4 u_mvp;
uniform vec3 u_color;
varying vec3 v_color;
void main() {
    v_color = u_color;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)";

static const char *FS_LINE = R"(
varying vec3 v_color;
void main() {
    gl_FragColor = vec4(v_color, 1.0);
}
)";

// ── helpers ──────────────────────────────────────────────────────────

static unsigned int compile_shader(unsigned int type, const char *src) {
    unsigned int s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        fprintf(stderr, "[CloudWidget] shader compile error: %s\n", log);
    }
    return s;
}

static unsigned int link_program(const char *vs_src, const char *fs_src) {
    unsigned int vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, fs_src);
    unsigned int p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    int ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        fprintf(stderr, "[CloudWidget] program link error: %s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

// Build color-per-vertex array from cloud
static void build_cloud_vbo(const std::vector<Point3D> &pts,
                            float z_min, float z_max,
                            std::vector<float> &out) {
    out.clear();
    out.reserve(pts.size() * 6);
    float zr = (z_max > z_min) ? (z_max - z_min) : 1.0f;
    for (const auto &p : pts) {
        float t = std::clamp((p.z - z_min) / zr, 0.0f, 1.0f);
        float r, g, b;
        if      (t < 0.25f) { float s=t/0.25f;         r=0;     g=s;   b=1; }
        else if (t < 0.5f)  { float s=(t-0.25f)/0.25f; r=0;     g=1;   b=1-s; }
        else if (t < 0.75f) { float s=(t-0.5f)/0.25f;  r=s;     g=1;   b=0; }
        else                { float s=(t-0.75f)/0.25f;  r=1;     g=1-s; b=0; }
        out.push_back(p.x); out.push_back(p.y); out.push_back(p.z);
        out.push_back(r);   out.push_back(g);   out.push_back(b);
    }
}

static void build_grid_vbo(std::vector<float> &verts, std::vector<float> &colors) {
    verts.clear(); colors.clear();
    for (int i = -10; i <= 10; ++i) {
        verts.insert(verts.end(), {float(i), -10.0f, 0.0f, float(i), 10.0f, 0.0f});
        verts.insert(verts.end(), {-10.0f, float(i), 0.0f, 10.0f, float(i), 0.0f});
        for (int j = 0; j < 4; ++j) {
            colors.push_back(0.18f); colors.push_back(0.18f); colors.push_back(0.22f);
        }
    }
}

static void build_axes_vbo(std::vector<float> &verts, std::vector<float> &colors) {
    verts.clear(); colors.clear();
    // X red
    verts.insert(verts.end(), {0,0,0, 1,0,0});
    colors.insert(colors.end(), {1,0,0, 1,0,0});
    // Y green
    verts.insert(verts.end(), {0,0,0, 0,1,0});
    colors.insert(colors.end(), {0,1,0, 0,1,0});
    // Z blue
    verts.insert(verts.end(), {0,0,0, 0,0,1});
    colors.insert(colors.end(), {0,0,1, 0,0,1});
}

// ── matrix math (MVP) ────────────────────────────────────────────────
static void identity(float m[16]) {
    for (int i = 0; i < 16; ++i) m[i] = 0;
    m[0] = m[5] = m[10] = m[15] = 1;
}

static void multiply(float r[16], const float a[16], const float b[16]) {
    float t[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            t[i*4+j] = 0;
            for (int k = 0; k < 4; ++k)
                t[i*4+j] += a[i*4+k] * b[k*4+j];
        }
    for (int i = 0; i < 16; ++i) r[i] = t[i];
}

static void perspective(float m[16], float fov, float aspect, float n, float f) {
    identity(m);
    float t = tanf(fov / 2.0f);
    m[0]  = 1.0f / (aspect * t);
    m[5]  = 1.0f / t;
    m[10] = -(f + n) / (f - n);
    m[11] = -1.0f;
    m[14] = -2.0f * f * n / (f - n);
    m[15] = 0.0f;
}

static void look_at(float m[16], float ex, float ey, float ez,
                    float tx, float ty, float tz,
                    float ux, float uy, float uz) {
    float f[3] = {tx - ex, ty - ey, tz - ez};
    float flen = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (flen > 0) { f[0] /= flen; f[1] /= flen; f[2] /= flen; }
    float s[3] = {f[1]*uz - f[2]*uy, f[2]*ux - f[0]*uz, f[0]*uy - f[1]*ux};
    float slen = sqrtf(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
    if (slen > 0) { s[0] /= slen; s[1] /= slen; s[2] /= slen; }
    float u[3] = {s[1]*f[2] - s[2]*f[1], s[2]*f[0] - s[0]*f[2], s[0]*f[1] - s[1]*f[0]};
    m[0] = s[0]; m[4] = s[1]; m[8]  = s[2]; m[12] = -s[0]*ex - s[1]*ey - s[2]*ez;
    m[1] = u[0]; m[5] = u[1]; m[9]  = u[2]; m[13] = -u[0]*ex - u[1]*ey - u[2]*ez;
    m[2] =-f[0]; m[6] =-f[1]; m[10] =-f[2]; m[14] =  f[0]*ex + f[1]*ey + f[2]*ez;
    m[3] = 0;    m[7] = 0;    m[11] = 0;    m[15] = 1;
}

// ── implementation ───────────────────────────────────────────────────

CloudWidget::CloudWidget(QWidget *parent) : QOpenGLWidget(parent) {
    setMinimumSize(600, 500);
}

CloudWidget::~CloudWidget() {
    makeCurrent();
    if (vbo_cloud_) glDeleteBuffers(1, &vbo_cloud_);
    if (vbo_traj_)  glDeleteBuffers(1, &vbo_traj_);
    if (vbo_grid_)  glDeleteBuffers(1, &vbo_grid_);
    if (vbo_axes_)  glDeleteBuffers(1, &vbo_axes_);
    if (prog_cloud_) glDeleteProgram(prog_cloud_);
    if (prog_line_)  glDeleteProgram(prog_line_);
    if (prog_grid_)  glDeleteProgram(prog_grid_);
    doneCurrent();
}

void CloudWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.06f, 0.06f, 0.10f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // programs
    prog_cloud_ = link_program(VS_CLOUD, FS_CLOUD);
    prog_line_  = link_program(VS_LINE,  FS_LINE);
    prog_grid_  = link_program(VS_LINE,  FS_LINE); // same shader

    // static VBO: grid
    std::vector<float> gv, gc;
    build_grid_vbo(gv, gc);
    glGenBuffers(1, &vbo_grid_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_grid_);
    // interleave verts + colors
    std::vector<float> grid_interleaved;
    grid_interleaved.reserve(gv.size() * 2);
    for (size_t i = 0; i < gv.size() / 3; ++i) {
        grid_interleaved.push_back(gv[i*3]);   grid_interleaved.push_back(gv[i*3+1]); grid_interleaved.push_back(gv[i*3+2]);
        grid_interleaved.push_back(gc[i*3]);   grid_interleaved.push_back(gc[i*3+1]); grid_interleaved.push_back(gc[i*3+2]);
    }
    glBufferData(GL_ARRAY_BUFFER, grid_interleaved.size() * sizeof(float), grid_interleaved.data(), GL_STATIC_DRAW);

    // static VBO: axes
    std::vector<float> av, ac;
    build_axes_vbo(av, ac);
    glGenBuffers(1, &vbo_axes_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_axes_);
    std::vector<float> axes_interleaved;
    axes_interleaved.reserve(av.size() * 2);
    for (size_t i = 0; i < av.size() / 3; ++i) {
        axes_interleaved.push_back(av[i*3]);   axes_interleaved.push_back(av[i*3+1]); axes_interleaved.push_back(av[i*3+2]);
        axes_interleaved.push_back(ac[i*3]);   axes_interleaved.push_back(ac[i*3+1]); axes_interleaved.push_back(ac[i*3+2]);
    }
    glBufferData(GL_ARRAY_BUFFER, axes_interleaved.size() * sizeof(float), axes_interleaved.data(), GL_STATIC_DRAW);

    // dynamic VBOs
    glGenBuffers(1, &vbo_cloud_);
    glGenBuffers(1, &vbo_traj_);
}

void CloudWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void CloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int w = width(), h = height() ? height() : 1;
    float aspect = float(w) / float(h);

    // MVP
    float proj[16], view[16], mvp[16];
    perspective(proj, 60.0f * float(M_PI) / 180.0f, aspect, 0.1f, 2000.0f);

    // Orbit camera: start at origin, apply azimuth/elevation rotation, move back by distance
    float dist = scale_ * 0.05f;
    float az = azimuth_   * float(M_PI) / 180.0f;
    float el = elevation_ * float(M_PI) / 180.0f;
    float cx = dist * cosf(el) * sinf(az);
    float cy = dist * cosf(el) * cosf(az);
    float cz = dist * sinf(el);
    float ux = 0, uy = 0, uz = 1;
    // Tilt up vector for elevation
    if (fabs(el) < 1.5f) { ux = -sinf(el)*sinf(az); uy = -sinf(el)*cosf(az); uz = cosf(el); }

    look_at(view, cx, cy, cz, 0, 0, 0, ux, uy, uz);
    multiply(mvp, proj, view);

    QMutexLocker lk(&mutex_);

    // ── grid ────────────────────────────────────────────────────────
    {
        glUseProgram(prog_grid_);
        glUniformMatrix4fv(glGetUniformLocation(prog_grid_, "u_mvp"), 1, GL_FALSE, mvp);
        glUniform3f(glGetUniformLocation(prog_grid_, "u_color"), 0.18f, 0.18f, 0.22f);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_grid_);
        GLint a_pos = glGetAttribLocation(prog_grid_, "a_pos");
        glEnableVertexAttribArray(a_pos);
        glVertexAttribPointer(a_pos, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
        glDrawArrays(GL_LINES, 0, 84); // 21 lines × 2 verts × 4 per line
        glDisableVertexAttribArray(a_pos);
    }

    // ── axes ────────────────────────────────────────────────────────
    {
        glUseProgram(prog_line_);
        glUniformMatrix4fv(glGetUniformLocation(prog_line_, "u_mvp"), 1, GL_FALSE, mvp);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_axes_);
        GLint a_pos = glGetAttribLocation(prog_line_, "a_pos");
        GLint u_col = glGetUniformLocation(prog_line_, "u_color");
        glEnableVertexAttribArray(a_pos);
        glVertexAttribPointer(a_pos, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
        // X (verts 0-1) red
        glUniform3f(u_col, 1, 0, 0); glDrawArrays(GL_LINES, 0, 2);
        // Y (verts 2-3) green
        glUniform3f(u_col, 0, 1, 0); glDrawArrays(GL_LINES, 2, 2);
        // Z (verts 4-5) blue
        glUniform3f(u_col, 0, 0, 1); glDrawArrays(GL_LINES, 4, 2);
        glDisableVertexAttribArray(a_pos);
    }

    // ── point cloud ─────────────────────────────────────────────────
    if (dirty_cloud_) rebuild_cloud_vbo();
    if (vbo_cloud_count_ > 0) {
        glUseProgram(prog_cloud_);
        glUniformMatrix4fv(glGetUniformLocation(prog_cloud_, "u_mvp"), 1, GL_FALSE, mvp);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_cloud_);
        GLint a_pos = glGetAttribLocation(prog_cloud_, "a_pos");
        GLint a_col = glGetAttribLocation(prog_cloud_, "a_color");
        glEnableVertexAttribArray(a_pos);
        glEnableVertexAttribArray(a_col);
        glVertexAttribPointer(a_pos, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
        glVertexAttribPointer(a_col, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                              (void*)(3*sizeof(float)));
        glDrawArrays(GL_POINTS, 0, vbo_cloud_count_);
        glDisableVertexAttribArray(a_pos);
        glDisableVertexAttribArray(a_col);
    }

    // ── trajectory ──────────────────────────────────────────────────
    if (show_trajectory_ && dirty_traj_) rebuild_traj_vbo();
    if (show_trajectory_ && vbo_traj_count_ >= 2) {
        glUseProgram(prog_line_);
        glUniformMatrix4fv(glGetUniformLocation(prog_line_, "u_mvp"), 1, GL_FALSE, mvp);
        glUniform3f(glGetUniformLocation(prog_line_, "u_color"), 1.0f, 0.24f, 0.24f);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_traj_);
        GLint a_pos = glGetAttribLocation(prog_line_, "a_pos");
        glEnableVertexAttribArray(a_pos);
        glVertexAttribPointer(a_pos, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), nullptr);
        glDrawArrays(GL_LINE_STRIP, 0, vbo_traj_count_);
        // current position (large point)
        glPointSize(10.0f);
        glDrawArrays(GL_POINTS, vbo_traj_count_ - 1, 1);
        glDisableVertexAttribArray(a_pos);
    }

    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void CloudWidget::rebuild_cloud_vbo() {
    std::vector<float> verts;
    build_cloud_vbo(cloud_, z_min_, z_max_, verts);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_cloud_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
    vbo_cloud_count_ = int(cloud_.size());
    dirty_cloud_ = false;
}

void CloudWidget::rebuild_traj_vbo() {
    std::vector<float> tv;
    tv.reserve(trajectory_.size() * 3);
    for (const auto &p : trajectory_)
        tv.insert(tv.end(), {p.x, p.y, p.z});
    glBindBuffer(GL_ARRAY_BUFFER, vbo_traj_);
    glBufferData(GL_ARRAY_BUFFER, tv.size() * sizeof(float), tv.data(), GL_DYNAMIC_DRAW);
    vbo_traj_count_ = int(trajectory_.size());
    dirty_traj_ = false;
}

// ── slots ────────────────────────────────────────────────────────────

void CloudWidget::update_cloud(const std::vector<Point3D> &pts) {
    QMutexLocker lk(&mutex_);
    cloud_.insert(cloud_.end(), pts.begin(), pts.end());

    if (cloud_.size() > max_points_)
        cloud_.erase(cloud_.begin(),
                     cloud_.begin() + long(cloud_.size() - max_points_));

    if (!cloud_.empty()) {
        z_min_ = z_max_ = cloud_[0].z;
        for (const auto &p : cloud_) {
            if (p.z < z_min_) z_min_ = p.z;
            if (p.z > z_max_) z_max_ = p.z;
        }
    }
    dirty_cloud_ = true;
    update();
}

void CloudWidget::add_trajectory_point(float x, float y, float z) {
    QMutexLocker lk(&mutex_);
    trajectory_.push_back({x, y, z, 0.0f});
    dirty_traj_ = true;
    update();
}

void CloudWidget::clear_map() {
    QMutexLocker lk(&mutex_);
    cloud_.clear();
    trajectory_.clear();
    dirty_cloud_ = true;
    dirty_traj_  = true;
    update();
}

void CloudWidget::set_max_points(int n) {
    QMutexLocker lk(&mutex_);
    max_points_ = size_t(n);
}

void CloudWidget::set_max_frames(int n) {
    set_max_points(n * 2000);
}

void CloudWidget::set_show_trajectory(bool v) {
    show_trajectory_ = v;
    update();
}

void CloudWidget::reset_view() {
    azimuth_   = 225.0f;
    elevation_ =  30.0f;
    scale_     =  80.0f;
    update();
}

bool CloudWidget::export_pcd(const QString &path) {
    QMutexLocker lk(&mutex_);
    if (cloud_.empty()) return false;

    std::ofstream f(path.toStdString());
    if (!f) return false;

    f << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
      << "WIDTH "  << cloud_.size() << "\nHEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << cloud_.size() << "\nDATA ascii\n";
    f << std::fixed;
    f.precision(4);
    for (const auto &p : cloud_)
        f << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
    return f.good();
}

void CloudWidget::wheelEvent(QWheelEvent *e) {
    float factor = (e->angleDelta().y() > 0) ? 1.15f : 0.87f;
    scale_ = std::clamp(scale_ * factor, 5.0f, 2000.0f);
    update();
}

void CloudWidget::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        rotating_   = true;
        last_mouse_ = e->pos();
    }
}

void CloudWidget::mouseMoveEvent(QMouseEvent *e) {
    if (rotating_) {
        QPoint d = e->pos() - last_mouse_;
        last_mouse_ = e->pos();
        azimuth_   -= d.x() * 0.5f;
        elevation_ += d.y() * 0.5f;
        elevation_  = std::clamp(elevation_, -89.0f, 89.0f);
        update();
    }
}

void CloudWidget::mouseReleaseEvent(QMouseEvent *) { rotating_ = false; }
