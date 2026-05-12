#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMutex>
#include <QMetaType>
#include <vector>

struct Point3D { float x, y, z, intensity; };
Q_DECLARE_METATYPE(std::vector<Point3D>)

class CloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit CloudWidget(QWidget *parent = nullptr);
    ~CloudWidget() override;

public slots:
    void update_cloud(const std::vector<Point3D> &pts);
    void add_trajectory_point(float x, float y, float z);
    void clear_map();
    void set_max_points(int n);
    void set_max_frames(int n);
    void reset_view();
    bool export_pcd(const QString &path);
    void set_show_trajectory(bool v);

public:
    int point_count() const { return int(cloud_.size()); }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;

private:
    void rebuild_cloud_vbo();
    void rebuild_traj_vbo();
    void render_grid();

    QMutex mutex_;
    std::vector<Point3D> cloud_;
    std::vector<Point3D> trajectory_;
    bool show_trajectory_{true};
    bool dirty_cloud_{false};
    bool dirty_traj_{false};

    float azimuth_{225.0f};
    float elevation_{30.0f};
    float scale_{80.0f};
    float z_min_{0.0f}, z_max_{1.0f};
    size_t max_points_{600000};

    QPoint last_mouse_;
    bool rotating_{false};

    // VBOs
    unsigned int vbo_cloud_{0};
    unsigned int vbo_traj_{0};
    unsigned int vbo_grid_{0};
    unsigned int vbo_axes_{0};
    int vbo_cloud_count_{0};
    int vbo_traj_count_{0};

    // Shaders
    unsigned int prog_cloud_{0};
    unsigned int prog_line_{0};
    unsigned int prog_grid_{0};
};
