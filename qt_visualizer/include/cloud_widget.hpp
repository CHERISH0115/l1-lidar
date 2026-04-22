#pragma once
#include <QWidget>
#include <QMutex>
#include <QMetaType>
#include <vector>

struct Point3D { float x, y, z, intensity; };
Q_DECLARE_METATYPE(std::vector<Point3D>)

class CloudWidget : public QWidget {
    Q_OBJECT
public:
    explicit CloudWidget(QWidget *parent = nullptr);

public slots:
    void update_cloud(const std::vector<Point3D> &pts);
    void clear_map();
    void set_max_frames(int n);
    void reset_view();
    bool export_pcd(const QString &path);

public:
    int point_count() const { return int(cloud_.size()); }

protected:
    void paintEvent(QPaintEvent *) override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;

private:
    QMutex mutex_;
    std::vector<Point3D> cloud_;

    float azimuth_{225.0f};   // horizontal rotation (degrees)
    float elevation_{30.0f};  // vertical tilt (degrees)
    float scale_{80.0f};      // pixels per metre
    float z_min_{0.0f}, z_max_{1.0f};
    int   max_frames_{200};

    QPoint last_mouse_;
    bool rotating_{false};

    QPointF project(float wx, float wy, float wz) const;
};
