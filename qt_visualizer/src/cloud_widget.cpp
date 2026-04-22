#include "cloud_widget.hpp"
#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QMutexLocker>
#include <cmath>
#include <algorithm>
#include <fstream>

CloudWidget::CloudWidget(QWidget *parent) : QWidget(parent) {
    setMinimumSize(600, 500);
}

// Orbital projection: rotate world by azimuth then elevation, then orthographic
QPointF CloudWidget::project(float wx, float wy, float wz) const {
    float az = azimuth_ * float(M_PI) / 180.0f;
    float el = elevation_ * float(M_PI) / 180.0f;

    float x1 =  wx * cosf(az) + wy * sinf(az);
    float y1 = -wx * sinf(az) + wy * cosf(az);
    float z1 = wz;

    float x2 = x1;
    float z2 = y1 * sinf(el) + z1 * cosf(el);

    return { width() / 2.0f + x2 * scale_,
             height() / 2.0f - z2 * scale_ };
}

void CloudWidget::set_max_frames(int n) {
    QMutexLocker lk(&mutex_);
    max_frames_ = n;
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

    f << "# .PCD v0.7 - Point Cloud Data\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH "  << cloud_.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << cloud_.size() << "\n"
      << "DATA ascii\n";

    f << std::fixed;
    f.precision(4);
    for (const auto &pt : cloud_)
        f << pt.x << ' ' << pt.y << ' ' << pt.z << ' ' << pt.intensity << '\n';

    return f.good();
}

void CloudWidget::update_cloud(const std::vector<Point3D> &pts) {
    QMutexLocker lk(&mutex_);
    // 滚动缓冲：保留最近 N 帧
    cloud_.insert(cloud_.end(), pts.begin(), pts.end());
    constexpr size_t PTS_PER_FRAME = 120;
    size_t max_pts = size_t(max_frames_) * PTS_PER_FRAME;
    if (cloud_.size() > max_pts)
        cloud_.erase(cloud_.begin(), cloud_.begin() + (cloud_.size() - max_pts));

    if (!cloud_.empty()) {
        z_min_ = z_max_ = cloud_[0].z;
        for (const auto &p : cloud_) {
            z_min_ = std::min(z_min_, p.z);
            z_max_ = std::max(z_max_, p.z);
        }
    }
    update();
}

void CloudWidget::clear_map() {
    QMutexLocker lk(&mutex_);
    cloud_.clear();
    update();
}

void CloudWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, false);

    QMutexLocker lk(&mutex_);

    // 深色背景
    p.fillRect(rect(), QColor(15, 15, 25));

    // 地面网格 (Z=0 平面)
    p.setPen(QPen(QColor(45, 45, 55), 1));
    for (int i = -10; i <= 10; ++i) {
        p.drawLine(project(float(i), -10, 0), project(float(i), 10, 0));
        p.drawLine(project(-10, float(i), 0), project(10, float(i), 0));
    }

    // 坐标轴
    p.setPen(QPen(Qt::red, 2));   p.drawLine(project(0,0,0), project(1,0,0));
    p.setPen(QPen(Qt::green, 2)); p.drawLine(project(0,0,0), project(0,1,0));
    p.setPen(QPen(Qt::blue, 2));  p.drawLine(project(0,0,0), project(0,0,1));

    // 点云：按Z高度着色 (蓝→青→绿→黄→红)
    float zrange = (z_max_ > z_min_) ? (z_max_ - z_min_) : 1.0f;
    for (const auto &pt : cloud_) {
        float t = std::clamp((pt.z - z_min_) / zrange, 0.0f, 1.0f);
        int r, g, b;
        if (t < 0.25f) {
            float s = t / 0.25f;
            r = 0; g = int(255 * s); b = 255;
        } else if (t < 0.5f) {
            float s = (t - 0.25f) / 0.25f;
            r = 0; g = 255; b = int(255 * (1 - s));
        } else if (t < 0.75f) {
            float s = (t - 0.5f) / 0.25f;
            r = int(255 * s); g = 255; b = 0;
        } else {
            float s = (t - 0.75f) / 0.25f;
            r = 255; g = int(255 * (1 - s)); b = 0;
        }
        auto s = project(pt.x, pt.y, pt.z);
        p.fillRect(QRectF(s.x() - 1.5, s.y() - 1.5, 3, 3), QColor(r, g, b));
    }

    // 提示文字
    p.setPen(QColor(120, 120, 140));
    p.drawText(8, 18, QString("Points: %1  Drag: rotate  Scroll: zoom")
               .arg(cloud_.size()));
}

void CloudWidget::wheelEvent(QWheelEvent *e) {
    float factor = (e->angleDelta().y() > 0) ? 1.15f : 0.87f;
    scale_ = std::clamp(scale_ * factor, 5.0f, 800.0f);
    update();
}

void CloudWidget::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        rotating_ = true;
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

void CloudWidget::mouseReleaseEvent(QMouseEvent *) {
    rotating_ = false;
}
