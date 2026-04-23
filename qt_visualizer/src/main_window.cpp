#include "main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QThread>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <cmath>

static void style_btn(QPushButton *b, const QString &color) {
    b->setStyleSheet(QString(
        "QPushButton {"
        "  background:%1; color:white; border:none;"
        "  border-radius:5px; padding:8px 0; font-size:13px; font-weight:bold;"
        "}"
        "QPushButton:hover { background:%1; opacity:0.85; }"
        "QPushButton:pressed { padding-top:10px; }"
    ).arg(color));
    b->setMinimumHeight(36);
}

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent) {
    setWindowTitle("L1 LiDAR SLAM Visualizer");
    resize(1200, 760);

    setStyleSheet("QMainWindow,QWidget{background:#1a1a2e;color:#e0e0e0;}"
                  "QGroupBox{border:1px solid #3a3a5c;border-radius:6px;"
                  "          margin-top:10px;padding-top:8px;font-weight:bold;}"
                  "QGroupBox::title{subcontrol-origin:margin;left:10px;color:#7f8fff;}"
                  "QLabel{color:#c0c0d0;}"
                  "QCheckBox{color:#c0c0d0;}"
                  "QSpinBox{background:#2a2a40;color:#e0e0e0;border:1px solid #4a4a6c;"
                  "         border-radius:4px;padding:4px;}");

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *root_h = new QHBoxLayout(central);
    root_h->setContentsMargins(8,8,8,8);
    root_h->setSpacing(8);

    cloud_widget_ = new CloudWidget(this);
    root_h->addWidget(cloud_widget_, 1);

    // ── 右侧面板 ──────────────────────────────────────────
    auto *panel = new QWidget(this);
    panel->setFixedWidth(210);
    panel->setStyleSheet("background:#13132a;border-radius:8px;");
    auto *pv = new QVBoxLayout(panel);
    pv->setContentsMargins(10,10,10,10);
    pv->setSpacing(10);
    root_h->addWidget(panel);

    // 状态卡片
    auto *grp = new QGroupBox("SLAM Status", panel);
    auto *gv  = new QVBoxLayout(grp);
    gv->setSpacing(6);
    lbl_fps_    = new QLabel("FPS: --",    grp);
    lbl_points_ = new QLabel("Map pts: --", grp);
    lbl_status_ = new QLabel("Waiting for SLAM...", grp);
    lbl_status_->setWordWrap(true);
    lbl_fps_->setStyleSheet("color:#7fff7f;font-size:14px;font-weight:bold;");
    lbl_points_->setStyleSheet("color:#7fbfff;font-size:13px;");
    lbl_status_->setStyleSheet("color:#ffcf7f;font-size:12px;");
    gv->addWidget(lbl_fps_);
    gv->addWidget(lbl_points_);
    gv->addWidget(lbl_status_);
    pv->addWidget(grp);

    // 地图密度控制
    auto *grp2 = new QGroupBox("Map Density", panel);
    auto *g2v  = new QVBoxLayout(grp2);
    g2v->addWidget(new QLabel("Max frames (buffer):"));
    spin_frames_ = new QSpinBox(grp2);
    spin_frames_->setRange(100, 5000);
    spin_frames_->setValue(500);
    spin_frames_->setSuffix(" fr");
    g2v->addWidget(spin_frames_);
    pv->addWidget(grp2);

    // 显示控制
    auto *grp4 = new QGroupBox("Display", panel);
    auto *g4v  = new QVBoxLayout(grp4);
    chk_traj_ = new QCheckBox("Show trajectory", grp4);
    chk_traj_->setChecked(true);
    g4v->addWidget(chk_traj_);
    pv->addWidget(grp4);

    // 操作按钮
    auto *grp3 = new QGroupBox("Controls", panel);
    auto *g3v  = new QVBoxLayout(grp3);
    g3v->setSpacing(8);
    btn_clear_  = new QPushButton("Clear Map",   grp3);
    btn_reset_  = new QPushButton("Reset View",  grp3);
    btn_export_ = new QPushButton("Export PCD",  grp3);
    style_btn(btn_clear_,  "#c0392b");
    style_btn(btn_reset_,  "#2980b9");
    style_btn(btn_export_, "#27ae60");
    g3v->addWidget(btn_clear_);
    g3v->addWidget(btn_reset_);
    g3v->addWidget(btn_export_);
    pv->addWidget(grp3);

    auto *hint = new QLabel("Drag: rotate\nScroll: zoom\nLeft btn: orbit", panel);
    hint->setStyleSheet("color:#606080;font-size:11px;");
    hint->setAlignment(Qt::AlignCenter);
    pv->addWidget(hint);

    pv->addStretch();

    // ── ROS 线程 ──────────────────────────────────────────
    ros_worker_ = new RosWorker();
    ros_thread_ = new QThread(this);
    ros_worker_->moveToThread(ros_thread_);

    connect(ros_thread_, &QThread::started, ros_worker_,
            [this, argc, argv]() { ros_worker_->start_ros(argc, argv); });

    // 注册点云 → 全局地图（FAST-LIO2 已在世界坐标系）
    connect(ros_worker_, &RosWorker::cloud_received,
            cloud_widget_, &CloudWidget::update_cloud,
            Qt::QueuedConnection);
    connect(ros_worker_, &RosWorker::cloud_received,
            this, [this](const std::vector<Point3D> &) {
                frame_count_++;
            }, Qt::QueuedConnection);

    // 里程计 → 轨迹点
    connect(ros_worker_, &RosWorker::pose_received,
            this, [this](float x, float y, float z, float /*yaw*/) {
                cloud_widget_->add_trajectory_point(x, y, z);
            }, Qt::QueuedConnection);
    connect(ros_worker_, &RosWorker::status_received,
            lbl_status_, &QLabel::setText, Qt::QueuedConnection);

    connect(btn_clear_,  &QPushButton::clicked, cloud_widget_, &CloudWidget::clear_map);
    connect(btn_reset_,  &QPushButton::clicked, cloud_widget_, &CloudWidget::reset_view);
    connect(btn_export_, &QPushButton::clicked, this, &MainWindow::on_export_clicked);
    connect(spin_frames_, QOverload<int>::of(&QSpinBox::valueChanged),
            cloud_widget_, &CloudWidget::set_max_frames);
    connect(chk_traj_, &QCheckBox::toggled,
            cloud_widget_, &CloudWidget::set_show_trajectory);

    fps_timer_ = new QTimer(this);
    connect(fps_timer_, &QTimer::timeout, this, &MainWindow::update_fps);
    fps_timer_->start(1000);

    ros_thread_->start();
}

MainWindow::~MainWindow() {
    ros_worker_->stop_ros();
    ros_thread_->quit();
    ros_thread_->wait();
    delete ros_worker_;
}

void MainWindow::on_export_clicked() {
    QString default_name = QString("map_%1.pcd")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString path = QFileDialog::getSaveFileName(
        this, "Export Map", QDir::homePath() + "/" + default_name,
        "PCD Files (*.pcd);;All Files (*)");
    if (path.isEmpty()) return;

    btn_export_->setEnabled(false);
    btn_export_->setText("Exporting...");

    bool ok = cloud_widget_->export_pcd(path);

    btn_export_->setEnabled(true);
    btn_export_->setText("Export PCD");

    if (ok)
        QMessageBox::information(this, "Export OK",
            QString("Saved %1 points to:\n%2")
            .arg(cloud_widget_->point_count()).arg(path));
    else
        QMessageBox::warning(this, "Export Failed",
            "No points to export or file write error.");
}

void MainWindow::on_clear_clicked()           { cloud_widget_->clear_map(); }
void MainWindow::on_reset_view_clicked()      { cloud_widget_->reset_view(); }
void MainWindow::on_max_frames_changed(int v) { cloud_widget_->set_max_frames(v); }

void MainWindow::update_fps() {
    lbl_fps_->setText(QString("FPS: %1").arg(frame_count_));
    lbl_points_->setText(QString("Map pts: %1").arg(cloud_widget_->point_count()));
    frame_count_ = 0;
}
