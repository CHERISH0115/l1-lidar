#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QSpinBox>
#include "cloud_widget.hpp"
#include "ros_worker.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_clear_clicked();
    void on_reset_view_clicked();
    void on_max_frames_changed(int v);
    void on_export_clicked();
    void update_fps();

private:
    CloudWidget  *cloud_widget_;
    QLabel       *lbl_status_;
    QLabel       *lbl_fps_;
    QLabel       *lbl_points_;

    QPushButton  *btn_clear_;
    QPushButton  *btn_reset_;
    QPushButton  *btn_export_;
    QSpinBox     *spin_frames_;

    RosWorker  *ros_worker_;
    QThread    *ros_thread_;
    QTimer     *fps_timer_;

    int frame_count_{0};
};
