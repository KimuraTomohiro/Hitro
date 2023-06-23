#ifndef Q_MOC_RUN
    #include <ros/ros.h>
    #include <rviz/panel.h>
#endif

class TestPanel: public rviz::Panel {
Q_OBJECT //継承時に必要なマクロ
public:
    TestPanel(QWidget* Parent = 0); //コンストラクタ
};