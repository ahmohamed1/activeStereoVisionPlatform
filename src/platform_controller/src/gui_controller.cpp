#include "QDebug"
#include "ros/ros.h"
#include <QApplication>
#include <QObject>
#include "QPushButton"
#include <QSlider>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <geometry_msgs/Vector3.h>
#include <QEvent>



float left_angle;

//Define the class
class gui_controller : public QObject
{


public:
    gui_controller();

  // this function use to be call when the value of the slide change to send to the controller
private Q_SLOTS:
  void updateLeftPan();

private:
  //define the storage of the publishe value
  geometry_msgs::Vector3 left_pan , right_pan;

  //define publisher
  ros::Publisher left_pan_pub;



  // QT difinitions
  QWidget * window;
  QPushButton *button_qt;
  QSpinBox *left_cam_sb;
  QSlider *left_cam_s;
  QLabel *left_label;

  QSpinBox *right_cam_sb;
  QSlider *right_cam_s;
  QLabel *right_label;
  QHBoxLayout *layout_right;

  QVBoxLayout *layout;
  QHBoxLayout *layout_baseline;
  QSpinBox *baseline_cam_sb ;
  QSlider *baseline_cam_s;
  QLabel *baseline_label;
  QHBoxLayout *layout_left;
};

////////////////////////////////////////////////////////////
/// Define the functions in the class
gui_controller::gui_controller()
{
  ros::NodeHandle nh;
  //define the publisher
  left_pan_pub = nh.advertise<geometry_msgs::Vector3> ("left/pan/move",50);

  window = new QWidget;
  window->setWindowTitle("Platform Controller");
  window->setFixedHeight(400);
  window->setFixedWidth(500);
  //define a bottum to quit the controller
  button_qt = new QPushButton("Quit");

  QObject::connect(button_qt,SIGNAL(clicked()),
		  window,SLOT(quit()));

  ////////////////////////////////////////////////////////////////////
  //Create the left cam slider and apinbox
  //define the spinbox and sliders
  left_cam_sb = new QSpinBox;
  left_cam_s = new QSlider(Qt::Horizontal);
  left_label = new QLabel("left pan motor");
  //set the limeted
  left_cam_sb->setRange(-70,70);
  left_cam_s->setRange(-70,70);
  //connect the spinbox and slider so they change the value when ever one change
  QObject::connect(left_cam_sb,SIGNAL(valueChanged(int)),
		   left_cam_s,SLOT(setValue(int)));
  QObject::connect(left_cam_s,SIGNAL(valueChanged(int)),
		       left_cam_sb,SLOT(setValue(int)));


  QObject::connect(left_cam_s,SIGNAL(valueChanged(int)),
		   this,SLOT(updateLeftPan()));



  //set the value at the start to zero
  left_cam_sb->setValue(0);

  //define the layout of the windows
  layout_left = new QHBoxLayout;
  layout_left->addWidget(left_label);
  layout_left->addWidget(left_cam_sb);
  layout_left->addWidget(left_cam_s);
  layout_left->addWidget(button_qt);


  ////////////////////////////////////////////////////////////////////
  //Create the right cam slider and apinbox
  //define the spinbox and sliders
  right_cam_sb = new QSpinBox;
  right_cam_s = new QSlider(Qt::Horizontal);
  right_label = new QLabel("right pan motor");
  //set the limeted
  right_cam_sb->setRange(-70,70);
  right_cam_s->setRange(-70,70);
  //connect the spinbox and slider so they change the value when ever one change
  QObject::connect(right_cam_sb,SIGNAL(valueChanged(int)),
		   right_cam_s,SLOT(setValue(int)));
  QObject::connect(right_cam_s,SIGNAL(valueChanged(int)),
		       right_cam_sb,SLOT(setValue(int)));
  //set the value at the start to zero
  right_cam_sb->setValue(0);

  //define the layout of the windows
  layout_right = new QHBoxLayout;
  layout_right->addWidget(right_label);
  layout_right->addWidget(right_cam_sb);
  layout_right->addWidget(right_cam_s);
  layout_right->addWidget(button_qt);

  ////////////////////////////////////////////////////////////////////
  //Create the baseline cam slider and apinbox
  //define the spinbox and sliders
  baseline_cam_sb = new QSpinBox;
  baseline_cam_s = new QSlider(Qt::Horizontal);
  baseline_label = new QLabel("baseline pan motor");
  //set the limeted
  baseline_cam_sb->setRange(49,520);
  baseline_cam_s->setRange(49,520);
  //connect the spinbox and slider so they change the value when ever one change
  QObject::connect(baseline_cam_sb,SIGNAL(valueChanged(int)),
		   baseline_cam_s,SLOT(setValue(int)));
  QObject::connect(baseline_cam_s,SIGNAL(valueChanged(int)),
		       baseline_cam_sb,SLOT(setValue(int)));
  //set the value at the start to zero
  baseline_cam_sb->setValue(0);

  //define the layout of the windows
  layout_baseline = new QHBoxLayout;
  layout_baseline->addWidget(baseline_label);
  layout_baseline->addWidget(baseline_cam_sb);
  layout_baseline->addWidget(baseline_cam_s);
  layout_baseline->addWidget(button_qt);
  ///////////////////////////////////////////////////////////////////////
  //define the main layout
  layout = new QVBoxLayout;
  layout->addLayout(layout_left);
  layout->addLayout(layout_right);
  layout->addLayout(layout_baseline);
  window->setLayout(layout);
  // show the windows
  window->show();

  //////////////////////////////////////////////////////////////////////////
  left_angle=left_cam_sb->value();
      qDebug()<< left_angle ;

}

void gui_controller::updateLeftPan(){
  // set the value
  /*left_pan.x=value;
  left_pan.y = 0;
  left_pan.z = 0;
  left_pan_pub.publish(left_pan);*/
  qDebug()<<" test: " ;
}


////////////////////////////////////////////////////////////////////////
/// Start the main function
int main(int argc, char *argv[])
{

  // initialize ros and the topics
  ros::init(argc, argv, "gui_controller");
  // initialize the gui
  QApplication app(argc,argv);
  gui_controller gui;
  double rate = 10.0;
  ros::Rate r(rate);

  return app.exec();
}
