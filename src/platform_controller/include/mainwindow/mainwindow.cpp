#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QSlider"
#include "QSpinBox"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //connect the sliding, Dial to the spinBox to exchange the values
    connect(ui->SBaseline,SIGNAL(valueChanged(int)),
            ui->SBBaseline,SLOT(setValue(int)));
    connect(ui->SBBaseline,SIGNAL(valueChanged(int)),
           ui->SBaseline,SLOT(setValue(int)));
    //Left Dail camera
    connect(ui->DleftCam,SIGNAL(valueChanged(int)),
            ui->SBleftCam,SLOT(setValue(int)));
    connect(ui->SBleftCam,SIGNAL(valueChanged(int)),
            ui->DleftCam,SLOT(setValue(int)));
    //Right Dail camera
    connect(ui->DrightCam,SIGNAL(valueChanged(int)),
            ui->SBrightCam,SLOT(setValue(int)));
    connect(ui->SBrightCam,SIGNAL(valueChanged(int)),
            ui->DrightCam,SLOT(setValue(int)));


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    // Reset the valuse to the Home plase
    ui->SBaseline->setValue(90);
    ui->SBBaseline->setValue(90);
    ui->DleftCam->setValue(90);
    ui->DrightCam->setValue(90);
}
