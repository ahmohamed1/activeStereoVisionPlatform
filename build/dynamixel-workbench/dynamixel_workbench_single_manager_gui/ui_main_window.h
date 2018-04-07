/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDial>
#include <QtGui/QDockWidget>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_12;
    QGridLayout *gridLayout_3;
    QListView *view_logging;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_3;
    QFrame *info_command_frame;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *dynamixel_information_group_box;
    QGridLayout *gridLayout;
    QLabel *get_device_name_label;
    QLabel *get_baud_rate_label;
    QLineEdit *get_protocol_version_line_edit;
    QLineEdit *get_model_name_line_edit;
    QLineEdit *get_device_name_line_edit;
    QLineEdit *get_baud_rate_line_edit;
    QLabel *get_protocol_version_label;
    QLabel *get_model_name_label;
    QLabel *get_id_label;
    QLineEdit *get_id_line_edit;
    QGroupBox *dynamixel_command_group_box;
    QGridLayout *gridLayout_2;
    QLabel *set_id_label;
    QComboBox *set_operating_mode_combo_box;
    QPushButton *set_position_zero_push_button;
    QLabel *set_baud_rate_label;
    QDial *set_address_value_dial;
    QLabel *set_operating_mode_label;
    QSpinBox *set_address_value_spin_box;
    QComboBox *set_address_name_combo_box;
    QComboBox *set_baud_rate_combo_box;
    QComboBox *set_id_combo_box;
    QLabel *set_address_value_label;
    QFrame *button_frame;
    QVBoxLayout *verticalLayout;
    QPushButton *torque_enable_toggle_button;
    QPushButton *factory_reset_push_button;
    QPushButton *reboot_push_button;
    QPushButton *quit_button;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(800, 871);
        MainWindowDesign->setMinimumSize(QSize(800, 0));
        MainWindowDesign->setMaximumSize(QSize(819, 16777215));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(390, 800));
        tab_manager->setMaximumSize(QSize(390, 800));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_12 = new QGroupBox(tab_status);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_12->sizePolicy().hasHeightForWidth());
        groupBox_12->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_12);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        view_logging = new QListView(groupBox_12);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_12);

        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 19));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        dockWidget = new QDockWidget(MainWindowDesign);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        dockWidget->setMinimumSize(QSize(390, 830));
        dockWidget->setMaximumSize(QSize(390, 830));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_3 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        info_command_frame = new QFrame(dockWidgetContents);
        info_command_frame->setObjectName(QString::fromUtf8("info_command_frame"));
        info_command_frame->setMinimumSize(QSize(370, 580));
        info_command_frame->setMaximumSize(QSize(370, 580));
        info_command_frame->setLayoutDirection(Qt::LeftToRight);
        info_command_frame->setFrameShape(QFrame::StyledPanel);
        info_command_frame->setFrameShadow(QFrame::Raised);
        verticalLayout_4 = new QVBoxLayout(info_command_frame);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        dynamixel_information_group_box = new QGroupBox(info_command_frame);
        dynamixel_information_group_box->setObjectName(QString::fromUtf8("dynamixel_information_group_box"));
        dynamixel_information_group_box->setMinimumSize(QSize(350, 250));
        dynamixel_information_group_box->setMaximumSize(QSize(350, 250));
        dynamixel_information_group_box->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        gridLayout = new QGridLayout(dynamixel_information_group_box);
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(9, -1, -1, 9);
        get_device_name_label = new QLabel(dynamixel_information_group_box);
        get_device_name_label->setObjectName(QString::fromUtf8("get_device_name_label"));
        get_device_name_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(get_device_name_label, 0, 0, 1, 1);

        get_baud_rate_label = new QLabel(dynamixel_information_group_box);
        get_baud_rate_label->setObjectName(QString::fromUtf8("get_baud_rate_label"));
        get_baud_rate_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(get_baud_rate_label, 2, 0, 1, 1);

        get_protocol_version_line_edit = new QLineEdit(dynamixel_information_group_box);
        get_protocol_version_line_edit->setObjectName(QString::fromUtf8("get_protocol_version_line_edit"));
        get_protocol_version_line_edit->setEnabled(false);
        get_protocol_version_line_edit->setAlignment(Qt::AlignCenter);
        get_protocol_version_line_edit->setReadOnly(true);

        gridLayout->addWidget(get_protocol_version_line_edit, 4, 1, 1, 1);

        get_model_name_line_edit = new QLineEdit(dynamixel_information_group_box);
        get_model_name_line_edit->setObjectName(QString::fromUtf8("get_model_name_line_edit"));
        get_model_name_line_edit->setEnabled(false);
        get_model_name_line_edit->setAlignment(Qt::AlignCenter);
        get_model_name_line_edit->setReadOnly(true);

        gridLayout->addWidget(get_model_name_line_edit, 6, 1, 1, 1);

        get_device_name_line_edit = new QLineEdit(dynamixel_information_group_box);
        get_device_name_line_edit->setObjectName(QString::fromUtf8("get_device_name_line_edit"));
        get_device_name_line_edit->setEnabled(false);
        get_device_name_line_edit->setAlignment(Qt::AlignCenter);
        get_device_name_line_edit->setReadOnly(true);

        gridLayout->addWidget(get_device_name_line_edit, 0, 1, 1, 1);

        get_baud_rate_line_edit = new QLineEdit(dynamixel_information_group_box);
        get_baud_rate_line_edit->setObjectName(QString::fromUtf8("get_baud_rate_line_edit"));
        get_baud_rate_line_edit->setEnabled(false);
        get_baud_rate_line_edit->setAlignment(Qt::AlignCenter);
        get_baud_rate_line_edit->setReadOnly(true);

        gridLayout->addWidget(get_baud_rate_line_edit, 2, 1, 1, 1);

        get_protocol_version_label = new QLabel(dynamixel_information_group_box);
        get_protocol_version_label->setObjectName(QString::fromUtf8("get_protocol_version_label"));
        get_protocol_version_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(get_protocol_version_label, 4, 0, 1, 1);

        get_model_name_label = new QLabel(dynamixel_information_group_box);
        get_model_name_label->setObjectName(QString::fromUtf8("get_model_name_label"));
        get_model_name_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(get_model_name_label, 6, 0, 1, 1);

        get_id_label = new QLabel(dynamixel_information_group_box);
        get_id_label->setObjectName(QString::fromUtf8("get_id_label"));
        get_id_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(get_id_label, 1, 0, 1, 1);

        get_id_line_edit = new QLineEdit(dynamixel_information_group_box);
        get_id_line_edit->setObjectName(QString::fromUtf8("get_id_line_edit"));
        get_id_line_edit->setEnabled(false);
        get_id_line_edit->setAlignment(Qt::AlignCenter);
        get_id_line_edit->setReadOnly(true);

        gridLayout->addWidget(get_id_line_edit, 1, 1, 1, 1);


        verticalLayout_4->addWidget(dynamixel_information_group_box);

        dynamixel_command_group_box = new QGroupBox(info_command_frame);
        dynamixel_command_group_box->setObjectName(QString::fromUtf8("dynamixel_command_group_box"));
        dynamixel_command_group_box->setMinimumSize(QSize(350, 300));
        dynamixel_command_group_box->setMaximumSize(QSize(350, 300));
        dynamixel_command_group_box->setFlat(false);
        gridLayout_2 = new QGridLayout(dynamixel_command_group_box);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        set_id_label = new QLabel(dynamixel_command_group_box);
        set_id_label->setObjectName(QString::fromUtf8("set_id_label"));
        set_id_label->setMinimumSize(QSize(130, 0));
        set_id_label->setMaximumSize(QSize(130, 16777215));
        set_id_label->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(set_id_label, 0, 0, 1, 1);

        set_operating_mode_combo_box = new QComboBox(dynamixel_command_group_box);
        set_operating_mode_combo_box->setObjectName(QString::fromUtf8("set_operating_mode_combo_box"));
        set_operating_mode_combo_box->setMinimumSize(QSize(190, 0));
        set_operating_mode_combo_box->setMaximumSize(QSize(190, 16777215));
        set_operating_mode_combo_box->setEditable(false);
        set_operating_mode_combo_box->setMaxCount(10);

        gridLayout_2->addWidget(set_operating_mode_combo_box, 1, 1, 1, 1);

        set_position_zero_push_button = new QPushButton(dynamixel_command_group_box);
        set_position_zero_push_button->setObjectName(QString::fromUtf8("set_position_zero_push_button"));
        set_position_zero_push_button->setMinimumSize(QSize(190, 0));
        set_position_zero_push_button->setMaximumSize(QSize(190, 16777215));

        gridLayout_2->addWidget(set_position_zero_push_button, 7, 1, 1, 1);

        set_baud_rate_label = new QLabel(dynamixel_command_group_box);
        set_baud_rate_label->setObjectName(QString::fromUtf8("set_baud_rate_label"));
        set_baud_rate_label->setMinimumSize(QSize(130, 0));
        set_baud_rate_label->setMaximumSize(QSize(130, 16777215));
        set_baud_rate_label->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(set_baud_rate_label, 2, 0, 1, 1);

        set_address_value_dial = new QDial(dynamixel_command_group_box);
        set_address_value_dial->setObjectName(QString::fromUtf8("set_address_value_dial"));
        set_address_value_dial->setMinimumSize(QSize(190, 0));
        set_address_value_dial->setMaximumSize(QSize(190, 16777215));

        gridLayout_2->addWidget(set_address_value_dial, 5, 1, 1, 1);

        set_operating_mode_label = new QLabel(dynamixel_command_group_box);
        set_operating_mode_label->setObjectName(QString::fromUtf8("set_operating_mode_label"));
        set_operating_mode_label->setMinimumSize(QSize(130, 0));
        set_operating_mode_label->setMaximumSize(QSize(130, 16777215));
        set_operating_mode_label->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(set_operating_mode_label, 1, 0, 1, 1);

        set_address_value_spin_box = new QSpinBox(dynamixel_command_group_box);
        set_address_value_spin_box->setObjectName(QString::fromUtf8("set_address_value_spin_box"));
        set_address_value_spin_box->setMinimumSize(QSize(190, 0));
        set_address_value_spin_box->setMaximumSize(QSize(190, 16777215));
        set_address_value_spin_box->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        set_address_value_spin_box->setKeyboardTracking(true);
        set_address_value_spin_box->setMinimum(-2048);
        set_address_value_spin_box->setMaximum(2048);

        gridLayout_2->addWidget(set_address_value_spin_box, 9, 1, 1, 1);

        set_address_name_combo_box = new QComboBox(dynamixel_command_group_box);
        set_address_name_combo_box->setObjectName(QString::fromUtf8("set_address_name_combo_box"));
        set_address_name_combo_box->setMinimumSize(QSize(130, 0));
        set_address_name_combo_box->setMaximumSize(QSize(130, 16777215));
        set_address_name_combo_box->setMaxVisibleItems(100);

        gridLayout_2->addWidget(set_address_name_combo_box, 5, 0, 1, 1);

        set_baud_rate_combo_box = new QComboBox(dynamixel_command_group_box);
        set_baud_rate_combo_box->setObjectName(QString::fromUtf8("set_baud_rate_combo_box"));
        set_baud_rate_combo_box->setMinimumSize(QSize(190, 0));
        set_baud_rate_combo_box->setMaximumSize(QSize(190, 16777215));
        set_baud_rate_combo_box->setMaxCount(10);

        gridLayout_2->addWidget(set_baud_rate_combo_box, 2, 1, 1, 1);

        set_id_combo_box = new QComboBox(dynamixel_command_group_box);
        set_id_combo_box->setObjectName(QString::fromUtf8("set_id_combo_box"));
        set_id_combo_box->setMinimumSize(QSize(190, 0));
        set_id_combo_box->setMaximumSize(QSize(190, 16777215));
        set_id_combo_box->setCursor(QCursor(Qt::ArrowCursor));
        set_id_combo_box->setAcceptDrops(false);
        set_id_combo_box->setEditable(false);
        set_id_combo_box->setMaxVisibleItems(10);
        set_id_combo_box->setMaxCount(254);
        set_id_combo_box->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);
        set_id_combo_box->setMinimumContentsLength(10);

        gridLayout_2->addWidget(set_id_combo_box, 0, 1, 1, 1);

        set_address_value_label = new QLabel(dynamixel_command_group_box);
        set_address_value_label->setObjectName(QString::fromUtf8("set_address_value_label"));
        set_address_value_label->setMinimumSize(QSize(130, 0));
        set_address_value_label->setMaximumSize(QSize(130, 16777215));
        set_address_value_label->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(set_address_value_label, 9, 0, 1, 1);


        verticalLayout_4->addWidget(dynamixel_command_group_box);


        verticalLayout_3->addWidget(info_command_frame);

        button_frame = new QFrame(dockWidgetContents);
        button_frame->setObjectName(QString::fromUtf8("button_frame"));
        button_frame->setMinimumSize(QSize(370, 200));
        button_frame->setMaximumSize(QSize(370, 200));
        button_frame->setFrameShape(QFrame::StyledPanel);
        button_frame->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(button_frame);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        torque_enable_toggle_button = new QPushButton(button_frame);
        torque_enable_toggle_button->setObjectName(QString::fromUtf8("torque_enable_toggle_button"));
        torque_enable_toggle_button->setMinimumSize(QSize(0, 30));
        torque_enable_toggle_button->setMaximumSize(QSize(16777215, 30));
        torque_enable_toggle_button->setCheckable(true);
        torque_enable_toggle_button->setChecked(false);

        verticalLayout->addWidget(torque_enable_toggle_button);

        factory_reset_push_button = new QPushButton(button_frame);
        factory_reset_push_button->setObjectName(QString::fromUtf8("factory_reset_push_button"));
        factory_reset_push_button->setMinimumSize(QSize(0, 30));
        factory_reset_push_button->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(factory_reset_push_button);

        reboot_push_button = new QPushButton(button_frame);
        reboot_push_button->setObjectName(QString::fromUtf8("reboot_push_button"));
        reboot_push_button->setMinimumSize(QSize(0, 30));
        reboot_push_button->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(reboot_push_button);

        quit_button = new QPushButton(button_frame);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy1);
        quit_button->setMinimumSize(QSize(0, 30));
        quit_button->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(quit_button);


        verticalLayout_3->addWidget(button_frame);

        dockWidget->setWidget(dockWidgetContents);
        MainWindowDesign->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        QWidget::setTabOrder(tab_manager, view_logging);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "Dynamixel Workbench Single Manager", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        groupBox_12->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Dynamixel current state", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
        dockWidget->setWindowTitle(QApplication::translate("MainWindowDesign", "    Manager Command Panel", 0, QApplication::UnicodeUTF8));
        dynamixel_information_group_box->setTitle(QApplication::translate("MainWindowDesign", "Dynamixel information", 0, QApplication::UnicodeUTF8));
        get_device_name_label->setText(QApplication::translate("MainWindowDesign", "Device Name", 0, QApplication::UnicodeUTF8));
        get_baud_rate_label->setText(QApplication::translate("MainWindowDesign", "Baudrate", 0, QApplication::UnicodeUTF8));
        get_protocol_version_label->setText(QApplication::translate("MainWindowDesign", "Protocol Version", 0, QApplication::UnicodeUTF8));
        get_model_name_label->setText(QApplication::translate("MainWindowDesign", "Model Name", 0, QApplication::UnicodeUTF8));
        get_id_label->setText(QApplication::translate("MainWindowDesign", "ID", 0, QApplication::UnicodeUTF8));
        dynamixel_command_group_box->setTitle(QApplication::translate("MainWindowDesign", "Dynamixel Command (Change to)", 0, QApplication::UnicodeUTF8));
        set_id_label->setText(QApplication::translate("MainWindowDesign", "ID", 0, QApplication::UnicodeUTF8));
        set_position_zero_push_button->setText(QApplication::translate("MainWindowDesign", "Position ZERO", 0, QApplication::UnicodeUTF8));
        set_baud_rate_label->setText(QApplication::translate("MainWindowDesign", "Baudrate", 0, QApplication::UnicodeUTF8));
        set_operating_mode_label->setText(QApplication::translate("MainWindowDesign", "Operating Mode", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_ACCESSIBILITY
        set_id_combo_box->setAccessibleDescription(QString());
#endif // QT_NO_ACCESSIBILITY
        set_address_value_label->setText(QApplication::translate("MainWindowDesign", "Change Value", 0, QApplication::UnicodeUTF8));
        torque_enable_toggle_button->setText(QApplication::translate("MainWindowDesign", "Torque Enable", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        factory_reset_push_button->setToolTip(QApplication::translate("MainWindowDesign", "Command for all data back to the factory settings values", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        factory_reset_push_button->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_WHATSTHIS
        factory_reset_push_button->setWhatsThis(QString());
#endif // QT_NO_WHATSTHIS
        factory_reset_push_button->setText(QApplication::translate("MainWindowDesign", "Factory reset", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        reboot_push_button->setToolTip(QApplication::translate("MainWindowDesign", "Reboot a dynamixel (Only for supports protocol 2.0)", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        reboot_push_button->setText(QApplication::translate("MainWindowDesign", "Reboot", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
