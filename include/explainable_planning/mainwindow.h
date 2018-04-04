#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ros/ros.h>
#include "std_srvs/Empty.h"

#include <string>

#include <QtWidgets/QApplication>
#include <QtCore/QObject>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QLayout>
#include <QtWidgets/QAction>
#include <QtWidgets/QActionGroup>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTextEdit>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>
#include <QTimer>

#include "explainable_planning_msgs/IntegerChoice.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"

using namespace QtCharts;


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:

	MainWindow();
	~MainWindow();

	/* ROS methods for linking to planning */
	void planCallback(const rosplan_dispatch_msgs::CompletePlan plan);
	void actionsCallback(const rosplan_dispatch_msgs::CompletePlan plan);

private slots:

	// actions linked to menu options, empty for now.
	void new_File();
	void open();
	void save();
	void print();

	void ros_update();
	
private:

	// ROS state
	bool expecting_initial_plan;

	// ROS subscribers and service clients
	ros::ServiceClient op_client;
	ros::ServiceClient problem_client;
	ros::ServiceClient planning_client;
	ros::ServiceClient parsing_client;
	ros::ServiceClient action_client;
	ros::ServiceClient apply_client;
	ros::ServiceClient move_client;

	// Menu Ribbon at the top
	// functions to create Menus and action to link to menu option
	void create_Actions();
	void create_Menus();



private:
	// placeholders for now, no real action connected to them
	QMenu *file_Menu;
	QMenu *edit_Menu;
	QAction *new_Act;
	QAction *open_Act;
	QAction *save_Act;
	QAction *print_Act;
	QAction *exit_Act;

	// Utilty - Text Widget
	QTextEdit	*text_Box;

	// Interface is divided into two main areas:
	// - side_Bar: which contains the initial plan
	// - content_Page_Widgets: which contains 
	//		* the options, 
	//		* the new plan (to go into area labelled "I am widget C"), 
	//		* the chart widget (chartFrame), 
	//		* a placehoder labelled for now "I am widget D" 

	QGroupBox *initial_Plan; //
	QGroupBox *alternative_Plan; //
	int alt_plan_id; // counter to save action plans, we assume initial pna id always 0;
	
	QFrame *side_Bar; // widget containing the initial plan
	QGroupBox *page_Widget_Option; // widget containing the options
	//QGroupBox *page_Widget_AltPlan; // widget containing the options
	QFrame *content_Page_Widgets; // see above
	QGroupBox *charts_Box;  // chart widget area
	QStackedWidget *other_Content_Stack; //stack of widgets: options, chart, new plan
	QStackedWidget *initial_Plan_Stack; //Stack of widget: - initial plan widget and the two buttons "generate" and cost
	QChartView *cost_Chart; // chart contained in chartsFrame

	QGridLayout *page_Widgets_Grid_Layout; //layout of Optio, charts and new plan widgets
	QGridLayout *central_Layout; // main layout containing all widgets
	QVBoxLayout *initial_Plan_Layout; // layout containing initial plan
	QVBoxLayout *alternative_Plan_Layout; // layout containing alternative plan

	// function to read initial plan "instructions" and create initial plan widget
	void create_Initial_Plan_Widget(QGroupBox *f, QBoxLayout *bl);

	// saves initial plan
	void save_Main_Plan();

	//save alternative plan
	void save_Alternative_Plan();
	// display alternative plan
	void display_Alternative_Plan();
	// function that when an instruction in initial plan is clicked provides alternative options within the Options widget
	void on_planOptionButton_clicked(std::string id);
	void on_actionOptionButton_clicked(std::string id);
	void on_saveWorkspaceButton_clicked();

	// utility function to clear a layout when content is updated
	void clear_Layout(QLayout *layout);

	

	void create_Bar_Chart(QChartView *cost_Chart);

	// generate a new initial plan
	void generate_plan();
};


#endif // MAINWINDOW_H
