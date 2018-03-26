#include "explainable_planning/mainwindow.h"
#include "explainable_planning/plan_table.h"

/*-------------*/
/* CONSTRUCTOR */
/*-------------*/

	MainWindow::MainWindow()
	{
	
		QWidget *central_Widget = new QWidget;
		setCentralWidget(central_Widget);

		// create initial plan widget
		side_Bar = new QFrame();
		initial_Plan_Stack = new QStackedWidget;
		initial_Plan_Layout = new QVBoxLayout;
		initial_plan = new QGroupBox("InitialPlan");

		side_Bar->setStyleSheet("background-color:white;");
		initial_Plan_Stack->setStyleSheet("background-color: red;");
		initial_plan->setStyleSheet("background-color: BlanchedAlmond ;");

		create_Initial_Plan_Widget(initial_plan, initial_Plan_Layout);

		// create generate and cost plan buttons	
		QGroupBox *hGroupBox = new QGroupBox("Buttons");
		QHBoxLayout *bplanLayout = new QHBoxLayout;
		QPushButton *generateNPlan = new QPushButton("Generate");
		QPushButton *costNPlan = new QPushButton("Cost");

		generateNPlan->setStyleSheet("background-color: lightblue; Text-align: center;");
		costNPlan->setStyleSheet("background-color: lightblue; Text-align: center;");

		// add actions to buttons // C++11
		connect(
			generateNPlan, &QPushButton::clicked,
			this, [this] {generate_plan(); }
		);

		// add actions to buttons // C++11
		connect(
			costNPlan, &QPushButton::clicked,
			this, [this] {create_Bar_Chart(cost_Chart); }
		);

		bplanLayout->addWidget(generateNPlan, 0);
		bplanLayout->addWidget(costNPlan, 0);
		hGroupBox->setLayout(bplanLayout);

		// lay out initial plan widget and buttons
		QVBoxLayout *initial_Plan_VLayout = new QVBoxLayout;
		initial_Plan_VLayout->addWidget(initial_plan,0,Qt::AlignLeft);
		initial_Plan_VLayout->addWidget(hGroupBox,0,Qt::AlignLeft);
	
		side_Bar->setLayout(initial_Plan_VLayout);
		initial_Plan_Stack->addWidget(side_Bar);
		//initial_Plan_Stack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

		// create widget container for Option, Chart and new plan widgets

		other_Content_Stack = new QStackedWidget; 
		content_Page_Widgets = new QFrame();

		// Options widget
		page_Widget_Option = new QGroupBox("Options");
		QVBoxLayout *vbl = new QVBoxLayout;
		page_Widget_Option->setLayout(vbl);

		// new plan widget
		QLabel *page1WidgetB = new QLabel("I will contain the new Plan");
		//other widget are a we migh use or ditch
		QLabel *page1WidgetC = new QLabel("I am widget C");
	
		// charts widgets
		charts_Box = new QGroupBox("Charts");
		charts_Box->setStyleSheet("background-color: white;");
		QVBoxLayout *chartsLayout = new QVBoxLayout();
		charts_Box->setLayout(chartsLayout);

		// initialise main layouts:
		page_Widgets_Grid_Layout = new QGridLayout();
		central_Layout = new QGridLayout();
	
		// Setup the content as stacked widgets
		page_Widgets_Grid_Layout->addWidget(page_Widget_Option, 0, 0, 3, 1);
		page_Widgets_Grid_Layout->addWidget(page1WidgetB, 0, 1, 1, 1);
		page_Widgets_Grid_Layout->addWidget(page1WidgetC, 1, 1, 2, 1);
		page_Widgets_Grid_Layout->addWidget(charts_Box, 3, 0, 1, 2);
		page_Widgets_Grid_Layout->setRowStretch(0, 3);
		page_Widgets_Grid_Layout->setRowStretch(1, 3);
		page_Widgets_Grid_Layout->setRowStretch(3, 2);
		content_Page_Widgets->setLayout(page_Widgets_Grid_Layout);

		other_Content_Stack->addWidget(content_Page_Widgets);
		other_Content_Stack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

		//Finally setup the main elements into the central layout...
	
		central_Layout->addWidget(initial_Plan_Stack, 1, 0, 1, 1);
		central_Layout->addWidget(other_Content_Stack, 1, 1, 1, 1);
		central_Layout->setColumnStretch(0, 0);
		central_Layout->setColumnStretch(1, 1);
	
		central_Widget->setLayout(central_Layout);
		setCentralWidget(central_Widget);

		/* Let's color it a little to better realize the positioning: */
		/*setStyleSheet("QWidget {"
			"border: 1px solid black;"
			"color: red"
			"}");
			*/
		setStyleSheet("QFrame {" "border: 1px solid #000000;" "}" "QPushButton{" "Text-align:left" "}");
	

		create_Actions();
		create_Menus();
		setObjectName("MainWindow");
		setWindowTitle("Qt Main Window Example");
		setMinimumSize(160, 160);
		resize(1800, 1200);

		// ROS services

		ros::NodeHandle nh("~");

		std::string opTopic = "/kcl_rosplan/get_domain_operators";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string actionTopic = "/rosplan_plan_explainer/get_actions";
		std::string applyTopic = "/rosplan_plan_explainer/apply_action";
		std::string moveTopic = "/rosplan_plan_explainer/move_execution";

		op_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>(opTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		action_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(actionTopic);
		apply_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(applyTopic);
		move_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(moveTopic);

		// timer begins ros_spin
		QTimer *timer = new QTimer(this);
		connect(timer, SIGNAL(timeout()), this, SLOT(ros_update()));
		timer->start(20);

		expecting_initial_plan = true;
	}

	MainWindow::~MainWindow()
	{
	}

/*------------------*/
/* CLEARING LAYOUTS */
/*------------------*/

	/**
	 * utility function to update widgets when content changes
	 */
	void MainWindow::clear_Layout(QLayout *layout)
	{
		QLayoutItem *item;

		while ((item = layout->takeAt(0))) {
			if (item->layout()) {
				clear_Layout(item->layout());
				delete item->layout();
			}
			if (item->widget()) {
				delete item->widget();
			}
			delete item;
		}
	}

/*-------------------------------*/
/* CREATING AND REFRESHING LISTS */
/*-------------------------------*/

	/* Initial Plan is a set of button labelled according to plan instructions
	 * at the moment "ficional" instructions are saved in a Table as "string"s
	 * Tables are in file plan_table.h
	 */
	void MainWindow::create_Initial_Plan_Widget(QGroupBox *f, QBoxLayout *bl)
	{

		clear_Layout(bl);

		QButtonGroup* group = new QButtonGroup(f);

		int index = 0;
		std::vector<std::string>::iterator ait = plan_table.begin();
		for(; ait!=plan_table.end(); ait++) {

			QPushButton* button = new QPushButton((*ait).c_str());

			// add actions to buttons // C++11
			connect(
				button, &QPushButton::clicked,
				this, [this, index] {on_planOptionButton_clicked(std::to_string(index)); }
			);
			// end of C++11 code

			button->setCheckable(true);
			group->addButton(button);
			bl->addWidget(button);

			index++;
		}
	
		group->setExclusive(true);

		bl->addStretch(1);
		f->setLayout(bl);
		f->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
	}

	/**
	 * function that when an instruction in initial plan is clicked provides alternative options within the Options widget
	 */
	void MainWindow::on_planOptionButton_clicked(std::string id)
	{
		// clear action list and layout
		action_table.clear();
		clear_Layout(page_Widget_Option->layout());
		//page_Widget_Option->layout()->addWidget(opt);

		// move execution to new state
		explainable_planning_msgs::IntegerChoice srv;
		srv.request.option = std::atoi(id.c_str());
		move_client.call(srv);

		// fetch operator names
		rosplan_knowledge_msgs::GetDomainOperatorService opsrv;
		op_client.call(opsrv);

		// for each operator, fetch applicable actions
		for(int i=0; i<opsrv.response.operators.size(); i++) {
			srv.request.option = i;
			action_client.call(srv);
			ros::spinOnce();
		}

		for (int i = 0; i < action_table.size(); i++)
		{
			QPushButton *action = new QPushButton(action_table[i].c_str());
			page_Widget_Option->layout()->addWidget(action);

			// add actions to buttons // C++11
			connect(
				action, &QPushButton::clicked,
				this, [this, i] {on_actionOptionButton_clicked(std::to_string(i)); }
			);
			// end of C++11 code
		}
	}

	/**
	 * function that when an alternative option is clicked provides the alternative plan
	 */
	void MainWindow::on_actionOptionButton_clicked(std::string id)
	{
		// clear action list and layout
		plan_table_alt.clear();
		clear_Layout(page_Widget_Option->layout());

		// move execution to new state
		explainable_planning_msgs::IntegerChoice srv;
		srv.request.option = std::atoi(id.c_str());
		apply_client.call(srv);

		expecting_initial_plan = false;
		generate_plan();

		for (int i = 0; i < plan_table_alt.size(); i++)
		{
			// TODO this is where to display the alternate plan
		}
	}

/*--------*/
/* CHARTS */
/*--------*/

	/**
	 * function to create a bar chart
	 * value will be changed to cost of new plan(s)
	 */
	void MainWindow::create_Bar_Chart(QChartView *cost_Chart)
	{
		QBarSet *plan0 = new QBarSet("P0");
		QBarSet *plan1 = new QBarSet("P1");
	

		*plan0 << 1 << 2 << 3 << 4 << 5 << 6;
		*plan1 << 5 << 4 << 3 << 4 << 2 << 7;

		QBarSeries *series = new QBarSeries();
		series->append(plan0);
		series->append(plan1);

		QChart *chart = new QChart();
		chart->addSeries(series);
		chart->setTitle("Simple barchart example");
		chart->setAnimationOptions(QChart::SeriesAnimations);

	
		QBarCategoryAxis *axis = new QBarCategoryAxis();
		chart->createDefaultAxes();
		chart->setAxisX(axis, series);

		chart->legend()->setVisible(true);
		chart->legend()->setAlignment(Qt::AlignBottom);

		clear_Layout(charts_Box->layout());


		cost_Chart = new QChartView(chart);
		cost_Chart->setRenderHint(QPainter::Antialiasing);
		charts_Box->layout()->addWidget(cost_Chart);

	
	}

/*--------------*/
/* PLACEHOLDERS */
/*--------------*/

	// Actions for menu items, placeholders for the moment
	void MainWindow::create_Actions()
	{
		new_Act = new QAction(tr("&New"), this);
		new_Act->setShortcuts(QKeySequence::New);
		new_Act->setStatusTip(tr("Create a new file"));
		connect(new_Act, &QAction::triggered, this, &MainWindow::new_File);

		open_Act = new QAction(tr("&Open..."), this);
		open_Act->setShortcuts(QKeySequence::Open);
		open_Act->setStatusTip(tr("Open an existing file"));
		connect(open_Act, &QAction::triggered, this, &MainWindow::open);

		save_Act = new QAction(tr("&Save"), this);
		save_Act->setShortcuts(QKeySequence::Save);
		save_Act->setStatusTip(tr("Save the document to disk"));
		connect(save_Act, &QAction::triggered, this, &MainWindow::save);

		print_Act = new QAction(tr("&Print..."), this);
		print_Act->setShortcuts(QKeySequence::Print);
		print_Act->setStatusTip(tr("Print the document"));
		connect(print_Act, &QAction::triggered, this, &MainWindow::print);

		exit_Act = new QAction(tr("E&xit"), this);
		exit_Act->setShortcuts(QKeySequence::Quit);
		exit_Act->setStatusTip(tr("Exit the application"));
		connect(exit_Act, &QAction::triggered, this, &QWidget::close);
	}

	void MainWindow::create_Menus()
	{
		file_Menu = menuBar()->addMenu(tr("&File"));
		file_Menu->addAction(new_Act);
		file_Menu->addAction(open_Act);
		file_Menu->addAction(save_Act);
		file_Menu->addAction(print_Act);
		file_Menu->addSeparator();
		file_Menu->addAction(exit_Act);
	}
	void MainWindow::new_File()
	{
	}

	void MainWindow::open()
	{
	}

	void MainWindow::save()
	{
	}

	void MainWindow::print()
	{
	}

	void MainWindow::ros_update()
	{
		ros::spinOnce();
	}

/*-----------------*/
/* ROS CONNECTIONS */
/*-----------------*/


	/**
	 * Initial and alternative plan callback method
	 */
	void MainWindow::planCallback(const rosplan_dispatch_msgs::CompletePlan plan) {

		ROS_INFO("KCL: (%s) Plan recieved.", ros::this_node::getName().c_str());

		if(expecting_initial_plan) {
			plan_table.clear();
		} else {
			 plan_table_alt.clear();
		}

		// save plan as labels for display
		std::stringstream ss;
		for(int i=0; i<plan.plan.size(); i++) {
			ss.str("");
			ss << i << ":\t(" << plan.plan[i].name;
			for(int j=0; j<plan.plan[i].parameters.size(); j++) {
				ss << " " << plan.plan[i].parameters[j].value;
			}
			ss << ")";
			if(expecting_initial_plan) {
				plan_table.push_back(ss.str());
				create_Initial_Plan_Widget(initial_plan, initial_Plan_Layout);
			} else {
				 plan_table_alt.push_back(ss.str());
			}
		}
	}

	/**
	 * Alternative action callback method
	 */
	void MainWindow::actionsCallback(const rosplan_dispatch_msgs::CompletePlan plan) {

		ROS_INFO("KCL: (%s) Actions recieved.", ros::this_node::getName().c_str());

		// save plan as labels for display
		std::stringstream ss;
		for(int i=0; i<plan.plan.size(); i++) {
			ss.str("");
			ss << i << ":\t(" << plan.plan[i].name;
			for(int j=0; j<plan.plan[i].parameters.size(); j++) {
				ss << " " << plan.plan[i].parameters[j].value;
			}
			ss << ")";
			action_table.push_back(ss.str());
		}
	}

	/**
	 * Service calls to generate a new plan.
	 */
	void MainWindow::generate_plan() {
		// generate a plan
		std_srvs::Empty empty;
		problem_client.call(empty);
		ros::Duration(1).sleep();
		planning_client.call(empty);
		ros::Duration(1).sleep();
		parsing_client.call(empty);
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

// END MAININDOW

/**
 * main method
 */
int main(int argc, char *argv[])
{
	ros::init(argc,argv,"rosplan_explaination_interface");
	ros::NodeHandle nh("~");

	// create the window
	QApplication app(argc, argv);
	MainWindow window;

	// subscribers
	std::string aas_topic = "/rosplan_plan_explainer/applicable_actions";
	std::string lps_topic = "/rosplan_parsing_interface/complete_plan";

	ros::Subscriber lps = nh.subscribe(lps_topic, 1, &MainWindow::planCallback, &window);
	ros::Subscriber aas = nh.subscribe(aas_topic, 1, &MainWindow::actionsCallback, &window);

	// begin
	window.show();
	return app.exec();
}
