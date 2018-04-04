#include <iostream>
#include "explainable_planning/mainwindow.h"
#include "explainable_planning/plan_table.h"

/*-------------*/
/* CONSTRUCTOR */
/*-------------*/

	MainWindow::MainWindow()
	{
	
		QWidget *central_Widget = new QWidget;
		setCentralWidget(central_Widget);

		// init tables
		alt_plan_id =0 ;
		plan_table_all.push_back(std::list<std::string>());

	

		// create initial plan widget
		side_Bar = new QFrame();
		side_Bar->setFrameRect(QRect(0,0,250,250));
		
		initial_Plan_Stack = new QStackedWidget;
		initial_Plan_Layout = new QVBoxLayout;
		
		initial_Plan = new QGroupBox("InitialPlan");
		initial_Plan->setFlat(true);
		

		side_Bar->setStyleSheet("background-color: BlanchedAlmond ;");
		initial_Plan_Stack->setStyleSheet("background-color: red;");
		initial_Plan->setStyleSheet("background-color: BlanchedAlmond; ");
		
	

		create_Initial_Plan_Widget(initial_Plan, initial_Plan_Layout);
	
		// create generate and cost plan buttons
		QGroupBox *hGroupBox = new QGroupBox("Buttons");
		QHBoxLayout *bplanLayout = new QHBoxLayout;
		QPushButton *generateNPlan = new QPushButton("Generate");
		QPushButton *costNPlan = new QPushButton("Cost");
		QPushButton *saveWrkSp = new QPushButton("Save Workspace");
		generateNPlan->setGeometry(0,0,20,20);
		generateNPlan->setStyleSheet("background-color: lightblue; Text-align: center;");
		costNPlan->setStyleSheet("background-color: lightblue; Text-align: center;");
		saveWrkSp->setStyleSheet("background-color: lightblue; Text-align: center;");

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
		bplanLayout->addWidget(saveWrkSp, 0);
		hGroupBox->setLayout(bplanLayout);

		// lay out initial plan widget and buttons
		QVBoxLayout *initial_Plan_VLayout = new QVBoxLayout;
		initial_Plan_VLayout->addWidget(initial_Plan,0,Qt::AlignLeft);
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

		// text widget
		text_Box = new QTextEdit("Ready to roll!", this);
	

		// new plan widget
		alternative_Plan = new QGroupBox("Alternative Plan");
		alternative_Plan_Layout = new QVBoxLayout();
		alternative_Plan->setStyleSheet("background-color: AliceBlue ;");

		//QLabel *page1WidgetB = new QLabel("I will contain the new Plan");
		//other widget are a we migh use or ditch
	
		// charts widgets
		charts_Box = new QGroupBox("Charts");
		charts_Box->setStyleSheet("background-color: white;");
		QVBoxLayout *chartsLayout = new QVBoxLayout();
		charts_Box->setLayout(chartsLayout);

		// initialise main layouts:
		page_Widgets_Grid_Layout = new QGridLayout();
		central_Layout = new QGridLayout();
	
		// Setup the content as stacked widgets
		page_Widgets_Grid_Layout->addWidget(page_Widget_Option, 0, 0, 2, 1);
		page_Widgets_Grid_Layout->addWidget(alternative_Plan, 0, 1, 3, 1);
		page_Widgets_Grid_Layout->addWidget(text_Box, 2, 0, 1, 1);
		page_Widgets_Grid_Layout->addWidget(charts_Box, 3, 0, 1, 2);
		page_Widgets_Grid_Layout->setRowStretch(0, 2);
		page_Widgets_Grid_Layout->setRowStretch(2, 1);
		page_Widgets_Grid_Layout->setRowStretch(3, 1);
		page_Widgets_Grid_Layout->setColumnStretch(0, 1);
		page_Widgets_Grid_Layout->setColumnStretch(1, 1);
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

		if (layout!=NULL)
		{
			std::cout <<"Cleaning\n";
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
			button->setAutoExclusive(true);
			group->addButton(button);
			bl->addWidget(button);

			index++;
		}
	
		group->setExclusive(true);

		bl->addStretch(2);
		f->setLayout(bl);
		f->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
	}

	/**
	 * function that when an instruction in initial plan is clicked provides alternative options within the Options widget
	 */
	void MainWindow::on_planOptionButton_clicked(std::string id)
	{
		std::string ac(plan_table[std::atoi(id.c_str())].c_str()); //action chosen
		plan_table_all[alt_plan_id].push_back((std::string("Action chosen").append(ac.c_str())).c_str());
	
		// warn user about selection
		text_Box->setTextColor(QColor("blue"));
		text_Box->append("Action chosen");
		text_Box->setTextColor(QColor("red"));
		text_Box->setAlignment(Qt::AlignRight);
		text_Box->append(ac.c_str());
		text_Box->setTextColor(QColor("black"));
		text_Box->setAlignment(Qt::AlignLeft);
		text_Box->append("Generating options...");
		QCoreApplication::processEvents();


		// clear action list and layout
		action_table.clear();
		plan_table_alt.clear();
		
		clear_Layout(page_Widget_Option->layout());
		clear_Layout(alternative_Plan->layout());
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
			action->setCheckable(true);
			action->setAutoExclusive(true);

		}
		text_Box->append("Generation complete.");
		QCoreApplication::processEvents();

	}

	/**
	 * function that when an alternative option is clicked provides the alternative plan
	 */
	void MainWindow::on_actionOptionButton_clicked(std::string id)
	{
		std::string oc(action_table[std::atoi(id.c_str())].c_str()); //option chosen
		plan_table_all[alt_plan_id].push_back("Alternative Plan for Option: "+oc);
	
		
		// clear alternative plan list and layout
		plan_table_alt.clear();
		text_Box->setTextColor(QColor("blue"));
		text_Box->append("Option chosen");
		text_Box->setTextColor(QColor("red"));
		text_Box->setAlignment(Qt::AlignRight);
		text_Box->append(oc.c_str());
		text_Box->setTextColor(QColor("black"));
		text_Box->setAlignment(Qt::AlignLeft);

		QCoreApplication::processEvents();
		

		
		// move execution to new state
		explainable_planning_msgs::IntegerChoice srv;
		srv.request.option = std::atoi(id.c_str());
		apply_client.call(srv);

		expecting_initial_plan = false;
		
		generate_plan();
		// save current alternative plan 
		save_Alternative_Plan();
	
		display_Alternative_Plan();

	}


	void MainWindow::display_Alternative_Plan()
	{
		int index = 0;
		clear_Layout(alternative_Plan->layout());


		std::vector<std::string>::iterator ait = plan_table_alt.begin();
		for (; ait!=plan_table_alt.end(); ait++)
		{
			// TODO this is where to display the alternate plan
			QLabel* action_label = new QLabel((*ait).c_str());
			std::cout<< "action: "<<(*ait).c_str() <<"\n";
			alternative_Plan_Layout->addWidget(action_label);

			index++;
		}
		
		alternative_Plan_Layout->addStretch(1);
		alternative_Plan->setLayout(alternative_Plan_Layout);
		alternative_Plan->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	}


/**
	 * function that when an alternative option is clicked provides the alternative plan
	 */
	void MainWindow::on_saveWorkspaceButton_clicked()
	{
		// let user know
		text_Box->setTextColor(QColor("blue"));
		text_Box->append("Saving Workspace");

        // save workspace in 2 files
        // file 1 plan tables 
        // file 2 textbox content

        text_Box->append("Workspace Saved");

		text_Box->setTextColor(QColor("black"));
		text_Box->setAlignment(Qt::AlignLeft);

		QCoreApplication::processEvents();
		

		

	}
/*
*  Save main plan: assumption we only have 1 main plan 
*/

	void MainWindow::save_Main_Plan()
	{

	
		if(!plan_table.empty())
		{
			std::cout <<"saving Main plan \n";
			std::vector<std::string>::iterator ait = plan_table.begin();

			plan_table_all.push_back(std::list<std::string>());
			plan_table_all[0].push_back("Main Plan : P0");
			for (; ait!=plan_table.end(); ait++)
			{
				plan_table_all[0].push_back((*ait).c_str());
			}
			std::cout <<"saved Main plan \n";
			// saving plan cost in cost_table
			plan_table_cost.push_back((strtok((char *)((*(--ait)).c_str()), ":")));
		}
		else
			std::cout <<"Could not save Main plan, plan_table empty \n";
			
	}

/*
*  Save alternative plan
*/

	void MainWindow::save_Alternative_Plan()
	{

	
		if(!plan_table_alt.empty())
		{
			std::cout <<"saving Aletrnative plan n. " << alt_plan_id<<"\n";
			std::vector<std::string>::iterator ait = plan_table_alt.begin();
			plan_table_all.push_back(std::list<std::string>());
			for (; ait!=plan_table_alt.end(); ait++)
			{
				plan_table_all[alt_plan_id].push_back((*ait).c_str());
			}
			std::cout <<"saving Alternative plan n. "<<alt_plan_id<<" \n";
		// saving plan cost in cost_table
			plan_table_cost.push_back((strtok((char *)((*(--ait)).c_str()), ":")));
	
		}
		else
			std::cout <<"Could not save Alternative plan, plan_alt_table empty \n";
		
		alt_plan_id++; 
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
		std::vector<QBarSet*> bar_charts;
		std::string pn = "P_";
		QBarSeries *series = new QBarSeries();

		int index = 0;
		if(!plan_table_cost.empty())
		{
			std::vector<std::string>::iterator ait = plan_table_cost.begin();
			for (; ait!= plan_table_cost.end(); ait++)
			{
				bar_charts.push_back( new QBarSet((pn+std::to_string(index)).c_str()));
				(*(bar_charts[index])) << atoi((*ait).c_str());
				std::cout<< "Cost of P "<< index <<" "<< atoi((*ait).c_str()) << "\n";
				series->append(bar_charts[index]);
				index++;
			}
		}
//		QBarSet *plan0 = new QBarSet("P0");
//		QBarSet *plan1 = new QBarSet("P1");
	

//		*plan0 << atoi(plan_table_cost[0].c_str());
//		*plan1 << atoi(plan_table_cost[1].c_str());
		//*plan1 << 5 << 4 << 3 << 4 << 2 << 7;

//		QBarSeries *series = new QBarSeries();
//		series->append(plan0);
//		series->append(plan1);

		QChart *chart = new QChart();
		chart->addSeries(series);
		chart->setTitle("Plan Cost - Time (ms)");
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
			ss << plan.plan[i].dispatch_time << ":\t(" << plan.plan[i].name;
			for(int j=0; j<plan.plan[i].parameters.size(); j++) {
				ss << " " << plan.plan[i].parameters[j].value;
			}
			ss << ") [" << plan.plan[i].duration << "]";
			if(expecting_initial_plan) {
				plan_table.push_back(ss.str());
				create_Initial_Plan_Widget(initial_Plan, initial_Plan_Layout);
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
			ss << "(" << plan.plan[i].name;
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
		
		text_Box->append("Generating plan ....");
		QCoreApplication::processEvents();
		

		// generate a plan
		std_srvs::Empty empty;
		problem_client.call(empty);
		ros::Duration(1).sleep();
		planning_client.call(empty);
		ros::Duration(1).sleep();
		parsing_client.call(empty);
		ros::Duration(1).sleep();
		ros::spinOnce();
		text_Box->append("Generation completed.");
		QCoreApplication::processEvents();
		if(expecting_initial_plan)
			save_Main_Plan();
		
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
	window.setWindowState(window.windowState()^Qt::WindowMaximized);
	window.show();
	return app.exec();
}