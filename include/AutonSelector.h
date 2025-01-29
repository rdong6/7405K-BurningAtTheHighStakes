#pragma once
#include "RobotBase.h"
#include "liblvgl/lvgl.h"
#include <vector>

class AutonSelector {
private:
	// our custom data identifying a button is stored as user flag 1
	class Button {
	private:
		lv_obj_t* btn = nullptr;
		lv_obj_t* label = nullptr;

	public:
		Button() = default;
		Button(lv_obj_t* parent, const char* name, lv_event_cb_t callbackFn, void* callbackData);

		void setPos(lv_coord_t x, lv_coord_t y);
		void setSize(lv_coord_t w, lv_coord_t h);
		void setStyle(lv_style_t* style);
		void setHidden(bool en);
	};

	// screens
	lv_obj_t* lcdScreen = nullptr;// llemu's lvgl screen
	lv_obj_t* selectorScreen = nullptr;

	// tabs
	lv_obj_t* tabview = nullptr;     // parent obj for tabview
	lv_obj_t* matchTypeTab = nullptr;// tab to decide whether to select for qual/elim
	lv_obj_t* allianceTab = nullptr; // to decide between blue and red alliance
	lv_obj_t* autonsTab = nullptr;   // tab to select corresponding autons given the previously selected info
	lv_obj_t* delayTab = nullptr;    // simply a tab that allows one to increase delay at start of auton

	// container obj that store all autons button for each alliance
	lv_obj_t* blueElimAutonsParent = nullptr;
	lv_obj_t* redElimAutonsParent = nullptr;
	lv_obj_t* blueQualAutonsParent = nullptr;
	lv_obj_t* redQualAutonsParent = nullptr;

	// buttons to choose qual or elim
	Button qualsBtn;
	Button elimsBtn;

	// buttons to choose red or blue alliance
	Button redAllianceBtn;
	Button blueAllianceBtn;

	// buttons to cycle between autons on the autonsTab
	Button cycleLeftBtn;
	Button cycleRightBtn;

	// index of current auton to display on autonTab
	int autonIndex = 0;


	std::vector<Button> redElimAutons;
	std::vector<Button> blueElimAutons;
	std::vector<Button> redQualAutons;
	std::vector<Button> blueQualAutons;

	Button blueRingsideElim;
	Button blueRingsideQual;
	Button redRingsideElim;
	Button redRingsideQual;

	std::vector<Button>& getSelectedAutons();
	lv_obj_t* getAutonParentObj();

public:
	AutonSelector();

	void addAuton(const char* name, bool isRed, bool isQual, AutonFn_t autonFnPtr);

	// show auton selector
	void run();
};