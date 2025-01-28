#pragma once
#include "RobotBase.h"
#include "liblvgl/lvgl.h"
#include <atomic>
#include <stdint.h>
#include <vector>

//

class AutonSelector {
private:
	// our custom data identifying a button is stored as user flag 1
	class Button {
	private:
		lv_obj_t* btn = nullptr;
		lv_obj_t* label = nullptr;

	public:
		Button() = default;
		Button(lv_obj_t* parent, const char* name, Auton auton, Alliance alliance);

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
	lv_obj_t* blueAutonsParent;
	lv_obj_t* redAutonsParent;

	std::vector<Button> redAutons;
	std::vector<Button> blueAutons;

	Button blueRingsideElim;
	Button blueRingsideQual;
	Button redRingsideElim;
	Button redRingsideQual;

public:
	AutonSelector();

	void addAuton(const char* name, bool isRed, bool isQual, TaskFunc autonFunc);

	// show auton selector
	void run();
};