#pragma once
#include "RobotBase.h"
#include "liblvgl/lvgl.h"
#include <atomic>
#include <stdint.h>
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
		Button(lv_obj_t* parent, const char* name, Auton auton, Alliance alliance);

		void setPos(lv_coord_t x, lv_coord_t y);
		void setSize(lv_coord_t w, lv_coord_t h);
		void setStyle(lv_style_t* style);
	};

	lv_obj_t* lcdScreen = nullptr;// llemu's lvgl screen
	lv_obj_t* selectorScreen = nullptr;

	Button blueRingsideElim;
	Button blueRingsideQual;
	Button redRingsideElim;
	Button redRingsideQual;

public:
	AutonSelector();

	// show auton selector
	void run();
};