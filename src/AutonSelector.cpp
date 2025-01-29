#include "AutonSelector.h"
#include "Robot.h"

static lv_style_t btnRedStyle;
static lv_style_t btnBlueStyle;
static lv_style_t btnGreenStyle;
static lv_style_t btnGrayStyle;


inline lv_coord_t percentX(double percent) {
	return 480.0 / 100 * percent;
}

inline lv_coord_t percentY(double percent) {
	return 240.0 / 100 * percent;
}

static uint32_t combine(uint16_t low, uint16_t high) {
	return (static_cast<uint32_t>(high) << 16) + static_cast<uint32_t>(low);
}

static std::pair<uint16_t, uint16_t> separate(uint32_t x) {
	return std::make_pair(static_cast<uint16_t>(x >> 16), static_cast<uint16_t>(x));
}


static void autonBtnCallback(lv_event_t* event) {
	// higher bits = alliance
	// lower bits = auton
	auto decodedData = separate((uint32_t) lv_event_get_user_data(event));
	robotInstance->curAlliance = static_cast<Alliance>(decodedData.first);
	// robotInstance->curAuton = static_cast<Auton>(decodedData.second);
}

static void cycleAutonBtnCallback(lv_event_t* event) {
	//
}

AutonSelector::Button::Button(lv_obj_t* parent, const char* name, lv_event_cb_t callbackFn, void* callbackData) {
	btn = lv_btn_create(parent);
	label = lv_label_create(btn);
	lv_label_set_text(label, name);
	lv_obj_center(label);

	lv_obj_add_event_cb(btn, callbackFn, LV_EVENT_PRESSED, callbackData);
	// lv_obj_add_event_cb(btn, callbackFn, LV_EVENT_PRESSED,
	//                     (void*) combine(static_cast<uint16_t>(auton), static_cast<uint16_t>(alliance)));
}

void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
	lv_obj_set_pos(btn, x, y);
}

void AutonSelector::Button::setSize(lv_coord_t w, lv_coord_t h) {
	lv_obj_set_size(btn, w, h);
}

void AutonSelector::Button::setStyle(lv_style_t* style) {
	lv_obj_add_style(btn, style, 0);
}

void AutonSelector::Button::setHidden(bool en) {
	if (en) {
		lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);
	} else {
		lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);
	}
}

AutonSelector::AutonSelector() {
	pros::lcd::initialize();
	lcdScreen = lv_scr_act();
}

void AutonSelector::run() {
	return;
	// setup styles
	lv_style_init(&btnRedStyle);
	lv_style_set_bg_color(&btnRedStyle, lv_color_hex(0xf21e0f));
	lv_style_set_bg_opa(&btnRedStyle, LV_OPA_COVER);
	lv_style_set_text_font(&btnRedStyle, &lv_font_montserrat_40);

	lv_style_init(&btnBlueStyle);
	lv_style_set_bg_color(&btnBlueStyle, lv_color_hex(0x0f4ff2));
	lv_style_set_bg_opa(&btnBlueStyle, LV_OPA_COVER);
	lv_style_set_text_font(&btnBlueStyle, &lv_font_montserrat_40);

	lv_style_init(&btnGreenStyle);
	lv_style_set_bg_color(&btnGreenStyle, lv_color_hex(0x0ff226));
	lv_style_set_bg_opa(&btnGreenStyle, LV_OPA_COVER);
	lv_style_set_text_font(&btnGreenStyle, &lv_font_montserrat_40);

	lv_style_init(&btnGrayStyle);
	lv_style_set_bg_color(&btnGrayStyle, lv_color_hex(0x5c5c5c));
	lv_style_set_bg_opa(&btnGrayStyle, LV_OPA_COVER);
	lv_style_set_text_font(&btnGrayStyle, &lv_font_montserrat_40);


	// create parent screen
	selectorScreen = lv_obj_create(nullptr);

	// create tabview for auton
	tabview = lv_tabview_create(selectorScreen, LV_DIR_TOP, 10 /* tune height of tab */);
	matchTypeTab = lv_tabview_add_tab(tabview, "Match Type");
	allianceTab = lv_tabview_add_tab(tabview, "Alliance");
	autonsTab = lv_tabview_add_tab(tabview, "Autons");
	delayTab = lv_tabview_add_tab(tabview, "Delay");

	// create parent container objs for the auton buttons
	blueQualAutonsParent = lv_obj_create(autonsTab);
	lv_obj_set_pos(blueQualAutonsParent, 145, -10);
	lv_obj_set_size(blueQualAutonsParent, 175, 150);
	blueQualAutonsParent = lv_obj_create(autonsTab);
	lv_obj_set_pos(blueQualAutonsParent, 145, -10);
	lv_obj_set_size(blueQualAutonsParent, 175, 150);
	redQualAutonsParent = lv_obj_create(autonsTab);
	lv_obj_set_pos(redQualAutonsParent, 145, -10);
	lv_obj_set_size(redQualAutonsParent, 175, 150);
	redElimAutonsParent = lv_obj_create(autonsTab);
	lv_obj_set_pos(redElimAutonsParent, 145, -10);
	lv_obj_set_size(redElimAutonsParent, 175, 150);
	// make the autons hidden by default
	lv_obj_add_flag(blueQualAutonsParent, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(blueElimAutonsParent, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(redQualAutonsParent, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(redElimAutonsParent, LV_OBJ_FLAG_HIDDEN);

	// qual or elim selector btn
	qualsBtn = Button(
	        matchTypeTab, "Quals",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        robotInstance->isElim = false;
		        autonSelector->qualsBtn.setStyle(&btnGreenStyle);
		        autonSelector->elimsBtn.setStyle(&btnGrayStyle);
		        lv_tabview_set_act(autonSelector->tabview, 1, LV_ANIM_ON);
	        },
	        this);
	qualsBtn.setPos(20, -10);
	qualsBtn.setSize(175, 150);
	qualsBtn.setStyle(&btnRedStyle);// set to orange
	elimsBtn = Button(
	        matchTypeTab, "Elims",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        robotInstance->isElim = true;
		        autonSelector->qualsBtn.setStyle(&btnGrayStyle);
		        autonSelector->elimsBtn.setStyle(&btnGreenStyle);
		        lv_tabview_set_act(autonSelector->tabview, 1, LV_ANIM_ON);
	        },
	        this);
	elimsBtn.setPos(275, 20);
	elimsBtn.setSize(175, 150);
	elimsBtn.setStyle(&btnBlueStyle);// set to dark purple

	redAllianceBtn = Button(
	        allianceTab, "Red",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        robotInstance->curAlliance = Alliance::RED;
		        autonSelector->redAllianceBtn.setStyle(&btnGreenStyle);
		        autonSelector->blueAllianceBtn.setStyle(&btnGrayStyle);
		        lv_tabview_set_act(autonSelector->tabview, 2, LV_ANIM_ON);
	        },
	        this);
	redAllianceBtn.setPos(20, -10);
	redAllianceBtn.setSize(175, 150);
	redAllianceBtn.setStyle(&btnRedStyle);
	blueAllianceBtn = Button(
	        allianceTab, "Blue",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        robotInstance->curAlliance = Alliance::BLUE;
		        autonSelector->redAllianceBtn.setStyle(&btnGrayStyle);
		        autonSelector->blueAllianceBtn.setStyle(&btnGreenStyle);
		        lv_tabview_set_act(autonSelector->tabview, 2, LV_ANIM_ON);
	        },
	        this);
	blueAllianceBtn.setPos(275, 20);
	blueAllianceBtn.setSize(175, 150);
	blueAllianceBtn.setStyle(&btnBlueStyle);

	cycleLeftBtn = Button(
	        autonsTab, "<",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        auto& autons = autonSelector->getSelectedAutons();
		        if (autons.size() <= 1) { return; }

		        autons[autonSelector->autonIndex].setHidden(true);
		        autonSelector->autonIndex = util::normalize(autonSelector->autonIndex - 1, (int) autons.size());
		        autons[autonSelector->autonIndex].setHidden(false);
	        },
	        this);
	cycleLeftBtn.setPos(0, 0);
	cycleLeftBtn.setSize(130, 150);

	cycleRightBtn = Button(
	        autonsTab, ">",
	        [](lv_event_t* event) -> void {
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
		        auto& autons = autonSelector->getSelectedAutons();
		        if (autons.size() <= 1) { return; }

		        autons[autonSelector->autonIndex].setHidden(true);
		        autonSelector->autonIndex = util::normalize(autonSelector->autonIndex + 1, (int) autons.size());
		        autons[autonSelector->autonIndex].setHidden(false);
	        },
	        this);
	cycleRightBtn.setPos(0, 0);
	cycleRightBtn.setSize(130, 150);


	// add callback for when tabview changes tab -> to dynamically update the auton which is shown
	lv_obj_add_event_cb(
	        tabview,
	        [](lv_event_t* event) -> void {
		        auto* tabviewObj = lv_event_get_target(event);
		        auto* autonSelector = static_cast<AutonSelector*>(lv_event_get_user_data(event));

		        uint16_t tabID = lv_tabview_get_tab_act(tabviewObj);
		        if (tabID == 2) {
			        // dynamically load what autons to show depending on what's been chosen
			        if (robotInstance->curAlliance == Alliance::INVALID) { return; }


			        lv_obj_add_flag(autonSelector->redElimAutonsParent, LV_OBJ_FLAG_HIDDEN);
			        lv_obj_add_flag(autonSelector->redQualAutonsParent, LV_OBJ_FLAG_HIDDEN);
			        lv_obj_add_flag(autonSelector->blueElimAutonsParent, LV_OBJ_FLAG_HIDDEN);
			        lv_obj_add_flag(autonSelector->blueQualAutonsParent, LV_OBJ_FLAG_HIDDEN);

			        lv_obj_clear_flag(autonSelector->getAutonParentObj(), LV_OBJ_FLAG_HIDDEN);

			        // display only the currently selected auton button
			        auto& autons = autonSelector->getSelectedAutons();
			        if (!autons.empty()) {
				        // shows only the current cycled auton's button
				        autonSelector->autonIndex = std::clamp(autonSelector->autonIndex, 0, (int) autons.size() - 1);
				        for (Button& autonBtn : autons) { autonBtn.setHidden(true); }
				        autons[autonSelector->autonIndex].setHidden(false);
			        }
		        }
	        },
	        LV_EVENT_VALUE_CHANGED, this);


	// create buttons

	// blue -> ringside elim
	// B -> ringside qual
	// R -> ringside elim
	// R -> ringisde qual

	// blueRingsideElim = Button(selectorScreen, "Blue Elim", autonBtnCallback, Auton::ELIM, Alliance::BLUE);
	// blueRingsideElim.setPos(percentX(10), percentY(5));
	// blueRingsideElim.setSize(150, 100);
	// blueRingsideQual = Button(selectorScreen, "Blue Qual", autonBtnCallback, Auton::QUAL, Alliance::BLUE);
	// blueRingsideQual.setPos(percentX(65), percentY(5));
	// blueRingsideQual.setSize(150, 100);
	// redRingsideElim = Button(selectorScreen, "Red Elim", autonBtnCallback, Auton::ELIM, Alliance::RED);
	// redRingsideElim.setPos(percentX(10), percentY(50));
	// redRingsideElim.setSize(150, 100);
	// redRingsideElim.setStyle(&btnRedStyle);
	// redRingsideQual = Button(selectorScreen, "Red Qual", autonBtnCallback, Auton::QUAL, Alliance::RED);
	// redRingsideQual.setPos(percentX(65), percentY(50));
	// redRingsideQual.setSize(150, 100);
	// redRingsideQual.setStyle(&btnRedStyle);

	lv_scr_load(selectorScreen);

	// uninstalls the entire pipeline incase someone forgets to select an auton
	while (robotInstance->autonFnPtr == nullptr && pros::competition::is_disabled()) { pros::delay(10); }
	lv_scr_load(lcdScreen);
}

void AutonSelector::addAuton(const char* name, bool isRed, bool isQual, AutonFn_t autonFnPtr) {
	if (autonsTab == nullptr) { return; }// auton selector isn't initialized

	lv_obj_t* parentObj = isQual ? (isRed ? redQualAutonsParent : blueQualAutonsParent)
	                             : (isRed ? redElimAutonsParent : blueElimAutonsParent);
	auto& autons = isQual ? (isRed ? redQualAutons : blueQualAutons) : (isRed ? redElimAutons : blueElimAutons);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"
#pragma GCC diagnostic ignored "-fpermissive"
#pragma GCC diagnostic ignored "-pedantic"
	Button button = Button(
	        parentObj, name,
	        [](lv_event_t* event) -> void {
		        // robotInstance->autonFnPtr = static_cast<AutonFn_t>(lv_event_get_user_data(event));
		        //
	        }, nullptr
	        /*autonFnPtr*/);
#pragma GCC diagnostic pop
	button.setSize(175, 150);
	button.setPos(0, 0);
	// button.setPos(140, -10);
	if (isRed) {
		button.setStyle(&btnRedStyle);
	} else {
		button.setStyle(&btnBlueStyle);
	}
	button.setHidden(true);

	autons.push_back(button);
}

std::vector<AutonSelector::Button>& AutonSelector::getSelectedAutons() {
	if (robotInstance->isElim) {
		if (robotInstance->curAlliance == Alliance::RED) { return redElimAutons; }
		return blueElimAutons;
	} else {
		if (robotInstance->curAlliance == Alliance::RED) { return redQualAutons; }
		return blueQualAutons;
	}
}

lv_obj_t* AutonSelector::getAutonParentObj() {
	if (robotInstance->isElim) {
		if (robotInstance->curAlliance == Alliance::RED) { return redElimAutonsParent; }
		return blueElimAutonsParent;
	} else {
		if (robotInstance->curAlliance == Alliance::RED) { return redQualAutonsParent; }
		return blueQualAutonsParent;
	}
}