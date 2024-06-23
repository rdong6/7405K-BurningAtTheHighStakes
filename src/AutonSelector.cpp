// #include "AutonSelector.h"
// #include "Robot.h"
// #include "lib/utils/Math.h"
// #include "liblvgl/lv_conf.h"
// #include "liblvgl/lv_core/lv_obj.h"
// #include "liblvgl/lv_core/lv_style.h"
// #include "liblvgl/lv_fonts/lv_font_builtin.h"
// #include "liblvgl/lv_misc/lv_color.h"
// #include "liblvgl/lv_objx/lv_btn.h"
// #include "liblvgl/lv_objx/lv_page.h"
// #include "liblvgl/lv_objx/lv_tabview.h"
// #include "main.h"
// #include "pros/misc.hpp"
// #include <algorithm>
// #include <cstddef>

// extern lv_font_t lv_font_dejavu_40;

// lv_style_t bTGL_Pr;
// lv_style_t bTGL_PrRED;
// lv_style_t bTGL_PrBLUE;
// lv_style_t bTGL_PrGREEN;


// inline lv_coord_t percentX(double percent) {
//     return 480.0 / 100 * percent;
// }

// inline lv_coord_t percentY(double percent) {
//     return 240.0 / 100 * percent;
// }

// // Button class stuff
// /*AutonSelector::Button::Button(lv_obj_t* screen, const char* name, Auton auton) {
//     btn = lv_btn_create(screen, NULL);
//     lv_btn_set_style(btn, LV_BTN_STYLE_REL, &bTGL_Pr);
//     lv_obj_set_size(btn, 130, 200);
//     lv_obj_set_free_num(btn, static_cast<uint32_t>(auton));
//     lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, [](lv_obj_t* btn) -> lv_res_t {
//         uint32_t num = lv_obj_get_free_num(btn);
//         Auton auton = static_cast<Auton>(num);
//         sRobot.setAuton(auton);
//         return LV_RES_OK;
//     });

//     label = lv_label_create(btn, NULL);
//     lv_label_set_text(label, name);
// }

// void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
//     lv_obj_set_pos(btn, x, y);
// }*/

// // New Button code

// AutonSelector::Button::Button(lv_obj_t* parent, const char* name, lv_action_t btnCallback, uint32_t optData) {
// 	btn = lv_btn_create(parent, NULL);
// 	lv_btn_set_style(btn, LV_BTN_STYLE_REL, &bTGL_Pr);
// 	lv_obj_set_free_num(btn, optData);
// 	lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, btnCallback);
// 	label = lv_label_create(btn, NULL);
// 	lv_label_set_text(label, name);
// }

// void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
// 	lv_obj_set_pos(btn, x, y);
// }

// void AutonSelector::Button::setSize(lv_coord_t w, lv_coord_t h) {
// 	lv_obj_set_size(btn, w, h);
// }

// void AutonSelector::Button::setStyle(void* style) {
// 	lv_btn_set_style(btn, LV_BTN_STYLE_REL, (lv_style_t*) style);
// }

// void AutonSelector::Button::setHidden(bool en) {
// 	lv_obj_set_hidden(btn, en);
// }

// // AutonSelector code
// void AutonSelector::initialize() {
// 	lv_style_copy(&bTGL_Pr, &lv_style_btn_tgl_pr);
// 	bTGL_Pr.body.radius = 0;
// 	bTGL_Pr.body.opa = LV_OPA_100;
// 	bTGL_Pr.body.border.color = LV_COLOR_BLACK;
// 	bTGL_Pr.body.border.width = 4;
// 	bTGL_Pr.body.border.opa = LV_OPA_100;
// 	bTGL_Pr.text.font = &lv_font_dejavu_40;

// 	lv_style_copy(&bTGL_PrRED, &bTGL_Pr);
// 	bTGL_PrRED.body.main_color = LV_COLOR_RED;
// 	bTGL_PrRED.body.grad_color = LV_COLOR_RED;


// 	lv_style_copy(&bTGL_PrBLUE, &bTGL_Pr);
// 	bTGL_PrBLUE.body.main_color = LV_COLOR_BLUE;
// 	bTGL_PrBLUE.body.grad_color = LV_COLOR_BLUE;

// 	lv_style_copy(&bTGL_PrGREEN, &bTGL_Pr);
// 	bTGL_PrGREEN.body.main_color = LV_COLOR_GREEN;
// 	bTGL_PrGREEN.body.grad_color = LV_COLOR_GREEN;


// 	pros::lcd::initialize();
// 	lcdScreen = lv_scr_act();

// 	// make auton selector screens
// 	selectorScreen = lv_obj_create(NULL, NULL);


// 	// create the different tabs
// 	tabview = lv_tabview_create(selectorScreen, NULL);
// 	lv_tabview_set_sliding(tabview, false);
// 	lv_tabview_set_anim_time(tabview, 250);
// 	matchTypeTab = lv_tabview_add_tab(tabview, "Match Type");
// 	sideTab = lv_tabview_add_tab(tabview, "Side");
// 	autonsTab = lv_tabview_add_tab(tabview, "Autons");

// 	// callback for when choosing tabs
// 	lv_tabview_set_tab_load_action(tabview, [](lv_obj_t* tabview, uint16_t act_id) -> lv_res_t {
// 		// returning LV_RES_INV prevents tab switching to occur
// 		if (act_id == 1 && !sAutonSelector.choseMatchType) { return LV_RES_INV; }
// 		if (act_id == 2 && !sAutonSelector.choseSide) { return LV_RES_INV; }

// 		if (act_id == 2) {
// 			// showing the autons selector tab
// 			// choose which to display

// 			// display the auton to show
// 			lv_obj_t* autonsButtonParent = sAutonSelector.getCurrentAutonsParent();

// 			lv_obj_set_hidden(sAutonSelector.farElimAutonsParent, true);
// 			lv_obj_set_hidden(sAutonSelector.closeElimAutonsParent, true);

// 			lv_obj_set_hidden(sAutonSelector.farQualAutonsParent, true);
// 			lv_obj_set_hidden(sAutonSelector.closeQualAutonsParent, true);

// 			lv_obj_set_hidden(autonsButtonParent, false);

// 			std::vector<Button>& autons = sAutonSelector.getCurrentAutons();

// 			if (autons.size() > 0) {
// 				sAutonSelector.autonIndex = std::clamp(sAutonSelector.autonIndex, 0, (int) autons.size() - 1);
// 				for (Button& btn : autons) { btn.setHidden(true); }
// 				autons[sAutonSelector.autonIndex].setHidden(false);
// 			}
// 		}
// 		return LV_RES_OK;
// 	});

// 	lv_page_set_sb_mode(matchTypeTab, LV_SB_MODE_OFF);
// 	lv_page_set_sb_mode(sideTab, LV_SB_MODE_OFF);
// 	lv_page_set_sb_mode(autonsTab, LV_SB_MODE_OFF);

// 	// create the buttons for selecting between match and elim autons
// 	qualsBtn = Button(
// 	        matchTypeTab, "Quals",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        sRobot.isQual = lv_obj_get_free_num(btn);
// 		        sAutonSelector.choseMatchType = true;
// 		        sAutonSelector.qualsBtn.setStyle(&bTGL_PrGREEN);
// 		        sAutonSelector.elimBtn.setStyle(&bTGL_PrBLUE);
// 		        lv_tabview_set_tab_act(sAutonSelector.tabview, 1, true);// go to auton side selection page
// 		        return LV_RES_OK;
// 	        },
// 	        1);

// 	qualsBtn.setPos(20, -10);
// 	qualsBtn.setSize(175, 150);
// 	qualsBtn.setStyle(&bTGL_PrRED);

// 	elimBtn = Button(
// 	        matchTypeTab, "Elims",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        sRobot.isQual = lv_obj_get_free_num(btn);
// 		        sAutonSelector.choseMatchType = true;
// 		        sAutonSelector.qualsBtn.setStyle(&bTGL_PrRED);
// 		        sAutonSelector.elimBtn.setStyle(&bTGL_PrGREEN);
// 		        lv_tabview_set_tab_act(sAutonSelector.tabview, 1, true);// go to auton side selection page
// 		        return LV_RES_OK;
// 	        },
// 	        0);
// 	elimBtn.setPos(275, 20);
// 	elimBtn.setSize(175, 150);
// 	elimBtn.setStyle(&bTGL_PrBLUE);


// 	// create the buttons for selecting between close and far autons
// 	closeBtn = Button(
// 	        sideTab, "Close",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        sAutonSelector.isFar = lv_obj_get_free_num(btn);
// 		        sAutonSelector.closeBtn.setStyle(&bTGL_PrGREEN);
// 		        sAutonSelector.farBtn.setStyle(&bTGL_PrBLUE);
// 		        sAutonSelector.choseSide = true;
// 		        lv_tabview_set_tab_act(sAutonSelector.tabview, 2, true);// now go to auton selection page
// 		        return LV_RES_OK;
// 	        },
// 	        0);

// 	closeBtn.setPos(20, -10);
// 	closeBtn.setSize(175, 150);
// 	closeBtn.setStyle(&bTGL_PrRED);


// 	farBtn = Button(
// 	        sideTab, "Far",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        sAutonSelector.isFar = lv_obj_get_free_num(btn);
// 		        sAutonSelector.closeBtn.setStyle(&bTGL_PrRED);
// 		        sAutonSelector.farBtn.setStyle(&bTGL_PrGREEN);
// 		        sAutonSelector.choseSide = true;
// 		        lv_tabview_set_tab_act(sAutonSelector.tabview, 2, true);// now go to auton selection page
// 		        return LV_RES_OK;
// 	        },
// 	        1);
// 	farBtn.setPos(275, 20);
// 	farBtn.setSize(175, 150);
// 	farBtn.setStyle(&bTGL_PrBLUE);

// 	// create parent container objects for the auton selector buttons
// 	farElimAutonsParent = lv_obj_create(autonsTab, NULL);
// 	lv_obj_set_hidden(farElimAutonsParent, true);// hide all the auton selector buttons as of now
// 	lv_obj_set_pos(farElimAutonsParent, 145, -10);
// 	lv_obj_set_size(farElimAutonsParent, 175, 150);
// 	farQualAutonsParent = lv_obj_create(autonsTab, farElimAutonsParent);
// 	closeElimAutonsParent = lv_obj_create(autonsTab, farElimAutonsParent);
// 	closeQualAutonsParent = lv_obj_create(autonsTab, farElimAutonsParent);

// 	cycleLeftButton = Button(
// 	        autonsTab, "<",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        std::vector<Button>& autons = sAutonSelector.getCurrentAutons();
// 		        if (autons.size() > 1) {
// 			        int shiftAmt = lv_obj_get_free_num(btn);
// 			        autons[sAutonSelector.autonIndex].setHidden(true);
// 			        sAutonSelector.autonIndex =
// 			                util::normalize(sAutonSelector.autonIndex + shiftAmt, (int) autons.size());
// 			        autons[sAutonSelector.autonIndex].setHidden(false);
// 		        }
// 		        return LV_RES_OK;
// 	        },
// 	        static_cast<uint32_t>(-1));
// 	cycleLeftButton.setPos(0, 0);
// 	cycleLeftButton.setSize(130, 150);

// 	cycleRightButton = Button(
// 	        autonsTab, ">",
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        std::vector<Button>& autons = sAutonSelector.getCurrentAutons();
// 		        if (autons.size() > 1) {
// 			        int shiftAmt = lv_obj_get_free_num(btn);
// 			        autons[sAutonSelector.autonIndex].setHidden(true);
// 			        sAutonSelector.autonIndex =
// 			                util::normalize(sAutonSelector.autonIndex + shiftAmt, (int) autons.size());
// 			        autons[sAutonSelector.autonIndex].setHidden(false);
// 		        }
// 		        return LV_RES_OK;
// 	        },
// 	        1);
// 	cycleRightButton.setPos(330, 20);
// 	cycleRightButton.setSize(130, 150);

// 	// left = Button(selectorScreen, "Left", Auton::LEFT);
// 	// left.setPos(percentX(38), percentY(10));
// 	// right = Button(selectorScreen, "Right", Auton::RIGHT);
// 	// right.setPos(percentX(75), percentY(10));
// 	// carry = Button(selectorScreen, "Disruptor", Auton::CARRY);
// 	// carry.setPos(percentX(5), percentY(10));
// }

// void AutonSelector::addAuton(const char* name, bool isQual, bool isFar, Auton auton) {
// 	if (autonsTab == nullptr) { return; }// not initialized, don't add autons

// 	lv_obj_t* parentObj = isQual  ? isFar ? farQualAutonsParent : closeQualAutonsParent
// 	                      : isFar ? farElimAutonsParent
// 	                              : closeElimAutonsParent;
// 	std::vector<Button>& autons = isQual  ? isFar ? farQualAutons : closeQualAutons
// 	                              : isFar ? farElimAutons
// 	                                      : closeElimAutons;

// 	Button button = Button(
// 	        parentObj, name,
// 	        [](lv_obj_t* btn) -> lv_res_t {
// 		        sRobot.setAuton(static_cast<Auton>(lv_obj_get_free_num(btn)));
// 		        return LV_RES_OK;
// 	        },
// 	        static_cast<uint32_t>(auton));
// 	button.setSize(175, 150);
// 	button.setPos(0, 0);
// 	// button.setPos(140, -10);
// 	if (isQual) {
// 		button.setStyle(&bTGL_PrRED);
// 	} else {
// 		button.setStyle(&bTGL_PrBLUE);
// 	}
// 	button.setHidden(true);

// 	autons.push_back(button);
// }

// void AutonSelector::showSelector() {
// 	lv_scr_load(selectorScreen);

// 	// unstalls the entire pipeline incase someone forgets to select an auton
// 	// temporarily commented out
// 	while (sRobot.getAuton() == Auton::NONE && pros::competition::is_disabled()) { pros::delay(10); }
// }

// void AutonSelector::restoreLCD() {
// 	lv_scr_load(lcdScreen);
// }